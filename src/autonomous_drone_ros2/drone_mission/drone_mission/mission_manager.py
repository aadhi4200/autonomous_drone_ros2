#!/usr/bin/env python3
"""
mission_manager.py | Package: drone_mission
Full mission orchestration: A → B → C → D → Return to A

Mission sequence and per-stop marker IDs come from whatever the frontend
uploaded on /mission/waypoints (order = array order sent by the website);
falls back to the original A->B->C->D default only if nothing was uploaded.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry

from drone_interfaces.constants import TOPIC_MISSION_WAYPOINTS, ARUCO_MARKER_ID

TAKEOFF_ALT     = 5.0
WAIT_ON_GROUND  = 5.0
ARM_TIMEOUT     = 10.0
TAKEOFF_TIMEOUT = 20.0
NAV_TIMEOUT     = 120.0  # must cover the navigator's accel-limited profile (its WP_TIMEOUT is also 120)
LAND_TIMEOUT    = 45.0  # must outlast the aruco node's own 30s search
                        # timeout, or ARUCO_LAND's abort fires before the
                        # FAILED fallback can run (raced live 2026-07-12)

# Default (no upload) mission is a single A->B->home hop per operator
# request 2026-07-11 -- C/D stops only fly when the website explicitly
# uploads them (the uploaded sequence always replaces this default).
DEFAULT_SEQUENCE = [
    ("B", "DELIVERY_B"),
]

class MS:
    IDLE           = "IDLE"
    PREFLIGHT      = "PREFLIGHT"
    TAKEOFF        = "TAKEOFF"
    GOTO_WAYPOINT  = "GOTO_WAYPOINT"
    ARUCO_LAND     = "ARUCO_LAND"
    WAIT_ON_GROUND = "WAIT_ON_GROUND"
    INTER_TAKEOFF  = "INTER_TAKEOFF"
    RETURN_HOME    = "RETURN_HOME"
    HOME_LAND      = "HOME_LAND"
    PX4_FAILSAFE   = "PX4_FAILSAFE"   # PX4 initiated RTL itself (geofence breach etc.) — stand down
    COMPLETE       = "MISSION_COMPLETE"
    ABORT          = "MISSION_ABORT"

class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.mission_sequence = list(DEFAULT_SEQUENCE)
        self.marker_ids  = {}   # label -> marker_id, from the frontend upload
        self.uploaded    = False
        self.wp_index    = 0
        self.state       = MS.IDLE
        self.state_start = None
        self.wait_start  = None
        self.rth_reason  = None   # set when RETURN_HOME was entered due to a failsafe RTH, not mission end
        self.ground_alt  = 0.0    # believed z while on the ground at the last takeoff -- the
                                  # accumulated EKF z-drift; all climb gates measure against it

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_subscription(String,   "/drone_base/status",    self._base_cb,    10)
        self.create_subscription(String,   "/waypoint_nav/status",  self._nav_cb,     10)
        self.create_subscription(String,   "/aruco_landing/status", self._aruco_cb,   10)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._odom_cb, sensor_qos)
        self.create_subscription(State,    "/mavros/state",               self._mavros_state_cb, sensor_qos)
        self.create_subscription(String,   "/mission/command",      self._mission_cmd, 10)
        self.create_subscription(String,   TOPIC_MISSION_WAYPOINTS, self._waypoints_cb, 10)

        self.base_cmd_pub  = self.create_publisher(String, "/drone_base/command",    10)
        self.ground_z_pub = self.create_publisher(Float64, "/mission/ground_z", 10)
        self.nav_cmd_pub   = self.create_publisher(String, "/waypoint_nav/command",  10)
        self.aruco_cmd_pub = self.create_publisher(String, "/aruco_landing/command", 10)
        self.status_pub    = self.create_publisher(String, "/mission/status",        10)

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.base_status  = "DISCONNECTED"
        self.nav_status   = "IDLE"
        self.aruco_status = "IDLE"
        self.altitude     = 0.0
        self.flight_mode  = ""

        self.create_timer(0.2, self._loop)
        self.create_timer(1.0, self._log)
        # Periodic heartbeat re-publish (not just on state change) so a
        # frozen/crashed mission_manager is distinguishable from one that
        # simply has nothing new to report (see connectivity-gate feature).
        self.create_timer(0.5, lambda: self.status_pub.publish(String(data=self.state)))
        self.get_logger().info("MissionManager ready. Publish 'START' to /mission/command")

    def _base_cb(self,  msg): self.base_status  = msg.data
    def _nav_cb(self,   msg): self.nav_status   = msg.data
    def _aruco_cb(self, msg): self.aruco_status = msg.data
    def _odom_cb(self,  msg): self.altitude     = msg.pose.pose.position.z

    def _mavros_state_cb(self, msg):
        prev = self.flight_mode
        self.flight_mode = msg.mode
        # PX4 switching itself to AUTO.RTL mid-mission means one of ITS OWN
        # failsafes fired (geofence breach, RC loss, etc.). This node never
        # commands RTL (only AUTO.LAND), so an uncommanded RTL is always
        # PX4's decision — stand down and let it fly home rather than
        # fighting it with an in-place-land abort (observed live 2026-07-12:
        # a geofence breach RTL was cut short by exactly that abort).
        active = self.state not in (MS.IDLE, MS.COMPLETE, MS.ABORT, MS.PX4_FAILSAFE)
        if msg.mode == "AUTO.RTL" and prev != "AUTO.RTL" and active:
            self._px4_failsafe_rtl()

    def _waypoints_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Bad /mission/waypoints payload: {e}")
            return
        self.mission_sequence = [(wp.get("label", "B"), f"DELIVERY_{wp.get('label','B')}") for wp in data]
        self.marker_ids = {wp.get("label", "B"): wp.get("marker_id") for wp in data}
        self.uploaded = True
        self.get_logger().info(f"Mission sequence from upload: {[k for k,_ in self.mission_sequence]}")

    def _mission_cmd(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "START" and self.state == MS.IDLE:
            self.get_logger().info("🚀 Mission START")
            self._enter(MS.PREFLIGHT)
        elif cmd == "ABORT":
            self._abort()
        elif cmd.startswith("RTH:"):
            # Failsafe-triggered return-to-home (comms/node loss) — distinct
            # from a manual/battery ABORT, which lands in place instead.
            reason = cmd.split(":", 1)[1]
            self._trigger_rth(reason)
        elif cmd == "RESET" and self.state in (MS.COMPLETE, MS.ABORT):
            # Nothing ever transitioned COMPLETE/ABORT back to IDLE on its
            # own -- confirmed live 2026-07-10: every "START" after a
            # drone's very first mission was silently ignored forever
            # (self.state == MS.IDLE guard above never matched again),
            # which reads exactly like "not taking off" from the website
            # even though the whole stack was healthy. Only reachable from
            # the two genuinely terminal states -- not from mid-flight
            # states, so this can't be used to interrupt an active mission.
            self.get_logger().info("🔄 Mission RESET → IDLE")
            self.wp_index = 0
            self.rth_reason = None
            self._enter(MS.IDLE)

    def _loop(self):
        if   self.state == MS.IDLE:          return
        elif self.state == MS.PREFLIGHT:     self._preflight()
        elif self.state == MS.TAKEOFF:       self._takeoff()
        elif self.state == MS.GOTO_WAYPOINT: self._goto()
        elif self.state == MS.ARUCO_LAND:    self._aruco_land()
        elif self.state == MS.WAIT_ON_GROUND:self._wait()
        elif self.state == MS.INTER_TAKEOFF: self._inter_takeoff()
        elif self.state == MS.RETURN_HOME:   self._return_home()
        elif self.state == MS.HOME_LAND:     self._home_land()
        elif self.state == MS.PX4_FAILSAFE:  self._px4_failsafe_wait()

    def _preflight(self):
        if self.base_status == "DISCONNECTED":
            self.get_logger().warn("Waiting for MAVROS...", throttle_duration_sec=3.0); return
        if self._elapsed() > ARM_TIMEOUT:
            self._abort(); return
        self.get_logger().info("✅ Preflight OK → TAKEOFF")
        self._record_ground_alt()
        self._send_base("TAKEOFF")
        self._enter(MS.TAKEOFF)

    def _takeoff(self):
        if self._elapsed() > TAKEOFF_TIMEOUT:
            self.get_logger().error("Takeoff timeout")
            self._abort()
            return
        if self.base_status == "AIRBORNE" and (self.altitude - self.ground_alt) >= 1.5:
            self.get_logger().info(
                f"✅ Airborne ({self.altitude - self.ground_alt:.1f} m above ground) → First Waypoint"
            )
            self._goto_next()

    def _goto(self):
        if self._elapsed() > NAV_TIMEOUT:
            self._abort(); return
        if self.nav_status == "ARRIVED":
            key, label = self.mission_sequence[self.wp_index]
            self.get_logger().info(f"✅ Arrived {label} → ArUco landing")
            self._send_nav("STOP")
            marker_id = self.marker_ids.get(key) or ARUCO_MARKER_ID
            self._send_aruco(f"START:{marker_id}")
            self._enter(MS.ARUCO_LAND)
        elif self.nav_status == "TIMEOUT":
            self._goto_next()

    def _aruco_land(self):
        # Status checks come BEFORE the timeout: the vision search runs a
        # full 30s before reporting FAILED, so a timeout checked first (and
        # equal to that 30s) aborted the mission 0.2s before the FAILED
        # fallback could ever run -- hit live 2026-07-12.
        if self.aruco_status == "COMPLETE":
            self.get_logger().info("✅ ArUco COMPLETE → AUTO.LAND")
            self._auto_land()
            # Reset the aruco node to IDLE now: drone_base suppresses its own
            # setpoint stream while aruco status != IDLE, so leaving it in
            # COMPLETE starves OFFBOARD of setpoints and the next
            # INTER_TAKEOFF can never arm (found live 2026-07-13, the first
            # time the COMPLETE path ever ran).
            self._send_aruco("ABORT")
            self._enter(MS.WAIT_ON_GROUND)
            return
        if self.aruco_status == "FAILED":
            # No marker found (camera can't render in WSL, or no pad was
            # generated for this stop) is NOT a mission-ending emergency:
            # land on the GPS waypoint without vision guidance and carry on
            # with the rest of the mission (next stop / return home).
            _, label = self.mission_sequence[self.wp_index]
            self.get_logger().warn(
                f"ArUco marker not found at {label} — landing on GPS "
                "position without vision guidance and continuing the mission")
            self._send_aruco("ABORT")  # the aruco node has no STOP cmd; ABORT = reset to IDLE + zero velocity
            self._auto_land()
            self._enter(MS.WAIT_ON_GROUND)
            return
        if self._elapsed() > LAND_TIMEOUT:
            self._abort()

    def _wait(self):
        if self.base_status != "LANDED":
            return  # AUTO.LAND still descending — don't start the clock early
        if self.wait_start is None:
            self.wait_start = self.get_clock().now()
            _,label = self.mission_sequence[self.wp_index]
            self.get_logger().info(f"📦 Payload drop at {label} — waiting {WAIT_ON_GROUND}s")
            return
        elapsed = (self.get_clock().now()-self.wait_start).nanoseconds*1e-9
        if elapsed >= WAIT_ON_GROUND:
            self.wait_start = None
            self.wp_index  += 1
            self._record_ground_alt()
            self._send_base("TAKEOFF")
            self._enter(MS.INTER_TAKEOFF)

    def _inter_takeoff(self):
        if self._elapsed() > TAKEOFF_TIMEOUT:
            self._abort(); return
        if self.base_status == "AIRBORNE" and (self.altitude - self.ground_alt) >= TAKEOFF_ALT * 0.85:
            if self.wp_index < len(self.mission_sequence):
                self._goto_next()
            else:
                self.get_logger().info("🏠 All deliveries done → Return Home")
                self.rth_reason = None
                self._send_nav("GOTO:A")
                self._enter(MS.RETURN_HOME)

    def _return_home(self):
        if self._elapsed() > NAV_TIMEOUT:
            self._abort(); return
        if self.nav_status == "ARRIVED":
            self.get_logger().info("✅ Home reached → final land")
            self._send_nav("STOP")
            self._auto_land()
            self._enter(MS.HOME_LAND)

    def _home_land(self):
        if self._elapsed() > LAND_TIMEOUT:
            self._abort(); return
        if self.base_status == "LANDED":
            if self.rth_reason:
                self.get_logger().info(f"🏁 Failsafe RTH complete (reason={self.rth_reason})")
            else:
                self.get_logger().info("🏁 Mission complete!")
            self._enter(MS.COMPLETE)

    def _record_ground_alt(self):
        """Snapshot the believed altitude while on the ground and broadcast it.
        In SITL the EKF z estimate drifts a few metres with every landing
        cycle; measuring climbs (and the navigator's target altitudes)
        against this snapshot instead of absolute zero keeps consecutive
        missions flying after the drift accumulates."""
        self.ground_alt = self.altitude
        self.ground_z_pub.publish(Float64(data=self.ground_alt))
        if abs(self.ground_alt) > 1.0:
            self.get_logger().warn(
                f"EKF z-drift: ground reads {self.ground_alt:.1f}m — climb gates now relative to it")

    def _goto_next(self):
        key,label = self.mission_sequence[self.wp_index]
        self.get_logger().info(f"📍 Navigating to {label}")
        self._send_nav(f"GOTO:{key}")
        self._enter(MS.GOTO_WAYPOINT)

    def _auto_land(self):
        req = SetMode.Request(); req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)

    def _trigger_rth(self, reason: str):
        """Return-to-home-and-land, for connectivity-loss failsafes. Distinct
        from _abort(): this flies home first instead of landing in place."""
        if self.state in (MS.IDLE, MS.COMPLETE, MS.ABORT, MS.RETURN_HOME, MS.HOME_LAND, MS.PX4_FAILSAFE):
            return
        self.get_logger().error(f"🛑 FAILSAFE RTH triggered (reason={reason})")
        self.rth_reason = reason
        self._send_aruco("ABORT")
        self._send_nav("GOTO:A")
        self._enter(MS.RETURN_HOME)

    def _px4_failsafe_rtl(self):
        """PX4 fired its own failsafe RTL (geofence breach, etc.). Stop our
        own commanding (setpoint stream, aruco search) and wait for PX4 to
        finish flying home and landing. Deliberately NO timeout-abort here:
        commanding AUTO.LAND while PX4 is mid-return would land the drone
        wherever it happens to be — the exact thing RTL exists to avoid."""
        self.get_logger().error(
            f"🛑 PX4 FAILSAFE RTL detected (mode=AUTO.RTL, state={self.state}) — "
            "standing down, PX4 owns the flight home.")
        self.rth_reason = "PX4_FAILSAFE_RTL"
        self._send_aruco("ABORT")
        self._send_nav("STOP")
        self._enter(MS.PX4_FAILSAFE)

    def _px4_failsafe_wait(self):
        if self.base_status == "LANDED":
            self.get_logger().info("🏁 PX4 failsafe RTL complete — landed.")
            self._enter(MS.COMPLETE)

    def _abort(self):
        """Immediate in-place land — for explicit operator abort, timeouts,
        and low-battery emergencies where flying further/home is itself
        unsafe. Comms/node-loss uses _trigger_rth() instead."""
        self.get_logger().error("🛑 ABORT (in-place land)")
        self._send_aruco("ABORT")
        self._send_nav("STOP")
        self._auto_land()
        self._enter(MS.ABORT)

    def _send_base(self, cmd):
        msg = String(); msg.data = cmd; self.base_cmd_pub.publish(msg)
    def _send_nav(self, cmd):
        msg = String(); msg.data = cmd; self.nav_cmd_pub.publish(msg)
    def _send_aruco(self, cmd):
        msg = String(); msg.data = cmd; self.aruco_cmd_pub.publish(msg)

    def _enter(self, new):
        self.get_logger().info(f"[MISSION] {self.state} → {new}")
        self.state = new; self.state_start = self.get_clock().now()
        self.wait_start = None
        msg = String(); msg.data = new; self.status_pub.publish(msg)

    def _elapsed(self):
        if self.state_start is None: return 0.0
        return (self.get_clock().now()-self.state_start).nanoseconds*1e-9

    def _log(self):
        wp = self.mission_sequence[self.wp_index][0] if self.wp_index < len(self.mission_sequence) else "HOME"
        self.get_logger().info(
            f"[MISSION] {self.state} | wp={wp} | alt={self.altitude:.1f}m | "
            f"base={self.base_status} | nav={self.nav_status} | aruco={self.aruco_status}",
            throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
