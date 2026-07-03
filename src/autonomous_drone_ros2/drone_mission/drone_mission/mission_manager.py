#!/usr/bin/env python3
"""
mission_manager.py | Package: drone_mission
Full mission orchestration: A → B → C → D → Return to A
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry

TAKEOFF_ALT     = 5.0
WAIT_ON_GROUND  = 5.0
ARM_TIMEOUT     = 10.0
TAKEOFF_TIMEOUT = 20.0
NAV_TIMEOUT     = 60.0
LAND_TIMEOUT    = 30.0

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
    COMPLETE       = "MISSION_COMPLETE"
    ABORT          = "MISSION_ABORT"

class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        # ── Mission sequence — edit to add/remove waypoints ──
        self.mission_sequence = [
            ("B", "DELIVERY_B"),
            ("C", "DELIVERY_C"),
            ("D", "DELIVERY_D"),
        ]
        self.wp_index    = 0
        self.state       = MS.IDLE
        self.state_start = None
        self.wait_start  = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_subscription(String,   "/drone_base/status",    self._base_cb,    10)
        self.create_subscription(String,   "/waypoint_nav/status",  self._nav_cb,     10)
        self.create_subscription(String,   "/aruco_landing/status", self._aruco_cb,   10)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self._odom_cb, sensor_qos)
        self.create_subscription(String,   "/mission/command",      self._mission_cmd, 10)

        self.base_cmd_pub  = self.create_publisher(String, "/drone_base/command",    10)
        self.nav_cmd_pub   = self.create_publisher(String, "/waypoint_nav/command",  10)
        self.aruco_cmd_pub = self.create_publisher(String, "/aruco_landing/command", 10)
        self.status_pub    = self.create_publisher(String, "/mission/status",        10)

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.base_status  = "DISCONNECTED"
        self.nav_status   = "IDLE"
        self.aruco_status = "IDLE"
        self.altitude     = 0.0

        self.create_timer(0.2, self._loop)
        self.create_timer(1.0, self._log)
        self.get_logger().info("MissionManager ready. Publish 'START' to /mission/command")

    def _base_cb(self,  msg): self.base_status  = msg.data
    def _nav_cb(self,   msg): self.nav_status   = msg.data
    def _aruco_cb(self, msg): self.aruco_status = msg.data
    def _odom_cb(self,  msg): self.altitude     = msg.pose.pose.position.z

    def _mission_cmd(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "START" and self.state == MS.IDLE:
            self.get_logger().info("🚀 Mission START")
            self._enter(MS.PREFLIGHT)
        elif cmd == "ABORT":
            self._abort()

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

    def _preflight(self):
        if self.base_status == "DISCONNECTED":
            self.get_logger().warn("Waiting for MAVROS...", throttle_duration_sec=3.0); return
        if self._elapsed() > ARM_TIMEOUT:
            self._abort(); return
        self.get_logger().info("✅ Preflight OK → TAKEOFF")
        self._send_base("TAKEOFF")
        self._enter(MS.TAKEOFF)

    # def _takeoff(self):
    #     if self._elapsed() > TAKEOFF_TIMEOUT:
    #         self._abort(); return
    #     if self.base_status == "AIRBORNE" and self.altitude >= TAKEOFF_ALT * 0.85:
    #         self.get_logger().info(f"✅ Airborne {self.altitude:.1f}m → first waypoint")
    #         self._goto_next()
    def _takeoff(self):
        if self._elapsed() > TAKEOFF_TIMEOUT:
            self.get_logger().error("Takeoff timeout")
            self._abort()
            return

        # Wait until the drone is safely airborne
        if self.base_status == "AIRBORNE" and self.altitude >= 1.5:
            self.get_logger().info(
                f"✅ Airborne ({self.altitude:.1f} m) → First Waypoint"
            )
            self._goto_next()

    def _goto(self):
        if self._elapsed() > NAV_TIMEOUT:
            self._abort(); return
        if self.nav_status == "ARRIVED":
            _,label = self.mission_sequence[self.wp_index]
            self.get_logger().info(f"✅ Arrived {label} → ArUco landing")
            self._send_nav("STOP")
            self._send_aruco("START")
            self._enter(MS.ARUCO_LAND)
        elif self.nav_status == "TIMEOUT":
            self._goto_next()

    def _aruco_land(self):
        if self._elapsed() > LAND_TIMEOUT:
            self._abort(); return
        if self.aruco_status == "COMPLETE":
            self.get_logger().info("✅ ArUco COMPLETE → AUTO.LAND")
            self._auto_land()
            self._enter(MS.WAIT_ON_GROUND)
        elif self.aruco_status == "FAILED":
            self._abort()

    def _wait(self):
        if self.wait_start is None:
            self.wait_start = self.get_clock().now()
            _,label = self.mission_sequence[self.wp_index]
            self.get_logger().info(f"📦 Payload drop at {label} — waiting {WAIT_ON_GROUND}s")
            return
        elapsed = (self.get_clock().now()-self.wait_start).nanoseconds*1e-9
        if elapsed >= WAIT_ON_GROUND:
            self.wait_start = None
            self.wp_index  += 1
            self._send_base("TAKEOFF")
            self._enter(MS.INTER_TAKEOFF)

    def _inter_takeoff(self):
        if self._elapsed() > TAKEOFF_TIMEOUT:
            self._abort(); return
        if self.base_status == "AIRBORNE" and self.altitude >= TAKEOFF_ALT * 0.85:
            if self.wp_index < len(self.mission_sequence):
                self._goto_next()
            else:
                self.get_logger().info("🏠 All deliveries done → Return Home")
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
            self.get_logger().info("🏁 Mission complete!")
            self._enter(MS.COMPLETE)

    def _goto_next(self):
        key,label = self.mission_sequence[self.wp_index]
        self.get_logger().info(f"📍 Navigating to {label}")
        self._send_nav(f"GOTO:{key}")
        self._enter(MS.GOTO_WAYPOINT)

    def _auto_land(self):
        req = SetMode.Request(); req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)

    def _abort(self):
        self.get_logger().error("🛑 ABORT")
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
