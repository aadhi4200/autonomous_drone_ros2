#!/usr/bin/env python3
"""
drone_base_node.py | Package: drone_base
Fixed version - properly handles MAVROS state connection
"""
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

TAKEOFF_ALT = 5.0
CLIMB_RATE  = 1.0   # m/s — the altitude setpoint ramps at this rate instead
NUDGE_STEP_M   = 0.6   # meters per manual-nudge press
YAW_STEP_RAD   = 0.26  # ~15 degrees per manual yaw-nudge press
                    # of stepping straight to TAKEOFF_ALT (a step commands
                    # PX4's max climb and kicks the IMU; gradual is the
                    # real-hardware requirement)
SETPOINT_HZ = 20.0

class DroneBaseNode(Node):
    STATUS_DISCONNECTED = "DISCONNECTED"
    STATUS_CONNECTED    = "CONNECTED"
    STATUS_ARMED        = "ARMED"
    STATUS_AIRBORNE     = "AIRBORNE"
    STATUS_LANDING      = "LANDING"
    STATUS_LANDED       = "LANDED"

    def __init__(self):
        super().__init__("drone_base_node")
        self.declare_parameter("takeoff_altitude", TAKEOFF_ALT)
        self.takeoff_altitude = self.get_parameter("takeoff_altitude").value

        # ── BEST_EFFORT QoS for ALL MAVROS topics ──────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers ────────────────────────────────────────
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            sensor_qos)          # ← MUST be BEST_EFFORT

        self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self._odom_cb,
            sensor_qos)          # ← MUST be BEST_EFFORT

        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._home_cb,
            sensor_qos)          # ← MUST be BEST_EFFORT

        self.create_subscription(
            String,
            "/drone_base/command",
            self._cmd_cb,
            10)                  # ← RELIABLE for commands

        # Manual bench/real-drone-test control (Flight Test Bench page):
        # directional nudges, isolated on their own topic so they can never
        # collide with mission-critical TAKEOFF/LAND/ARM/DISARM commands on
        # /drone_base/command.
        self.create_subscription(
            String,
            "/manual/nudge",
            self._manual_nudge_cb,
            10)

        # Peer controllers publish their state every 0.5s (see mission_manager's
        # handoff via /waypoint_nav/command, /aruco_landing/command). While either
        # is active, IT owns /mavros/setpoint_position/local — this node must not
        # also stream setpoints, or the two publishers race on the same topic.
        self.create_subscription(
            String,
            "/waypoint_nav/status",
            self._nav_status_cb,
            10)

        self.create_subscription(
            String,
            "/aruco_landing/status",
            self._landing_status_cb,
            10)

        # ── Publishers ─────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10)

        self.status_pub = self.create_publisher(
            String,
            "/drone_base/status",
            10)

        # ── Service clients ────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Internal state ─────────────────────────────────────
        self.vehicle_state = State()
        self.current_pos   = None
        self.home_lat      = None
        self.home_lon      = None
        self.home_alt      = None
        self.drone_status  = self.STATUS_DISCONNECTED
        self._sp_x         = 0.0
        self._sp_y         = 0.0
        self._sp_z         = 0.0   # stream ground level until takeoff is commanded
        self._sp_z_target  = 0.0
        self._nav_active     = False
        self._landing_active = False
        self.current_yaw   = 0.0   # tracked for manual-nudge body->world rotation
        self._sp_yaw        = 0.0  # yaw setpoint; 0.0 preserves the exact
                                    # pre-existing identity-quaternion hold
                                    # behavior until a manual yaw nudge is
                                    # actually issued -- purely additive

        # ── Timers ─────────────────────────────────────────────
        self.create_timer(1.0 / SETPOINT_HZ, self._setpoint_loop)
        self.create_timer(1.0,               self._status_loop)

        self.get_logger().info("DroneBaseNode started — waiting for MAVROS /mavros/state")

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        """Receives MAVROS state — MUST use BEST_EFFORT QoS."""
        prev_connected = self.vehicle_state.connected
        prev_armed     = self.vehicle_state.armed
        self.vehicle_state = msg

        # ANY disarm — commanded or PX4's own auto-disarm after the final
        # mission landing — invalidates the streamed setpoint. After a
        # mission it still pointed at the LAST delivery stop (+takeoff alt),
        # set by the inter-stop _arm_and_offboard and never cleared
        # (auto-disarm skips _disarm()), so the next OFFBOARD entry from the
        # Test Bench flew diagonally to the previous waypoint (seen live
        # 2026-07-17). Re-anchor to wherever the drone actually is.
        if prev_armed and not msg.armed:
            self._anchor_setpoint_here()

        # Log when connection status changes
        if not prev_connected and msg.connected:
            self.get_logger().info(
                f"✅ MAVROS connected! mode={msg.mode}")

        # Update status based on connection + armed state
        if not msg.connected:
            self._update_status(self.STATUS_DISCONNECTED)

        elif msg.connected and not msg.armed:
            # Connected but not armed
            if self.drone_status == self.STATUS_DISCONNECTED:
                self._update_status(self.STATUS_CONNECTED)
            elif self.drone_status in [self.STATUS_LANDING, self.STATUS_AIRBORNE]:
                self._update_status(self.STATUS_LANDED)

        elif msg.connected and msg.armed:
            if self.drone_status not in [self.STATUS_AIRBORNE, self.STATUS_LANDING]:
                self._update_status(self.STATUS_ARMED)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        # Auto-detect airborne
        if (self.vehicle_state.armed
                and p.z > 0.3
                and self.drone_status == self.STATUS_ARMED):
            self._update_status(self.STATUS_AIRBORNE)

    def _nav_status_cb(self, msg: String):
        self._nav_active = (msg.data != "IDLE")

    def _landing_status_cb(self, msg: String):
        self._landing_active = (msg.data != "IDLE")

    def _home_cb(self, msg: HomePosition):
        if self.home_lat is None:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.home_alt = msg.geo.altitude
            self.get_logger().info(
                f"✅ Home GPS: ({self.home_lat:.6f}, {self.home_lon:.6f})")

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[CMD] {cmd}")
        if   cmd == "TAKEOFF": self._arm_and_offboard()
        elif cmd == "LAND":    self._land()
        elif cmd == "ARM":     self._arm()
        elif cmd == "DISARM":  self._disarm()

    # ─────────────────────────────────────────────────────────
    # Flight commands
    # ─────────────────────────────────────────────────────────

    def _arm(self):
        if not self.vehicle_state.connected:
            self.get_logger().error("Cannot arm — MAVROS not connected")
            return
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Arm result: {f.result().success}"))

    def _disarm(self):
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req)
        self.get_logger().info("Disarm requested")
        # Anchor to the actual pose, not (0,0,0): zeros mean "home", which
        # is its own stale-setpoint trap if the drone was disarmed anywhere
        # else (an in-place abort landing at a delivery stop, for example).
        self._anchor_setpoint_here()

    def _anchor_setpoint_here(self):
        """Point the 20Hz OFFBOARD stream at the drone's current pose so a
        later mode switch holds position instead of chasing a stale target."""
        if self.current_pos is not None:
            self._sp_x, self._sp_y, self._sp_z = self.current_pos
        else:
            self._sp_x = self._sp_y = self._sp_z = 0.0
        self._sp_z_target = self._sp_z
        self._sp_yaw = self.current_yaw

    def _manual_nudge_cb(self, msg: String):
        """Bench/real-drone manual control: small position/yaw nudges,
        one per message (the frontend repeats while a button is held).
        Body-frame forward/right, rotated into world ENU by the drone's
        OWN current yaw -- same pattern already used for ArUco alignment
        velocity in aruco_landing_node.
        Never interferes with an active mission: nav_active/landing_active
        already gate the setpoint loop this feeds, and the backend also
        rejects manual commands server-side while a mission is running.
        """
        if self._nav_active or self._landing_active:
            return
        if not self.vehicle_state.armed or self.current_pos is None:
            return
        cmd = msg.data.strip().upper()
        if cmd == "HOLD":
            # Re-anchor to the ACTUAL current pose -- stops any queued
            # nudges from continuing to walk the setpoint further.
            self._sp_x, self._sp_y, self._sp_z = self.current_pos
            self._sp_z_target = self._sp_z
            self._sp_yaw = self.current_yaw
            return
        # Right = 90deg CLOCKWISE from forward (facing East, right is
        # South): world_dx = fwd*cos + right*sin, world_dy = fwd*sin -
        # right*cos. Caught by an isolated unit test 2026-07-13 -- the
        # first version had this sign backwards (LEFT/RIGHT swapped),
        # found because live SITL verification was too flaky (WSL EKF
        # altitude glitches) to trust for this, so the rotation math was
        # proven with a standalone test instead of a live flight.
        c, sn = math.cos(self.current_yaw), math.sin(self.current_yaw)
        def world_delta(fwd, right):
            return (fwd * c + right * sn, fwd * sn - right * c)
        dx = dy = dz = dyaw = 0.0
        if   cmd == "FWD":       dx, dy = world_delta(NUDGE_STEP_M, 0)
        elif cmd == "BACK":      dx, dy = world_delta(-NUDGE_STEP_M, 0)
        elif cmd == "RIGHT":     dx, dy = world_delta(0, NUDGE_STEP_M)
        elif cmd == "LEFT":      dx, dy = world_delta(0, -NUDGE_STEP_M)
        elif cmd == "UP":        dz = NUDGE_STEP_M
        elif cmd == "DOWN":      dz = -NUDGE_STEP_M
        elif cmd == "YAW_RIGHT": dyaw = -YAW_STEP_RAD
        elif cmd == "YAW_LEFT":  dyaw = YAW_STEP_RAD
        else:
            self.get_logger().warn(f"Unknown manual nudge: {cmd}")
            return
        self._sp_x += dx
        self._sp_y += dy
        self._sp_z = max(0.3, self._sp_z + dz)  # never nudge below 0.3m
        self._sp_z_target = self._sp_z
        self._sp_yaw += dyaw

    def _set_offboard(self):
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"OFFBOARD result: {f.result().mode_sent}"))

    def _arm_and_offboard(self):
        if not self.vehicle_state.connected:
            self.get_logger().error("Cannot takeoff — MAVROS not connected")
            return
        # Aim the setpoint stream BEFORE requesting OFFBOARD/arm: PX4
        # latches onto whatever is streaming the instant the mode switch
        # lands, so setting the target only afterwards leaves a window where
        # a stale setpoint (a previous mission's last stop) gets chased.
        # Ramp from the current altitude — _setpoint_loop walks _sp_z toward
        # the target at CLIMB_RATE instead of stepping the whole way at once.
        # Hold the CURRENT horizontal position while climbing: the default
        # _sp_x/_sp_y of (0,0) is home, so a takeoff from a delivery stop
        # used to be yanked sideways toward home until the navigator took
        # over (seen live 2026-07-12 as a 3.3 m/s horizontal spike right
        # after inter-stop liftoff).
        if self.current_pos:
            self._sp_x, self._sp_y = self.current_pos[0], self.current_pos[1]
        self._sp_z = self.current_pos[2] if self.current_pos else 0.0
        # Relative to the ground reading at arm time: EKF z drifts several
        # metres per landing cycle in SITL, so an absolute target can sit at
        # or below the believed altitude while physically on the ground --
        # the ramp then finishes instantly and the drone never lifts off
        # (2nd consecutive mission never took off, seen live 2026-07-14).
        self._sp_z_target = self._sp_z + self.takeoff_altitude
        self._set_offboard()
        self._arm()
        self.get_logger().info(
            f"Takeoff → {self.takeoff_altitude}m (ramped at {CLIMB_RATE} m/s)")

    def _land(self):
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)
        self._update_status(self.STATUS_LANDING)
        self.get_logger().info("AUTO.LAND requested")

    # ─────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────

    def set_setpoint(self, x, y, z):
        self._sp_x, self._sp_y, self._sp_z = x, y, z
        self._sp_z_target = z

    def get_position(self):   return self.current_pos
    def get_home_gps(self):
        if self.home_lat is None: return None
        return (self.home_lat, self.home_lon, self.home_alt)
    def is_armed(self):       return self.vehicle_state.armed
    def is_airborne(self):    return (self.current_pos is not None
                                     and self.current_pos[2] > 0.3
                                     and self.vehicle_state.armed)
    def is_connected(self):   return self.vehicle_state.connected
    def get_altitude(self):   return self.current_pos[2] if self.current_pos else 0.0
    def reached_altitude(self, target, thresh=0.3):
        return abs(self.get_altitude() - target) < thresh

    # ─────────────────────────────────────────────────────────
    # Timers
    # ─────────────────────────────────────────────────────────

    def _setpoint_loop(self):
        """Stream setpoints at 20Hz — PX4 requires this for OFFBOARD mode.

        Suppressed while waypoint_navigator or aruco_landing_node is active —
        whichever of them is driving the mission owns this topic during that
        phase, and a second publisher racing on it causes setpoint jitter.
        """
        if self._nav_active or self._landing_active:
            return
        if self._sp_z < self._sp_z_target:
            self._sp_z = min(self._sp_z + CLIMB_RATE / SETPOINT_HZ, self._sp_z_target)
        elif self._sp_z > self._sp_z_target:
            self._sp_z = max(self._sp_z - CLIMB_RATE / SETPOINT_HZ, self._sp_z_target)
        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = self._sp_x
        sp.pose.position.y = self._sp_y
        sp.pose.position.z = self._sp_z
        sp.pose.orientation.z = math.sin(self._sp_yaw / 2.0)
        sp.pose.orientation.w = math.cos(self._sp_yaw / 2.0)
        self.setpoint_pub.publish(sp)

    def _status_loop(self):
        """Publish status every 1s for MissionManager."""
        msg = String()
        msg.data = self.drone_status
        self.status_pub.publish(msg)
        self.get_logger().info(
            f"[STATUS] {self.drone_status} | "
            f"connected={self.vehicle_state.connected} | "
            f"mode={self.vehicle_state.mode} | "
            f"alt={self.get_altitude():.1f}m",
            throttle_duration_sec=2.0)

    def _update_status(self, new: str):
        if self.drone_status != new:
            self.get_logger().info(
                f"[STATUS] {self.drone_status} → {new}")
            self.drone_status = new


def main(args=None):
    rclpy.init(args=args)
    node = DroneBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()