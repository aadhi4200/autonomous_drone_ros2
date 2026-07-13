#!/usr/bin/env python3
"""
aruco_landing_node.py | Package: drone_controller
SEARCH → ALIGN → DESCEND → LAND state machine for ArUco precision landing.
"""
from geometry_msgs import msg
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from drone_interfaces.constants import ARUCO_MARKER_ID, TOPIC_VISION_TARGET_ID

SEARCH_YAW_RATE  = 0.3
ALIGN_THRESHOLD  = 20.0
ALIGN_KP         = 0.003
DESCEND_ALTITUDE = 1.5
DESCEND_RATE     = 0.15
LAND_ALTITUDE    = 0.3
LOSS_TIMEOUT     = 2.0
SEARCH_TIMEOUT   = 30.0
MAX_ALIGN_VEL    = 0.5
BLIND_LAND_ALTITUDE = 2.5  # below this, a marker lost mid-DESCEND finishes
                           # the landing blind instead of re-searching

class ArucoLandingNode(Node):
    STATE_IDLE     = "IDLE"
    STATE_SEARCH   = "SEARCH"
    STATE_ALIGN    = "ALIGN"
    STATE_DESCEND  = "DESCEND"
    STATE_LAND     = "LAND"
    STATE_COMPLETE = "COMPLETE"
    STATE_FAILED   = "FAILED"

    def __init__(self):
        super().__init__("aruco_landing_node")
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_subscription(PointStamped, "/vision/landing_target",      self._target_cb, sensor_qos)
        self.create_subscription(Odometry,     "/mavros/local_position/odom", self._odom_cb,   sensor_qos)
        self.create_subscription(String,       "/aruco_landing/command",      self._cmd_cb,    10)

        # cmd_vel takes TwistStamped; cmd_vel_unstamped takes plain Twist.
        # Publishing TwistStamped on the _unstamped topic was a silent type
        # mismatch (the project's classic failure mode): PX4 received ZERO
        # setpoints during ALIGN, lost the offboard stream, and fired its
        # RTL failsafe ~2s after every marker lock (seen live 2026-07-13).
        self.vel_pub      = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10)
        self.setpoint_pub = self.create_publisher(PoseStamped,  "/mavros/setpoint_position/local", 10)
        self.status_pub   = self.create_publisher(String,       "/aruco_landing/status", 10)
        self.target_id_pub = self.create_publisher(Int32,       TOPIC_VISION_TARGET_ID, 10)

        self.current_pos      = None
        self.current_yaw      = 0.0
        self.landing_state    = self.STATE_IDLE
        self.marker_detected  = False
        self.marker_offset_x  = 0.0
        self.marker_offset_y  = 0.0
        self.state_start      = None
        self.last_detection   = None
        self.search_yaw       = 0.0
        self.expected_marker_id = ARUCO_MARKER_ID

        self.create_timer(1.0/20.0, self._loop)
        self.create_timer(0.5,      self._pub_status)
        self.get_logger().info("ArucoLandingNode ready.")

    def _target_cb(self, msg):
        self.marker_detected = True
        self.marker_offset_x = msg.point.x
        self.marker_offset_y = msg.point.y
        self.last_detection  = self.get_clock().now()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "START" or cmd.startswith("START:"):
            parts = cmd.split(":")
            self.expected_marker_id = int(parts[1]) if len(parts) > 1 else ARUCO_MARKER_ID
            self.target_id_pub.publish(Int32(data=self.expected_marker_id))
            self.get_logger().info(f"Expecting marker ID={self.expected_marker_id} at this stop")
            self._enter(self.STATE_SEARCH)
        elif cmd == "ABORT": self._enter(self.STATE_IDLE); self._stop_vel()

    def _loop(self):
        if self.landing_state == self.STATE_IDLE or self.current_pos is None: return
        self._check_loss()
        alt = self.current_pos[2]
        if   self.landing_state == self.STATE_SEARCH:  self._search()
        elif self.landing_state == self.STATE_ALIGN:   self._align(alt)
        elif self.landing_state == self.STATE_DESCEND: self._descend(alt)
        elif self.landing_state == self.STATE_LAND:    self._land()

    def _search(self):
        if self._elapsed() > SEARCH_TIMEOUT:
            self._enter(self.STATE_FAILED); return
        if self.marker_detected:
            self.get_logger().info("✅ Marker found → ALIGN")
            self._enter(self.STATE_ALIGN); return
        self.search_yaw += SEARCH_YAW_RATE*(1.0/20.0)
        cx,cy,cz = self.current_pos
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x=cx; sp.pose.position.y=cy; sp.pose.position.z=cz
        sp.pose.orientation.z = math.sin(self.search_yaw/2.0)
        sp.pose.orientation.w = math.cos(self.search_yaw/2.0)
        self.setpoint_pub.publish(sp)

    def _error_to_world_vel(self, ex, ey):
        """Pixel error -> ENU world velocity, corrected for current yaw.

        Down-camera mounted rpy (0, 1.5707, 0): image-right = body -y,
        image-down = body -x. The old mapping ignored yaw entirely, which is
        only correct at one heading -- and after the search spin the heading
        is arbitrary."""
        v_bx = self._clamp(-ALIGN_KP*ey, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
        v_by = self._clamp(-ALIGN_KP*ex, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
        c, sn = math.cos(self.current_yaw), math.sin(self.current_yaw)
        return (v_bx*c - v_by*sn, v_bx*sn + v_by*c)

    def _align(self, alt):
        if not self.marker_detected:
            # Keep the offboard stream alive -- a gap >1s trips PX4's
            # offboard-loss failsafe mid-landing.
            self._pub_vel(0, 0, 0)
            return
        ex,ey = self.marker_offset_x, self.marker_offset_y
        vx, vy = self._error_to_world_vel(ex, ey)
        self._pub_vel(vx, vy, 0.0)
        err = math.sqrt(ex*ex+ey*ey)
        self.get_logger().info(f"[ALIGN] err={err:.1f}px", throttle_duration_sec=1.0)
        if err < ALIGN_THRESHOLD:
            self._enter(self.STATE_DESCEND if alt > DESCEND_ALTITUDE else self.STATE_LAND)

    def _descend(self, alt):
        if not self.marker_detected: self._pub_vel(0,0,0); return
        ex,ey = self.marker_offset_x, self.marker_offset_y
        vx, vy = self._error_to_world_vel(ex, ey)
        self._pub_vel(vx, vy, -DESCEND_RATE)
        self.get_logger().info(f"[DESCEND] alt={alt:.2f}m", throttle_duration_sec=1.0)
        if alt < LAND_ALTITUDE: self._enter(self.STATE_LAND)

    def _land(self):
        self._stop_vel()
        self._enter(self.STATE_COMPLETE)

    def _check_loss(self):
        if self.landing_state not in [self.STATE_ALIGN, self.STATE_DESCEND]: return
        if self.last_detection is None: return
        if (self.get_clock().now()-self.last_detection).nanoseconds*1e-9 > LOSS_TIMEOUT:
            self.marker_detected = False
            # Blind final descent: losing the marker LOW during DESCEND is
            # normal (the marker fills/leaves the tilting camera view near
            # the ground) and we were already centered when DESCEND began --
            # finish the landing instead of wastefully re-searching from a
            # position that is already on target (seen live 2026-07-13:
            # centered at 11px error, then a re-search at 2.1m burned 30s
            # and gave up).
            alt = self.current_pos[2] if self.current_pos else 99.0
            if self.landing_state == self.STATE_DESCEND and alt < BLIND_LAND_ALTITUDE:
                self.get_logger().info(
                    f"Marker lost at {alt:.1f}m during DESCEND — already centered, landing blind")
                self._enter(self.STATE_LAND)
                return
            self.get_logger().warn("Marker lost → SEARCH")
            self._enter(self.STATE_SEARCH)

    def _pub_vel(self, vx, vy, vz):
        # msg = TwistStamped()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "map"
        # msg.twist.linear.x=vx; msg.twist.linear.y=vy; msg.twist.linear.z=vz
        # self.vel_pub.publish(msg)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        self.vel_pub.publish(msg)

    def _stop_vel(self): self._pub_vel(0,0,0)

    def _pub_status(self):
        msg = String(); msg.data = self.landing_state
        self.status_pub.publish(msg)

    def _enter(self, new):
        self.get_logger().info(f"[ARUCO] {self.landing_state} → {new}")
        self.landing_state = new
        self.state_start   = self.get_clock().now()
        self.marker_detected = False

    def _elapsed(self):
        if self.state_start is None: return 0.0
        return (self.get_clock().now()-self.state_start).nanoseconds*1e-9

    def _clamp(self, v, lo, hi): return max(lo, min(hi, v))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
