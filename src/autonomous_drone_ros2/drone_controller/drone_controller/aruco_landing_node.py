#!/usr/bin/env python3
"""
aruco_landing_node.py | Package: drone_controller
SEARCH → ALIGN → DESCEND → LAND state machine for ArUco precision landing.
"""
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

SEARCH_YAW_RATE  = 0.3
ALIGN_THRESHOLD  = 20.0
ALIGN_KP         = 0.003
DESCEND_ALTITUDE = 1.5
DESCEND_RATE     = 0.15
LAND_ALTITUDE    = 0.3
LOSS_TIMEOUT     = 2.0
SEARCH_TIMEOUT   = 30.0
MAX_ALIGN_VEL    = 0.5

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

        self.vel_pub      = self.create_publisher(TwistStamped, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10)
        self.setpoint_pub = self.create_publisher(PoseStamped,  "/mavros/setpoint_position/local", 10)
        self.status_pub   = self.create_publisher(String,       "/aruco_landing/status", 10)

        self.current_pos      = None
        self.current_yaw      = 0.0
        self.landing_state    = self.STATE_IDLE
        self.marker_detected  = False
        self.marker_offset_x  = 0.0
        self.marker_offset_y  = 0.0
        self.state_start      = None
        self.last_detection   = None
        self.search_yaw       = 0.0

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
        if cmd == "START": self._enter(self.STATE_SEARCH)
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

    def _align(self, alt):
        if not self.marker_detected: return
        ex,ey = self.marker_offset_x, self.marker_offset_y
        vx = self._clamp( ALIGN_KP*ex, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
        vy = self._clamp(-ALIGN_KP*ey, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
        self._pub_vel(vx, vy, 0.0)
        err = math.sqrt(ex*ex+ey*ey)
        self.get_logger().info(f"[ALIGN] err={err:.1f}px", throttle_duration_sec=1.0)
        if err < ALIGN_THRESHOLD:
            self._enter(self.STATE_DESCEND if alt > DESCEND_ALTITUDE else self.STATE_LAND)

    def _descend(self, alt):
        if not self.marker_detected: self._pub_vel(0,0,0); return
        ex,ey = self.marker_offset_x, self.marker_offset_y
        vx = self._clamp( ALIGN_KP*ex, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
        vy = self._clamp(-ALIGN_KP*ey, -MAX_ALIGN_VEL, MAX_ALIGN_VEL)
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
            self.get_logger().warn("Marker lost → SEARCH")
            self.marker_detected = False
            self._enter(self.STATE_SEARCH)

    def _pub_vel(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x=vx; msg.twist.linear.y=vy; msg.twist.linear.z=vz
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
