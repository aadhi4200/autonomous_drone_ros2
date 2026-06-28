#!/usr/bin/env python3
"""
drone_base_node.py | Package: drone_base
Responsibilities: MAVROS connection, Arm/Disarm, Takeoff, Land
Foundation node — all other nodes depend on this.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

TAKEOFF_ALT = 5.0
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

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)

        self.create_subscription(State,        "/mavros/state",               self._state_cb, sensor_qos)
        self.create_subscription(Odometry,     "/mavros/local_position/odom", self._odom_cb,  sensor_qos)
        self.create_subscription(HomePosition, "/mavros/home_position/home",  self._home_cb,  sensor_qos)
        self.create_subscription(String,       "/drone_base/command",         self._cmd_cb,   10)

        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.status_pub   = self.create_publisher(String,      "/drone_base/status", 10)

        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        self.vehicle_state = State()
        self.current_pos   = None
        self.home_lat = self.home_lon = self.home_alt = None
        self.drone_status  = self.STATUS_DISCONNECTED
        self._sp_x = self._sp_y = 0.0
        self._sp_z = self.takeoff_altitude

        self.create_timer(1.0 / SETPOINT_HZ, self._setpoint_loop)
        self.create_timer(1.0,               self._status_loop)
        self.get_logger().info("DroneBaseNode started.")

    def _state_cb(self, msg):
        self.vehicle_state = msg
        if not msg.connected:
            self._update_status(self.STATUS_DISCONNECTED)
        elif msg.armed and self.drone_status not in [self.STATUS_AIRBORNE, self.STATUS_LANDING]:
            self._update_status(self.STATUS_ARMED)
        elif not msg.armed and self.drone_status not in [self.STATUS_DISCONNECTED]:
            self._update_status(self.STATUS_LANDED)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)
        if self.vehicle_state.armed and p.z > 0.3 and self.drone_status == self.STATUS_ARMED:
            self._update_status(self.STATUS_AIRBORNE)

    def _home_cb(self, msg):
        if self.home_lat is None:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.home_alt = msg.geo.altitude
            self.get_logger().info(f"Home GPS: ({self.home_lat:.6f}, {self.home_lon:.6f})")

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[CMD] {cmd}")
        if   cmd == "TAKEOFF": self._arm_and_offboard()
        elif cmd == "LAND":    self._land()
        elif cmd == "ARM":     self._arm()
        elif cmd == "DISARM":  self._disarm()

    def _arm(self):
        req = CommandBool.Request(); req.value = True
        self.arming_client.call_async(req)

    def _disarm(self):
        req = CommandBool.Request(); req.value = False
        self.arming_client.call_async(req)

    def _set_offboard(self):
        req = SetMode.Request(); req.custom_mode = "OFFBOARD"
        self.set_mode_client.call_async(req)

    def _arm_and_offboard(self):
        self._set_offboard()
        self._arm()
        self._sp_z = self.takeoff_altitude
        self.get_logger().info(f"Takeoff → {self.takeoff_altitude}m")

    def _land(self):
        req = SetMode.Request(); req.custom_mode = "AUTO.LAND"
        self.set_mode_client.call_async(req)
        self._update_status(self.STATUS_LANDING)

    def set_setpoint(self, x, y, z):
        self._sp_x, self._sp_y, self._sp_z = x, y, z

    def get_position(self):   return self.current_pos
    def get_home_gps(self):   return (self.home_lat, self.home_lon, self.home_alt) if self.home_lat else None
    def is_armed(self):       return self.vehicle_state.armed
    def is_airborne(self):    return self.current_pos is not None and self.current_pos[2] > 0.3 and self.vehicle_state.armed
    def is_connected(self):   return self.vehicle_state.connected
    def get_altitude(self):   return self.current_pos[2] if self.current_pos else 0.0
    def reached_altitude(self, target, thresh=0.3): return abs(self.get_altitude() - target) < thresh

    def _setpoint_loop(self):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = self._sp_x
        sp.pose.position.y = self._sp_y
        sp.pose.position.z = self._sp_z
        sp.pose.orientation.w = 1.0
        self.setpoint_pub.publish(sp)

    def _status_loop(self):
        msg = String(); msg.data = self.drone_status
        self.status_pub.publish(msg)
        self.get_logger().info(
            f"[STATUS] {self.drone_status} | mode={self.vehicle_state.mode} | alt={self.get_altitude():.1f}m",
            throttle_duration_sec=2.0)

    def _update_status(self, new):
        if self.drone_status != new:
            self.get_logger().info(f"[STATUS] {self.drone_status} → {new}")
            self.drone_status = new

def main(args=None):
    rclpy.init(args=args)
    node = DroneBaseNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
