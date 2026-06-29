#!/usr/bin/env python3
"""
waypoint_navigator.py | Package: drone_controller
GPS waypoint navigation with Haversine conversion.
Commands: GOTO:A / GOTO:B / GOTO:C / GOTO:D / STOP
"""
import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import HomePosition

WP_TOLERANCE = 0.4
WP_TIMEOUT   = 60.0
CRUISE_ALT   = 5.0
SETPOINT_HZ  = 20.0

class Waypoint:
    def __init__(self, lat, lon, alt=CRUISE_ALT, label="WP"):
        self.lat=lat; self.lon=lon; self.alt=alt; self.label=label

class WaypointNavigator(Node):
    NAV_IDLE       = "IDLE"
    NAV_NAVIGATING = "NAVIGATING"
    NAV_ARRIVED    = "ARRIVED"
    NAV_TIMEOUT    = "TIMEOUT"

    def __init__(self):
        super().__init__("waypoint_navigator")
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)
        self.create_subscription(Odometry,     "/mavros/local_position/odom", self._odom_cb, sensor_qos)
        self.create_subscription(HomePosition, "/mavros/home_position/home",  self._home_cb, sensor_qos)
        self.create_subscription(String,       "/waypoint_nav/command",       self._cmd_cb,  10)

        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.status_pub   = self.create_publisher(String,      "/waypoint_nav/status", 10)

        # ── Edit GPS waypoints here ──────────────────────
        self.waypoints = {
            "A": Waypoint(10.850500, 76.271000, label="HOME"),
            "B": Waypoint(10.850600, 76.271200, label="POINT_B"),
            "C": Waypoint(10.850700, 76.271400, label="POINT_C"),
            "D": Waypoint(10.850800, 76.271600, label="POINT_D"),
        }

        # ====================================================================================================
        
        # self.create_subscription(
        #     String, "/mission/waypoints",
        #     self._waypoints_cb, 10)

        # # Add this callback
        # def _waypoints_cb(self, msg: String):
        #     """Receive GPS waypoints from frontend via MissionManager."""
        #     import json
        #     data = json.loads(msg.data)

        #     # Always add home as A
        #     if self.home_lat is not None:
        #         self.waypoints["A"] = Waypoint(
        #             self.home_lat, self.home_lon, label="HOME")

        #     # Add frontend waypoints as B, C, D...
        #     for wp in data:
        #         key = wp.get("label", "B")
        #         self.waypoints[key] = Waypoint(
        #             lat   = wp["lat"],
        #             lon   = wp["lon"],
        #             alt   = wp.get("alt", 5.0),
        #             label = wp.get("label", key)
        #         )
        #         self.get_logger().info(
        #             f"Waypoint added: {key} → "
        #             f"({wp['lat']:.6f}, {wp['lon']:.6f})")


        #========================================================================================================



        self.current_pos     = None
        self.home_lat = self.home_lon = None
        self.nav_state       = self.NAV_IDLE
        self.active_wp       = None
        self.active_wp_local = None
        self.state_start     = None

        self.create_timer(1.0/SETPOINT_HZ, self._nav_loop)
        self.create_timer(0.5,             self._pub_status)
        self.get_logger().info("WaypointNavigator ready.")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.current_pos = (p.x, p.y, p.z)

    def _home_cb(self, msg):
        if self.home_lat is None:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.get_logger().info(f"Home GPS: ({self.home_lat:.6f},{self.home_lon:.6f})")

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd.startswith("GOTO:"): self._start_nav(cmd.split(":")[1])
        elif cmd == "STOP": self.nav_state = self.NAV_IDLE

    def _start_nav(self, key):
        if self.home_lat is None:
            self.get_logger().error("Home GPS not locked!"); return
        if key not in self.waypoints:
            self.get_logger().error(f"Unknown WP: {key}"); return
        self.active_wp = self.waypoints[key]
        north, east = self._gps_to_local(self.home_lat, self.home_lon,
                                          self.active_wp.lat, self.active_wp.lon)
        self.active_wp_local = (east, north, self.active_wp.alt)
        self.state_start = self.get_clock().now()
        self.nav_state = self.NAV_NAVIGATING
        self.get_logger().info(f"Navigating → {self.active_wp.label}")

    def _nav_loop(self):
        if self.nav_state == self.NAV_IDLE or self.current_pos is None: return
        wp_x, wp_y, wp_z = self.active_wp_local
        elapsed = (self.get_clock().now()-self.state_start).nanoseconds*1e-9
        if self.nav_state == self.NAV_NAVIGATING:
            if elapsed > WP_TIMEOUT:
                self.nav_state = self.NAV_TIMEOUT; self._pub_status(); return
            self._pub_setpoint(wp_x, wp_y, wp_z)
            dist = self._dist(wp_x, wp_y, wp_z)
            self.get_logger().info(f"[NAV] → {self.active_wp.label} dist={dist:.2f}m",
                                   throttle_duration_sec=2.0)
            if dist < WP_TOLERANCE:
                self.get_logger().info(f"✅ Arrived at {self.active_wp.label}")
                self.nav_state = self.NAV_ARRIVED; self._pub_status()
        elif self.nav_state == self.NAV_ARRIVED:
            self._pub_setpoint(wp_x, wp_y, wp_z)

    def _gps_to_local(self, hlat, hlon, tlat, tlon):
        R = 6371000
        north = math.radians(tlat-hlat)*R
        east  = math.radians(tlon-hlon)*R*math.cos(math.radians(hlat))
        return north, east

    def _dist(self, tx, ty, tz):
        if self.current_pos is None: return float("inf")
        dx,dy,dz = self.current_pos[0]-tx, self.current_pos[1]-ty, self.current_pos[2]-tz
        return math.sqrt(dx*dx+dy*dy+dz*dz)

    def _pub_setpoint(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x=x; sp.pose.position.y=y; sp.pose.position.z=z
        sp.pose.orientation.w = 1.0
        self.setpoint_pub.publish(sp)

    def _pub_status(self):
        msg = String(); msg.data = self.nav_state
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
