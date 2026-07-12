#!/usr/bin/env python3
"""
waypoint_navigator.py | Package: drone_controller
GPS waypoint navigation with Haversine conversion.
Commands: GOTO:A / GOTO:B / GOTO:C / GOTO:D / STOP

"A" (home) is never stored as a static coordinate — it is always resolved to
wherever /mavros/home_position/home currently says home is. That's what makes
the SITL-home-tracks-the-operator's-actual-location fix (see 3.3 in the task
spec) safe: there is no stale "A" entry that could point at yesterday's city.
"""
import json
import os

import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mavros_msgs.msg import HomePosition

from drone_interfaces.geo import gps_to_local
from drone_interfaces.constants import TOPIC_MISSION_WAYPOINTS, RTH_ALTITUDE

WP_TOLERANCE = 0.4
WP_TIMEOUT   = 120.0  # accel-limited flight is slower than the old
                      # step-setpoint flight; 60s was already borderline
                      # for a 160m leg at 3 m/s
# Gradual motion profile (real-hardware requirement): the setpoint "carrot"
# accelerates/decelerates smoothly instead of handing PX4 the full target as
# one step input (a 160m step made PX4 pitch hard — big IMU swings).
NAV_ACCEL        = 1.0   # m/s^2
NAV_CRUISE_SPEED = 3.0   # m/s, matches the MPC_XY_VEL_MAX default the UI sets
NAV_MIN_SPEED    = 0.3   # m/s floor so the carrot always finishes
CRUISE_ALT   = 5.0
SETPOINT_HZ  = 20.0

# Standalone-testing default stops, relative to wherever home turns out to be
# (metres north/east of home) — mirrors the static Gazebo world's original
# 10m/20m/30m-north pad layout. Only used until the frontend uploads real
# waypoints; never a hardcoded absolute GPS literal.
DEFAULT_RELATIVE_STOPS = {
    "B": (10.0, 0.0),
    "C": (20.0, 0.0),
    "D": (30.0, 0.0),
}

LAST_SYNCED_HOME_FILE = os.path.expanduser("~/drone_ws2/.last_synced_home")


class Waypoint:
    def __init__(self, lat, lon, alt=CRUISE_ALT, label="WP", marker_id=None):
        self.lat = lat; self.lon = lon; self.alt = alt
        self.label = label; self.marker_id = marker_id


def _read_fallback_home():
    """Best-effort default for the home_lat/home_lon ROS params, read once at
    startup from whatever /system/set-home last persisted. This is only ever
    used before the real /mavros/home_position/home arrives."""
    try:
        with open(LAST_SYNCED_HOME_FILE) as f:
            lat_str, lon_str = f.read().strip().split(",")
            return float(lat_str), float(lon_str)
    except Exception:
        return 0.0, 0.0


class WaypointNavigator(Node):
    NAV_IDLE       = "IDLE"
    NAV_NAVIGATING = "NAVIGATING"
    NAV_ARRIVED    = "ARRIVED"
    NAV_TIMEOUT    = "TIMEOUT"

    def __init__(self):
        super().__init__("waypoint_navigator")

        fallback_lat, fallback_lon = _read_fallback_home()
        self.declare_parameter("home_lat", fallback_lat)
        self.declare_parameter("home_lon", fallback_lon)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)
        self.create_subscription(Odometry,     "/mavros/local_position/odom", self._odom_cb, sensor_qos)
        self.create_subscription(HomePosition, "/mavros/home_position/home",  self._home_cb, sensor_qos)
        self.create_subscription(String,       "/waypoint_nav/command",       self._cmd_cb,  10)
        self.create_subscription(String,       TOPIC_MISSION_WAYPOINTS,       self._waypoints_cb, 10)

        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.status_pub   = self.create_publisher(String,      "/waypoint_nav/status", 10)

        # B/C/D (and beyond) only — "A"/home is resolved live, never stored here.
        self.waypoints = {}
        self.uploaded  = False

        param_lat = self.get_parameter("home_lat").value
        param_lon = self.get_parameter("home_lon").value
        self.home_lat, self.home_lon = param_lat, param_lon
        self.home_is_real = False
        if (param_lat, param_lon) == (0.0, 0.0):
            self.get_logger().warn(
                "No home ever synced (no /system/set-home + no persisted fallback) — "
                "using (0.0, 0.0) as a placeholder home until /mavros/home_position/home arrives.")

        self.current_pos     = None
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
        if not self.home_is_real:
            self.home_lat = msg.geo.latitude
            self.home_lon = msg.geo.longitude
            self.home_is_real = True
            self.get_logger().info(f"Home GPS (real, from MAVROS): ({self.home_lat:.6f},{self.home_lon:.6f})")
            self._seed_default_stops_if_needed()

    def _seed_default_stops_if_needed(self):
        """Fill in B/C/D relative to home for standalone testing, only if the
        frontend hasn't uploaded a real mission yet."""
        if self.uploaded or self.waypoints:
            return
        for label, (north_m, east_m) in DEFAULT_RELATIVE_STOPS.items():
            dlat = math.degrees(north_m / 6371000)
            dlon = math.degrees(east_m / (6371000 * math.cos(math.radians(self.home_lat))))
            self.waypoints[label] = Waypoint(
                self.home_lat + dlat, self.home_lon + dlon, label=f"DEFAULT_{label}")
        self.get_logger().info("Seeded default B/C/D stops relative to home (no mission uploaded yet).")

    def _waypoints_cb(self, msg: String):
        """Receive GPS waypoints uploaded from the website (frontend -> backend
        -> /mission/waypoints), replacing any default/previous stops."""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Bad /mission/waypoints payload: {e}")
            return

        self.waypoints = {}
        for wp in data:
            key = wp.get("label", "B")
            self.waypoints[key] = Waypoint(
                lat=wp["lat"], lon=wp["lon"], alt=wp.get("alt", CRUISE_ALT),
                label=wp.get("label", key), marker_id=wp.get("marker_id"))
            self.get_logger().info(
                f"Waypoint added: {key} → ({wp['lat']:.6f}, {wp['lon']:.6f}) "
                f"alt={wp.get('alt', CRUISE_ALT)} marker_id={wp.get('marker_id')}")
        self.uploaded = True

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd.startswith("GOTO:"): self._start_nav(cmd.split(":")[1])
        elif cmd == "STOP": self.nav_state = self.NAV_IDLE

    def _start_nav(self, key):
        if not self.home_is_real:
            self.get_logger().warn("Home GPS not yet confirmed by MAVROS — using unsynced fallback home!")
        if key == "A":
            self.active_wp = Waypoint(self.home_lat, self.home_lon, alt=RTH_ALTITUDE, label="HOME")
            self.active_wp_local = (0.0, 0.0, RTH_ALTITUDE)
        else:
            if key not in self.waypoints:
                self.get_logger().error(f"Unknown WP: {key}"); return
            self.active_wp = self.waypoints[key]
            north, east = gps_to_local(self.home_lat, self.home_lon,
                                        self.active_wp.lat, self.active_wp.lon)
            self.active_wp_local = (east, north, self.active_wp.alt)
        self.state_start = self.get_clock().now()
        # Carrot starts from wherever the drone actually is, so the very
        # first setpoint is right next to it — no step input.
        sx, sy, sz = self.current_pos if self.current_pos else self.active_wp_local
        tx, ty, tz = self.active_wp_local
        dx, dy, dz = tx - sx, ty - sy, tz - sz
        self.carrot_len = math.sqrt(dx*dx + dy*dy + dz*dz)
        self.carrot_dir = (dx / self.carrot_len, dy / self.carrot_len, dz / self.carrot_len) \
            if self.carrot_len > 1e-6 else (0.0, 0.0, 0.0)
        self.carrot_origin = (sx, sy, sz)
        self.carrot_s = 0.0
        self.carrot_v = 0.0
        self.nav_state = self.NAV_NAVIGATING
        self.get_logger().info(f"Navigating → {self.active_wp.label}")

    def _nav_loop(self):
        if self.nav_state == self.NAV_IDLE or self.current_pos is None: return
        wp_x, wp_y, wp_z = self.active_wp_local
        elapsed = (self.get_clock().now()-self.state_start).nanoseconds*1e-9
        if self.nav_state == self.NAV_NAVIGATING:
            if elapsed > WP_TIMEOUT:
                self.nav_state = self.NAV_TIMEOUT; self._pub_status(); return
            # Trapezoidal speed profile: ramp up at NAV_ACCEL, cruise, and
            # ramp down so v^2 <= 2*a*remaining — PX4 receives a smoothly
            # moving target instead of a distant fixed one.
            dt = 1.0 / SETPOINT_HZ
            remaining = max(self.carrot_len - self.carrot_s, 0.0)
            self.carrot_v = min(self.carrot_v + NAV_ACCEL * dt,
                                NAV_CRUISE_SPEED,
                                max(math.sqrt(2.0 * NAV_ACCEL * remaining), NAV_MIN_SPEED))
            self.carrot_s = min(self.carrot_s + self.carrot_v * dt, self.carrot_len)
            ox, oy, oz = self.carrot_origin
            cdx, cdy, cdz = self.carrot_dir
            self._pub_setpoint(ox + cdx * self.carrot_s,
                               oy + cdy * self.carrot_s,
                               oz + cdz * self.carrot_s)
            dist = self._dist(wp_x, wp_y, wp_z)
            self.get_logger().info(f"[NAV] → {self.active_wp.label} dist={dist:.2f}m",
                                   throttle_duration_sec=2.0)
            if dist < WP_TOLERANCE:
                self.get_logger().info(f"✅ Arrived at {self.active_wp.label}")
                self.nav_state = self.NAV_ARRIVED; self._pub_status()
        elif self.nav_state == self.NAV_ARRIVED:
            self._pub_setpoint(wp_x, wp_y, wp_z)

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
