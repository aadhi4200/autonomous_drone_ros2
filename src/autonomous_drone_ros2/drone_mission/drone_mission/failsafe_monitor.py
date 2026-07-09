#!/usr/bin/env python3
"""
failsafe_monitor.py | Package: drone_mission
Watches the onboard links (MAVROS<->PX4, node<->node) independently of the
website/backend, and triggers a return-to-home when they drop mid-mission.
Kept as its own node (rather than folded into mission_manager) so this
safety-critical logic is easy to test/reason about in isolation.

Deliberately does NOT watch the website/backend connection at all — losing
the browser tab is not a flight emergency (the dashboard is monitoring-only).
Only onboard-link loss triggers RTH here.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State

from drone_interfaces.constants import NODE_HEARTBEAT_STALE_S, TOPIC_MISSION_SAFETY_EVENT

MONITORED_STATUS_TOPICS = [
    "/drone_base/status",
    "/waypoint_nav/status",
    "/aruco_landing/status",
    "/vision_node/heartbeat",
    "/camera_node/heartbeat",
    "/mission/status",
]

LOW_BATTERY_PCT = 15.0
AIRBORNE_STATES = {"PREFLIGHT", "TAKEOFF", "GOTO_WAYPOINT", "ARUCO_LAND",
                    "WAIT_ON_GROUND", "INTER_TAKEOFF", "RETURN_HOME", "HOME_LAND"}


class FailsafeMonitor(Node):
    def __init__(self):
        super().__init__("failsafe_monitor")

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)

        self.mavros_connected = False
        self.mission_state = "IDLE"
        self.last_seen = {topic: None for topic in MONITORED_STATUS_TOPICS}
        self.battery_pct = 100.0
        self.rth_already_triggered = False
        self.battery_land_already_triggered = False

        self.create_subscription(State, "/mavros/state", self._state_cb, sensor_qos)
        self.create_subscription(BatteryState, "/mavros/battery", self._battery_cb, sensor_qos)
        for topic in MONITORED_STATUS_TOPICS:
            self.create_subscription(String, topic, self._make_heartbeat_cb(topic), 10)

        self.cmd_pub = self.create_publisher(String, "/mission/command", 10)
        self.safety_event_pub = self.create_publisher(String, TOPIC_MISSION_SAFETY_EVENT, 10)

        self.create_timer(0.5, self._check)
        self.get_logger().info("FailsafeMonitor ready — watching onboard MAVROS/node links.")

    def _state_cb(self, msg):
        self.mavros_connected = msg.connected

    def _battery_cb(self, msg):
        if msg.percentage is not None and msg.percentage >= 0:
            self.battery_pct = msg.percentage * 100.0

    def _make_heartbeat_cb(self, topic):
        def cb(msg):
            self.last_seen[topic] = time.monotonic()
            if topic == "/mission/status":
                self.mission_state = msg.data
        return cb

    def _airborne(self):
        return self.mission_state in AIRBORNE_STATES

    def _stale_nodes(self):
        now = time.monotonic()
        stale = []
        for topic, seen in self.last_seen.items():
            if seen is None or (now - seen) > NODE_HEARTBEAT_STALE_S:
                stale.append(topic)
        return stale

    def _check(self):
        if not self._airborne():
            self.rth_already_triggered = False
            self.battery_land_already_triggered = False
            return

        # Battery emergency: land now, don't try to fly home on a dying pack.
        if self.battery_pct < LOW_BATTERY_PCT and not self.battery_land_already_triggered:
            self.battery_land_already_triggered = True
            self._log_safety_event("LOW_BATTERY_LAND", f"battery={self.battery_pct:.1f}%")
            self.cmd_pub.publish(String(data="ABORT"))
            return

        if self.rth_already_triggered:
            return

        if not self.mavros_connected:
            self.rth_already_triggered = True
            self._log_safety_event("MAVROS_LOST", "MAVROS reported disconnected mid-mission")
            self.cmd_pub.publish(String(data="RTH:MAVROS_LOST"))
            return

        stale = self._stale_nodes()
        if stale:
            self.rth_already_triggered = True
            self._log_safety_event("NODE_HEARTBEAT_LOST", f"stale topics: {stale}")
            self.cmd_pub.publish(String(data="RTH:NODE_HEARTBEAT_LOST"))

    def _log_safety_event(self, event_type, detail):
        self.get_logger().error(f"[FAILSAFE] {event_type}: {detail}")
        self.safety_event_pub.publish(String(data=json.dumps(
            {"event_type": event_type, "detail": detail})))


def main(args=None):
    rclpy.init(args=args)
    node = FailsafeMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
