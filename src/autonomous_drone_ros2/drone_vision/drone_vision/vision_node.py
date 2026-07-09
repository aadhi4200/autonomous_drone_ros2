#!/usr/bin/env python3
"""
vision_node.py | Package: drone_vision
Detects ArUco ID=17 (DICT_6X6_250).
Publishes pixel offset to /vision/landing_target as PointStamped:
  x = pixel offset from center (+ right)
  y = pixel offset from center (+ down)
  z = marker area (larger = closer to ground)
"""
import rclpy, cv2, numpy as np
import cv2.aruco as aruco
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
from drone_interfaces.constants import ARUCO_MARKER_ID, TOPIC_VISION_TARGET_ID, TOPIC_VISION_HEARTBEAT

IMAGE_W   = 640
IMAGE_H   = 480

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.bridge       = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Compatible with OpenCV 4.5.x (apt-packaged) and newer (4.7+)
        if hasattr(aruco, "ArucoDetector"):
            self.aruco_params = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_params = aruco.DetectorParameters_create()
            self.detector = None  # use module-level aruco.detectMarkers(...) instead

        # Expected marker ID for the *current* waypoint — set per stop by
        # aruco_landing_node so SEARCH looks for the right marker, not a
        # single global constant (see aruco_landing_node._cmd_cb).
        self.target_id = ARUCO_MARKER_ID

        self.cx = IMAGE_W / 2.0
        self.cy = IMAGE_H / 2.0

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)
        self.create_subscription(Image, "/camera/image_raw", self._image_cb, sensor_qos)
        self.create_subscription(Int32, TOPIC_VISION_TARGET_ID, self._target_id_cb, 10)
        self.target_pub = self.create_publisher(PointStamped, "/vision/landing_target", 10)
        self.heartbeat_pub = self.create_publisher(String, TOPIC_VISION_HEARTBEAT, 10)
        self.create_timer(0.5, self._pub_heartbeat)
        self.get_logger().info(f"VisionNode ready — detecting ArUco ID={self.target_id}")

    def _target_id_cb(self, msg):
        if msg.data != self.target_id:
            self.get_logger().info(f"Target marker ID → {msg.data}")
        self.target_id = msg.data

    def _pub_heartbeat(self):
        self.heartbeat_pub.publish(String(data="ALIVE"))

    def _detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0); return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        if ids is None: return

        for i, mid in enumerate(ids.flatten()):
            if mid != self.target_id: continue
            c          = corners[i][0]
            marker_cx  = float(np.mean(c[:, 0]))
            marker_cy  = float(np.mean(c[:, 1]))
            offset_x   = marker_cx - self.cx
            offset_y   = marker_cy - self.cy
            w = np.linalg.norm(c[0]-c[1]); h = np.linalg.norm(c[1]-c[2])
            area = float(w * h)

            pt = PointStamped()
            pt.header.stamp    = self.get_clock().now().to_msg()
            pt.header.frame_id = "camera"
            pt.point.x = offset_x
            pt.point.y = offset_y
            pt.point.z = area
            self.target_pub.publish(pt)
            self.get_logger().info(
                f"ArUco ID={mid} | offset=({offset_x:.1f},{offset_y:.1f})px | area={area:.0f}",
                throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
