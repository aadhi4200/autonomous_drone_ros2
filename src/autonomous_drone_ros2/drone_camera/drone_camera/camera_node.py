#!/usr/bin/env python3
"""
camera_node.py | Package: drone_camera
Bridges Gazebo camera to /camera/image_raw for vision pipeline.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from drone_interfaces.constants import TOPIC_CAMERA_HEARTBEAT

GAZEBO_CAM = "/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image"

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("show_preview", False)
        self.show_preview = self.get_parameter("show_preview").value
        self.bridge = CvBridge()
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE, depth=10)
        self.create_subscription(Image, GAZEBO_CAM, self._image_cb, sensor_qos)
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.heartbeat_pub = self.create_publisher(String, TOPIC_CAMERA_HEARTBEAT, 10)
        self.create_timer(0.5, lambda: self.heartbeat_pub.publish(String(data="ALIVE")))
        self.get_logger().info("CameraNode started.")

    def _image_cb(self, msg):
        self.image_pub.publish(msg)
        if self.show_preview:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow("Drone Camera", frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(str(e), throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
