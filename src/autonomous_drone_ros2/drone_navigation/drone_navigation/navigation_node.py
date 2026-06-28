#!/usr/bin/env python3
"""
navigation_node.py | Package: drone_navigation
Phase 5+ — Nav2, SLAM, obstacle avoidance.
Currently a placeholder node.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.get_logger().info("NavigationNode placeholder — Phase 5+ (Nav2 + SLAM)")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
