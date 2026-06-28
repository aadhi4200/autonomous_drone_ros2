"""vision.launch.py — camera bridge + ArUco vision only"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_camera', executable='camera_node', name='camera_node', output='screen'),
        Node(package='drone_vision', executable='vision_node', name='vision_node', output='screen'),
    ])
