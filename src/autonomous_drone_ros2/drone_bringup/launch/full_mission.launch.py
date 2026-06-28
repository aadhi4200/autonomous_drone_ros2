"""full_mission.launch.py — launches ALL nodes in one command"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_camera',     executable='camera_node',        name='camera_node',      output='screen'),
        Node(package='drone_vision',     executable='vision_node',        name='vision_node',      output='screen'),
        Node(package='drone_base',       executable='drone_base_node',    name='drone_base',       output='screen'),
        Node(package='drone_controller', executable='waypoint_navigator', name='waypoint_nav',     output='screen'),
        Node(package='drone_controller', executable='aruco_landing_node', name='aruco_landing',    output='screen'),
        Node(package='drone_mission',    executable='mission_manager',    name='mission_manager',  output='screen'),
    ])
