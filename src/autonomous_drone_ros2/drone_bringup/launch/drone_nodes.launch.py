"""drone_nodes.launch.py — base + controller + mission (no camera/vision)"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Follow Gazebo /clock so every state-machine timeout (TAKEOFF_TIMEOUT,
    # NAV_TIMEOUT, ...) counts simulation seconds, not wall seconds. Under
    # CPU-only rendering the sim runs well below 1x real time, and wall-clock
    # timeouts fire mid-climb (INTER_TAKEOFF abort at B, seen 2026-07-15).
    # Hardware has no /clock — full_mission.launch.py passes false there.
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = [{'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true: nodes follow Gazebo /clock (sim); false on real hardware'),
        Node(package='drone_base',       executable='drone_base_node',    name='drone_base',       output='screen', parameters=params),
        Node(package='drone_controller', executable='waypoint_navigator', name='waypoint_nav',     output='screen', parameters=params),
        Node(package='drone_controller', executable='aruco_landing_node', name='aruco_landing',    output='screen', parameters=params),
        Node(package='drone_mission',    executable='mission_manager',    name='mission_manager',  output='screen', parameters=params),
        Node(package='drone_mission',    executable='failsafe_monitor',   name='failsafe_monitor', output='screen', parameters=params),
    ])
