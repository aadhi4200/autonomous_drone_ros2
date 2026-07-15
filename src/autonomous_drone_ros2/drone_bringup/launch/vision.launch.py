"""vision.launch.py — camera bridge + ArUco vision, sim/hardware aware.

`mode:=sim` (default) starts the Gazebo->ROS2 camera bridge (the manual
Terminal-3 `ros_gz_bridge parameter_bridge` step from the README) plus
camera_node + vision_node.

`mode:=hardware` skips the Gazebo bridge and expects a real camera driver
node to already be publishing /camera/image_raw (e.g. an IMX219/OAK-D Lite
node, per the project's Phase 3/4 hardware plan) — that driver does not
exist in this repo yet, so this just logs what's missing rather than
launching a fake stand-in.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

GAZEBO_CAM_TOPIC = "/world/default/model/x500_lidar_cam_down_0/link/camera_link/sensor/camera/image"


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    is_sim = PythonExpression(["'", mode, "' == 'sim'"])
    is_hardware = PythonExpression(["'", mode, "' == 'hardware'"])
    # Nodes follow Gazebo /clock in sim (bridged below) so their timers and
    # timeouts scale with the sim's real-time factor; wall clock on hardware.
    sim_time_params = [{'use_sim_time': is_sim}]

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                               description="'sim' (Gazebo camera bridge) or 'hardware' (real camera driver)"),

        ExecuteProcess(
            condition=IfCondition(is_sim),
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 f'{GAZEBO_CAM_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image',
                 # one-way gz->ROS /clock bridge: the single source of sim
                 # time for every node launched with use_sim_time
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        # camera_node just republishes the Gazebo-bridged topic onto
        # /camera/image_raw — sim-only. On real hardware the camera driver
        # node publishes /camera/image_raw itself, so camera_node has
        # nothing to do there.
        Node(package='drone_camera', executable='camera_node', name='camera_node',
             output='screen', condition=IfCondition(is_sim), parameters=sim_time_params),
        # vision_node is source-agnostic — runs in both modes unchanged.
        Node(package='drone_vision', executable='vision_node', name='vision_node',
             output='screen', parameters=sim_time_params),

        LogInfo(
            condition=IfCondition(is_hardware),
            msg="mode=hardware: no Gazebo bridge started. A real camera driver node "
                "must publish /camera/image_raw itself (not yet implemented in this repo — "
                "wire your IMX219/OAK-D driver here)."),
    ])
