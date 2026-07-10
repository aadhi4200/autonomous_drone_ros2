"""px4_sitl.launch.py — PX4 SITL + Gazebo only, standalone (Terminal 1
equivalent). Run this on its own, watch its output for "Startup script
returned successfully", then start MAVROS yourself (`ros2 launch mavros
px4.launch fcu_url:=...`) — matches the separate-terminal workflow in
README.md rather than chaining automatically into the next stage.

For the one-command everything-at-once version, use full_stack.launch.py
instead — it reuses the exact same process definitions (see px4_stack.py)
plus event-handler gating between stages.

Prerequisite (not done here — a build step, not a launch step):
    cd ~/PX4-Autopilot && make px4_sitl_default
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration

from drone_bringup import px4_stack


def generate_launch_description():
    gz_world = LaunchConfiguration('gz_world')
    headless = LaunchConfiguration('headless')

    gazebo_process = px4_stack.make_gazebo_process(gz_world, headless)
    px4_process = px4_stack.make_px4_process(gz_world)

    def on_px4_stdout(event):
        if b'Startup script returned successfully' in event.text:
            return [LogInfo(msg='✅ PX4 SITL ready — start MAVROS yourself now '
                                 '(ros2 launch mavros px4.launch fcu_url:=...)')]
        return None

    return LaunchDescription([
        DeclareLaunchArgument('gz_world', default_value='aruco_landing'),
        DeclareLaunchArgument('headless', default_value='0',
                              description='1 = no Gazebo GUI window'),
        gazebo_process,
        px4_process,
        RegisterEventHandler(OnProcessIO(target_action=px4_process, on_stdout=on_px4_stdout)),
    ])
