"""full_mission.launch.py — launches ALL nodes in one command.

Composed from vision.launch.py (camera/vision, sim/hardware aware) +
drone_nodes.launch.py (base/controller/mission/failsafe) rather than
duplicating node definitions, so the sim/hardware branching only lives
in one place.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim',
                               description="'sim' or 'hardware' — forwarded to vision.launch.py"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'vision.launch.py')),
            launch_arguments={'mode': mode}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'drone_nodes.launch.py')),
        ),
    ])
