from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    create_repub_share = get_package_share_directory("create3_republisher")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(create_repub_share, "bringup", "create3_republisher_launch.py")
                ),
                launch_arguments=[
                    ('robot_ns', '/create3'),
                    ('republisher_ns', '/'),
                ],
            ),

        ]
    )
