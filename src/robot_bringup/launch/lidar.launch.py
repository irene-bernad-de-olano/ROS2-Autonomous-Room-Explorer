from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get share directory of the LIDAR package
    lidar_share = get_package_share_directory("rplidar_ros")

    return LaunchDescription([
        # Include the LIDAR launch file with frame_id set
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_share, "launch", "rplidar_c1_launch.py")
            ),
            launch_arguments={"frame_id": "laser_frame"}.items()
        ),
    ])
