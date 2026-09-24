import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('aruco_opencv_bringup'),
        'config', 'aruco_opencv_params.yaml')

    aruco_tracker_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        aruco_tracker_node,
    ])