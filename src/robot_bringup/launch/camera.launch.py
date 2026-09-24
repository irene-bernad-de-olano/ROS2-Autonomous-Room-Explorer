from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument # Nicht mehr benötigt
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration # Nicht mehr benötigt
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess


def generate_launch_description():

    return LaunchDescription(
        [
             DeclareLaunchArgument(
                 "camera_name", default_value="camera", description="Camera unique name"
             ),
             DeclareLaunchArgument(
                 "camera_namespace", default_value="camera", description="Camera namespace",
             ),
             DeclareLaunchArgument(
                 "enable_color", default_value="true", description="Enable color stream"
             ),
             DeclareLaunchArgument(
                 "enable_depth", default_value="false", description="Enable depth stream"
             ),
             DeclareLaunchArgument(
                 "pointcloud_enable", default_value="false", description="Enable pointcloud",
             ),
             DeclareLaunchArgument(
                 "accel_enable", default_value="false", description="Enable Acceleration",
             ),
             DeclareLaunchArgument(
                 "gyro_enable", default_value="false", description="Enable Gyro",
             ),
             DeclareLaunchArgument(
                 "imu_enable", default_value="false", description="Enable IMU",
             ),
             DeclareLaunchArgument(
                 "infra_enable", default_value="false", description="Enable Infrared Sensors",
             ),

             Node(
                 package="realsense2_camera",
                 executable="realsense2_camera_node",
                 namespace=LaunchConfiguration("camera_namespace"),
                 name=LaunchConfiguration("camera_name"),
                 parameters=[
                     {
                         "enable_color": LaunchConfiguration("enable_color"),
                         "enable_depth": LaunchConfiguration("enable_depth"),
                         "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
                         "enable_accel": LaunchConfiguration("accel_enable"),
                         "enable_gyro": LaunchConfiguration("gyro_enable"),
                         "unite_imu_topics": LaunchConfiguration("imu_enable"),
                         "enable_infra1": LaunchConfiguration("infra_enable"),
                         "enable_infra2": LaunchConfiguration("infra_enable"),
                         'rgb_camera.color_profile': '640x480x6',# Farb- (RGB) Kamera auf 480p und 5 Hz einstellen
                     }
                 ],
             ),

            ExecuteProcess( #disableling saftey features like no-backup ("full" would enalble faster max speed as well)
                cmd=['ros2', 'param', 'set', '/create3/motion_control', 'safety_override', 'backup_only'], 
                output='screen'
            ),

            ExecuteProcess( #disableling reflex features like cliff sensors and bump
                cmd=['ros2', 'param', 'set', '/create3/motion_control', 'reflexes_enabled', 'false'], # possible that the node name is wrong because of the namespace
                output='screen'
            ),

        ]
    )



# Avaliable Options for realsense Resolution and fps
#     1280x720x15
# 1280x720x30
# 1280x720x6
# 1920x1080x15
# 1920x1080x30
# 1920x1080x6
# 320x180x30
# 320x180x6
# 320x180x60
# 320x240x30
# 320x240x6
# 320x240x60
# 424x240x15
# 424x240x30
# 424x240x6
# 424x240x60
# 640x360x15
# 640x360x30
# 640x360x6
# 640x360x60
# 640x480x15
# 640x480x30
# 640x480x6
# 640x480x60
# 848x480x15
# 848x480x30
# 848x480x6
# 848x480x60
# 960x540x15
# 960x540x30
# 960x540x6