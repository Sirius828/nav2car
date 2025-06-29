import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# this launch file inputs depth image and laser scans, merges them, and outputs a combined laser scan

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rb21_localization'),
        'config',
        'localization.yaml'
    )
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='ros2_laser_scan_merger',
            executable='ros2_laser_scan_merger',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=2,
        ),

        launch_ros.actions.Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            parameters=[config],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan_depth')
            ],
        ),

        launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[config],
            remappings=[
                ('cloud_in', 'cloud_merged'),
                ('scan', 'scan_merged')
            ],
        ),
    ])
