import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('agri_vslam_bringup'),
        'rviz',
        'camera_only_results.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config,
            description='RViz config for camera-only map, scan, TF, and odometry results.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_camera_only_results',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
