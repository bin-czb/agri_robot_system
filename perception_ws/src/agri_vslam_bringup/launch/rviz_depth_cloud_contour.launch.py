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
        'depth_cloud_contour.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config,
            description='RViz config for raw depth point cloud contours.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_depth_cloud_contour',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
