#!/usr/bin/env python3
"""
深度增强版导航控制节点启动文件（备用）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 包路径与默认配置
    pkg_path = get_package_share_directory('navigation_controller')
    config_path = os.path.join(pkg_path, 'config', 'navigation_params.yaml')

    return LaunchDescription([
        # 可选：覆盖最大速度
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='0.5',
            description='Maximum linear velocity in m/s'
        ),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='1.0',
            description='Maximum angular velocity in rad/s'
        ),
        # 深度相关参数
        DeclareLaunchArgument(
            'use_depth_estimation',
            default_value='true',
            description='Use depth image for distance estimation'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Depth image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Depth camera info topic'
        ),
        DeclareLaunchArgument(
            'depth_window_size',
            default_value='5',
            description='Median window size (pixels) for depth'
        ),
        DeclareLaunchArgument(
            'depth_min',
            default_value='0.2',
            description='Minimum valid depth (m)'
        ),
        DeclareLaunchArgument(
            'depth_max',
            default_value='8.0',
            description='Maximum valid depth (m)'
        ),
        DeclareLaunchArgument(
            'depth_scale_m',
            default_value='0.001',
            description='Depth scale to meters (16UC1: 0.001, 32FC1: 1.0)'
        ),

        Node(
            package='navigation_controller',
            executable='navigation_controller_depth',
            name='navigation_controller_depth',
            parameters=[
                config_path,
                {
                    'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                    'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                    'use_depth_estimation': LaunchConfiguration('use_depth_estimation'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'depth_window_size': LaunchConfiguration('depth_window_size'),
                    'depth_min': LaunchConfiguration('depth_min'),
                    'depth_max': LaunchConfiguration('depth_max'),
                    'depth_scale_m': LaunchConfiguration('depth_scale_m'),
                }
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])



