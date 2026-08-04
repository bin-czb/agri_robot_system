#!/usr/bin/env python3
"""
导航控制系统启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('navigation_controller')
    config_path = os.path.join(pkg_path, 'config', 'navigation_params.yaml')
    
    return LaunchDescription([
        # 启动参数
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
        
        # 导航控制节点
        Node(
            package='navigation_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[
                config_path,
                {
                    'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                    'max_angular_velocity': LaunchConfiguration('max_angular_velocity')
                }
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])

