#!/usr/bin/env python3
"""
树干跟踪系统最小启动文件 (不包含相机)
用于测试和调试算法节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成最小启动描述"""
    
    # 获取包路径
    detection_pkg = get_package_share_directory('trunk_detection')
    navigation_pkg = get_package_share_directory('navigation_controller')
    chassis_pkg = get_package_share_directory('chassis_control')
    
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'model_path',
            default_value='best.pt',
            description='YOLO模型文件路径'
        ),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='输入图像话题（仿真一般用 /camera/image_raw）'
        ),
        
        DeclareLaunchArgument(
            'serial_port', 
            default_value='/dev/ttyUSB0',
            description='串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'test_mode',
            default_value='normal',
            description='测试模式: normal, test, test_only'
        ),
        DeclareLaunchArgument(
            'image_source',
            default_value='',
            description='可选测试图像或视频路径'
        ),
        
        # 1. 图像发布节点 (用于测试，发布静态图像或视频)
        Node(
            package='trunk_tracking_launch',
            executable='image_publisher_test',
            name='image_publisher_test',
            parameters=[
                {'image_source': LaunchConfiguration('image_source')},
                {'loop_video': True},
                {'fps': 10.0}
            ],
            remappings=[
                ('/test_image', '/camera/color/image_raw')
            ],
            output='screen',
        ),
        
        # 2. 启动检测系统
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(detection_pkg, 'launch', 'trunk_detection.launch.py')
                    ),
                    launch_arguments={
                        'model_path': LaunchConfiguration('model_path'),
                        'image_topic': LaunchConfiguration('image_topic'),
                    }.items()
                )
            ]
        ),
        
        # 3. 启动导航控制
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(navigation_pkg, 'launch', 'navigation_controller.launch.py')
                    )
                )
            ]
        ),
        
        # 4. 启动底盘控制 (可选)
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(chassis_pkg, 'launch', 'chassis_control.launch.py')
                    ),
                    launch_arguments={
                        'serial_port': LaunchConfiguration('serial_port'),
                        'test_mode': LaunchConfiguration('test_mode')
                    }.items()
                )
            ]
        ),
    ])
