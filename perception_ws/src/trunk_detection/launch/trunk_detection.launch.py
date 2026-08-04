#!/usr/bin/env python3
"""
树干检测系统启动文件
启动检测节点和直线拟合节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # 获取包路径
    pkg_path = get_package_share_directory('trunk_detection')
    config_path = os.path.join(pkg_path, 'config', 'detection_params.yaml')
    
    # 获取启动参数
    model_path = LaunchConfiguration('model_path').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    center_offset = LaunchConfiguration('center_offset_pixels').perform(context)
    
    # 树干检测节点
    trunk_detector_node = Node(
        package='trunk_detection',
        executable='trunk_detector',
        name='trunk_detector',
        parameters=[
            config_path,
            {'model_path': model_path},
            {'image_topic': image_topic}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # 直线拟合节点
    line_fitter_node = Node(
        package='trunk_detection',
        executable='line_fitter', 
        name='line_fitter',
        parameters=[
            config_path,
            {'image_topic': image_topic},
            {'center_offset_pixels': int(center_offset)} # 传递偏置参数
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return [trunk_detector_node, line_fitter_node]


def generate_launch_description():
    return LaunchDescription([
        # 启动参数声明
        DeclareLaunchArgument(
            'model_path',
            default_value='best.pt',
            description='Path to YOLO model file'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='Input image topic'
        ),
        DeclareLaunchArgument(
            'center_offset_pixels',
            default_value='200',
            description='Target line offset from tree row (pixels)'
        ),
        
        # 启动节点
        OpaqueFunction(function=launch_setup)
    ])
