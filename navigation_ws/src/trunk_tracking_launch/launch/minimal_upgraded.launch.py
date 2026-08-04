#!/usr/bin/env python3
"""
最小化升级版系统启动文件
仅启动升级版串口通讯和基本检测功能
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成最小化启动描述"""
    
    # 启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', 
        default_value='115200',
        description='串口波特率'
    )
    
    # 升级版串口通讯节点
    serial_bridge_node = Node(
        package='serial_bridge_ros2',
        executable='serial_bridge_node',
        name='upgraded_serial_bridge',
        namespace='',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'slave_id': 1,
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        serial_bridge_node,
    ])
