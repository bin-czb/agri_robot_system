#!/usr/bin/env python3
"""
升级版底盘控制系统启动文件
支持高级ModBus-RTU串口通信
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # 获取包路径
    pkg_path = get_package_share_directory('chassis_control')
    config_path = os.path.join(pkg_path, 'config', 'chassis_params.yaml')
    
    # 获取启动参数
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baud_rate = LaunchConfiguration('baud_rate').perform(context)
    slave_id = LaunchConfiguration('slave_id').perform(context)
    test_mode = LaunchConfiguration('test_mode').perform(context)
    
    nodes = []
    
    # 升级版底盘控制节点
    if test_mode != 'test_only':
        chassis_node = Node(
            package='chassis_control',
            executable='chassis_controller',
            name='chassis_controller',
            parameters=[
                config_path,
                {
                    'serial_port': serial_port,
                    'baud_rate': int(baud_rate),
                    'device_id': int(slave_id),
                    'max_linear_speed': 1.0,
                    'max_angular_speed': 1.0,
                    'track_width': 1.12,
                    'control_frequency': 10.0,
                    'enable_feedback': True,
                    'watchdog_timeout': 2.0,
                    'emergency_stop_enabled': True,
                }
            ],
            output='screen',
            emulate_tty=True,
            prefix='stdbuf -o L',  # 无缓冲输出
        )
        nodes.append(chassis_node)
    
    # 串口测试节点 (可选)
    if test_mode in ['test', 'test_only']:
        test_node = Node(
            package='chassis_control',
            executable='serial_test',
            name='serial_test',
            parameters=[
                config_path,
                {
                    'serial_port': serial_port,
                    'baud_rate': int(baud_rate),
                    'device_id': int(slave_id)
                }
            ],
            output='screen',
            emulate_tty=True,
        )
        nodes.append(test_node)
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port device path for ModBus-RTU communication'
        ),
        
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial port baud rate (default: 115200 for upgraded version)'
        ),
        
        DeclareLaunchArgument(
            'slave_id',
            default_value='1',
            description='ModBus slave ID'
        ),
        
        DeclareLaunchArgument(
            'test_mode',
            default_value='normal',
            description='Test mode: normal, test, test_only'
        ),
        
        # 启动节点
        OpaqueFunction(function=launch_setup)
    ])
