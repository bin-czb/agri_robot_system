#!/usr/bin/env python3
"""
系统集成启动文件
启动完整的履带底盘跟踪系统，包括升级版串口通讯
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
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
    
    slave_id_arg = DeclareLaunchArgument(
        'slave_id',
        default_value='1',
        description='ModBus从机地址'
    )
    
    # 升级版串口通讯节点
    serial_bridge_node = Node(
        package='serial_bridge_ros2',
        executable='serial_bridge_node',
        name='serial_bridge',
        namespace='',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'slave_id': LaunchConfiguration('slave_id'),
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'cmd_vel_topic': '/cmd_vel',
            'battery_voltage_topic': '/battery_voltage',
            'modbus_connected_topic': '/modbus_connected',
            'chassis_status_topic': '/chassis_status',
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # 底盘控制节点（如果需要保留原有的）
    chassis_control_node = Node(
        package='chassis_control',
        executable='chassis_controller',
        name='chassis_controller',
        namespace='',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('slave_id'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    # 躯干检测节点
    trunk_detection_node = Node(
        package='trunk_detection',
        executable='trunk_detector',
        name='trunk_detector',
        namespace='',
        output='screen',
        emulate_tty=True,
    )
    
    # 导航控制节点
    navigation_node = Node(
        package='navigation_controller',
        executable='navigation_controller',
        name='navigation_controller',
        namespace='',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        slave_id_arg,
        
        # 分组启动 - 升级版串口通讯
        GroupAction([
            serial_bridge_node,
        ], scoped=False),
        
        # 分组启动 - 检测和导航
        GroupAction([
            trunk_detection_node,
            navigation_node,
        ], scoped=False),
        
        # 可选：保留原有底盘控制节点（如果需要）
        # chassis_control_node,
    ])
