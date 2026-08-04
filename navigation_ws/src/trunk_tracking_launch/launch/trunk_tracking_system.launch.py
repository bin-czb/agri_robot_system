#!/usr/bin/env python3
"""
树干跟踪系统完整启动文件
启动所有必要的节点和组件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """启动设置函数"""
    
    # 获取包路径
    launch_pkg = get_package_share_directory('trunk_tracking_launch')
    detection_pkg = get_package_share_directory('trunk_detection')
    navigation_pkg = get_package_share_directory('navigation_controller')
    chassis_pkg = get_package_share_directory('chassis_control')
    orbbec_pkg = get_package_share_directory('orbbec_camera')
    
    # 获取启动参数
    model_path = LaunchConfiguration('model_path').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_enabled = LaunchConfiguration('camera_enabled').perform(context) == 'true'
    serial_port = LaunchConfiguration('serial_port').perform(context)
    use_orbbec_camera = LaunchConfiguration('use_orbbec_camera').perform(context) == 'true'
    debug_mode = LaunchConfiguration('debug_mode').perform(context) == 'true'
    
    # 配置文件路径
    system_config = os.path.join(launch_pkg, 'config', 'system_params.yaml')
    
    launch_actions = []
    
    # 1. 启动相机系统 (如果启用)
    if camera_enabled:
        if use_orbbec_camera:
            # 启动Orbbec相机
            orbbec_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(orbbec_pkg, 'launch', 'ob_camera.launch.py')
                ),
                launch_arguments={
                    'camera_name': 'camera',
                    'device_type': '',  # 自动检测
                    'serial_number': '',
                    'usb_port': '',
                    'enable_point_cloud': 'false',
                    'enable_colored_point_cloud': 'false',
                    'connection_delay': '100',
                    'enable_accel': 'true',
                    'accel_rate': '100hz',
                    'enable_gyro': 'true',
                    'gyro_rate': '100hz',
                    'gyro_range': '1000dps',
                }.items()
            )
            launch_actions.append(orbbec_launch)
        else:
            # 使用USB摄像头
            usb_camera_node = Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='usb_camera',
                parameters=[
                    {'video_device': '/dev/video0'},
                    {'image_size': [640, 480]},
                    {'time_per_frame': [1, 30]},
                ],
                remappings=[
                    ('/image_raw', '/camera/color/image_raw'),
                ],
                output='screen' if debug_mode else 'log',
            )
            launch_actions.append(usb_camera_node)
    
    # 2. 延时启动检测系统 (等待相机初始化)
    detection_launch = TimerAction(
        period=3.0,  # 等待3秒让相机初始化
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(detection_pkg, 'launch', 'trunk_detection.launch.py')
                ),
                launch_arguments={
                    'model_path': model_path,
                    'image_topic': image_topic,
                }.items()
            )
        ]
    )
    launch_actions.append(detection_launch)
    
    # 3. 启动导航控制系统
    navigation_launch = TimerAction(
        period=4.0,  # 再等待1秒让检测系统初始化
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(navigation_pkg, 'launch', 'navigation_controller.launch.py')
                )
            )
        ]
    )
    launch_actions.append(navigation_launch)
    
    # 4. 启动升级版底盘控制系统
    chassis_launch = TimerAction(
        period=5.0,  # 最后启动底盘控制
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(chassis_pkg, 'launch', 'chassis_control.launch.py')
                ),
                launch_arguments={
                    'serial_port': serial_port,
                    'baud_rate': '115200',  # 升级版波特率
                    'slave_id': '1',        # ModBus从机地址
                    'test_mode': 'test' if debug_mode else 'normal'
                }.items()
            )
        ]
    )
    launch_actions.append(chassis_launch)
    
    # 5. 系统监控节点 (可选)
    if debug_mode:
        system_monitor = TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='trunk_tracking_launch',
                    executable='system_monitor',
                    name='system_monitor',
                    parameters=[system_config],
                    output='screen',
                )
            ]
        )
        launch_actions.append(system_monitor)
    
    return launch_actions


def generate_launch_description():
    """生成启动描述"""
    return LaunchDescription([
        # 启动参数声明
        DeclareLaunchArgument(
            'model_path',
            default_value='best.pt',
            description='YOLO模型文件路径'
        ),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='输入图像话题（仿真一般用 /camera/image_raw，实机/Orbbec 常用 /camera/color/image_raw）'
        ),
        
        DeclareLaunchArgument(
            'camera_enabled',
            default_value='true',
            description='是否启用相机'
        ),
        
        DeclareLaunchArgument(
            'use_orbbec_camera',
            default_value='true', 
            description='是否使用Orbbec相机 (否则使用USB相机)'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='升级版底盘控制串口设备 (ModBus-RTU)'
        ),
        
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='是否启用调试模式'
        ),
        
        # 启动所有组件
        OpaqueFunction(function=launch_setup)
    ])
