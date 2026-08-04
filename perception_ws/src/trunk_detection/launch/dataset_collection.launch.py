#!/usr/bin/env python3
"""
数据集采集一键启动文件
========================================================

同时启动 Orbbec 相机和数据集采集节点。

一键启动（在 trunk_tracking_ws 目录下执行）：
  source install/setup.bash
  ros2 launch trunk_detection dataset_collection.launch.py

可选参数覆盖：
  ros2 launch trunk_detection dataset_collection.launch.py \
    save_interval:=0.5 \
    output_dir:=/home/czb/my_dataset \
    image_topic:=/camera/color/image_raw \
    show_preview:=true

参数说明：
  save_interval  — 自动保存间隔（秒），默认 1.0
  output_dir     — 数据集保存根目录，默认 ~/trunk_dataset
  image_topic    — 相机图像话题，默认 /camera/color/image_raw
  show_preview   — 是否显示实时预览，默认 true
========================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 启动参数声明 ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'save_interval',
            default_value='1.0',
            description='自动保存间隔（秒），例如 0.5 表示每 0.5 秒保存一张'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='~/trunk_dataset',
            description='数据集保存根目录'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
            description='相机彩色图像话题'
        ),
        DeclareLaunchArgument(
            'show_preview',
            default_value='true',
            description='是否显示实时预览窗口（true/false）'
        ),
    ]

    # ── Orbbec 相机节点（使用 ob_camera.launch.py 通用启动文件） ──
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'ob_camera.launch.py',
            ])
        ),
        launch_arguments={
            'camera_name':                  'camera',
            'enable_color':                 'true',
            'color_width':                  '640',
            'color_height':                 '480',
            'color_fps':                    '30',
            'color_format':                 'MJPG',
            'enable_depth':                 'false',   # 采集数据集不需要深度
            'enable_ir':                    'false',
            'enable_point_cloud':           'false',
            'enable_colored_point_cloud':   'false',
            'connection_delay':             '100',
        }.items()
    )

    # ── 数据集采集节点（延迟 3 秒启动，等待相机完成初始化） ────────
    collector_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='trunk_detection',
                executable='dataset_collector',
                name='dataset_collector',
                output='screen',
                parameters=[{
                    'image_topic':   LaunchConfiguration('image_topic'),
                    'save_interval': LaunchConfiguration('save_interval'),
                    'output_dir':    LaunchConfiguration('output_dir'),
                    'show_preview':  LaunchConfiguration('show_preview'),
                }],
            )
        ]
    )

    return LaunchDescription(args + [orbbec_launch, collector_node])
