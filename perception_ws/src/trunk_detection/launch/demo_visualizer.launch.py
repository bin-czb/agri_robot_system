#!/usr/bin/env python3
"""Launch the standalone demo visualizer dashboard.

Example:
  ros2 launch trunk_detection demo_visualizer.launch.py
  ros2 launch trunk_detection demo_visualizer.launch.py \
      image_topic:=/camera/color/image_raw \
      gyro_topic:=/camera/gyro \
      accel_topic:=/camera/accel \
      display_fps:=15 \
      display_image_width:=720
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('detection_topic', default_value='/trunk_detection/detections'),
        DeclareLaunchArgument('line_topic', default_value='/trunk_detection/trunk_line'),
        DeclareLaunchArgument('gyro_topic', default_value='/camera/gyro/sample'),
        DeclareLaunchArgument('accel_topic', default_value='/camera/accel/sample'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_max_range_m', default_value='5.0'),
        DeclareLaunchArgument('lidar_view_height', default_value='280'),
        DeclareLaunchArgument('display_fps', default_value='15.0'),
        DeclareLaunchArgument('display_image_width', default_value='720'),
        DeclareLaunchArgument('window_name', default_value='Trunk Tracking - Demo Dashboard'),

        Node(
            package='trunk_detection',
            executable='demo_visualizer',
            name='demo_visualizer',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'detection_topic': LaunchConfiguration('detection_topic'),
                'line_topic': LaunchConfiguration('line_topic'),
                'gyro_topic': LaunchConfiguration('gyro_topic'),
                'accel_topic': LaunchConfiguration('accel_topic'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'lidar_max_range_m': LaunchConfiguration('lidar_max_range_m'),
                'lidar_view_height': LaunchConfiguration('lidar_view_height'),
                'display_fps': LaunchConfiguration('display_fps'),
                'display_image_width': LaunchConfiguration('display_image_width'),
                'window_name': LaunchConfiguration('window_name'),
            }],
        ),
    ])
