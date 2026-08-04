#!/usr/bin/env python3
"""
Trunk detection visualization helper.

This launches rqt_image_view subscribed to a given image topic.

Example:
  ros2 launch trunk_detection trunk_detection_viz.launch.py topic:=/trunk_detection/detection_image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topic = LaunchConfiguration('topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'topic',
            default_value='/trunk_detection/detection_image',
            description='sensor_msgs/Image topic to visualize in rqt_image_view',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', topic],
            output='screen',
        ),
    ])




