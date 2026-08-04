#!/usr/bin/env python3
"""
行偏置 + 停靠（Gazebo 仿真验证用）

目标：
- 仅使用左侧树干点拟合“左侧树行”
- 将左侧树行向右偏置 1.5m 作为导航线（车辆应沿该偏置线行走）
- 接近每棵树时减速，到达后停止 2 秒

说明：该 launch 只启动 navigation_controller（不包含检测与相机）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('navigation_controller')
    params_file = os.path.join(pkg_path, 'config', 'navigation_row_offset_stop.yaml')

    return LaunchDescription([
        Node(
            package='navigation_controller',
            executable='navigation_controller',
            name='navigation_controller',
            parameters=[params_file],
            output='screen',
            emulate_tty=True,
        ),
    ])





