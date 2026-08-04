"""Launch the UM982 NTRIP node with its default parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('um982_ntrip')
    config = os.path.join(package_share, 'config', 'um982_ntrip.yaml')
    credentials = {
        'ntrip_username': os.environ.get('NTRIP_USERNAME', ''),
        'ntrip_password': os.environ.get('NTRIP_PASSWORD', ''),
    }
    return LaunchDescription([
        Node(
            package='um982_ntrip',
            executable='um982_ntrip_node',
            name='um982_ntrip',
            output='screen',
            parameters=[config, credentials],
        ),
    ])
