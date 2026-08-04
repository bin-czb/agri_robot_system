import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('agri_vslam_bringup')
    rviz_config = os.path.join(package_share, 'rviz', 'vslam_mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_stack',
            default_value='true',
            description=(
                'Start the complete camera-only RGB-D mapping chain before '
                'RViz. Set false only when that chain is already running.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_delay',
            default_value='8.0',
            description='Delay RViz until camera and RTAB-Map topics exist.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'rgbd_vslam_camera_only_stable_scan.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('start_stack')),
        ),
        TimerAction(
            period=LaunchConfiguration('rviz_delay'),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_vslam_mapping',
                    output='screen',
                    arguments=['-d', rviz_config],
                ),
            ],
        ),
    ])
