import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    agri_nav2_dir = get_package_share_directory('agri_nav2_config')

    default_params = os.path.join(
        agri_nav2_dir,
        'config',
        'nav2_minimal_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use wall time for the formal camera/chassis chain.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level Nav2 namespace. Keep empty for the current single-robot setup.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters for the formal EKF + RTAB-Map chain.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed Nav2 bringup.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn Nav2 nodes if they crash. Keep false during bringup.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Nav2 log level.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': LaunchConfiguration('use_composition'),
                'use_respawn': LaunchConfiguration('use_respawn'),
                'log_level': LaunchConfiguration('log_level'),
            }.items(),
        ),
    ])
