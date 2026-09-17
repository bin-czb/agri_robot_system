"""Start navigation-side adapters; sensor drivers are started separately."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('selector_params', default_value=os.path.join(
            share('agri_localization_selector'), 'config', 'selector.yaml')),
        DeclareLaunchArgument('rtk_params', default_value=os.path.join(
            share('agri_map_integration'), 'config', 'rtk_uav_test_map.yaml')),
        DeclareLaunchArgument('aoa_params', default_value=os.path.join(
            share('agri_global_localization'), 'config', 'aoa_uav_test_map.yaml')),
        Node(package='agri_rtk_localization', executable='rtk_localizer',
             name='rtk_global_localizer', parameters=[
                 os.path.join(share('agri_rtk_localization'), 'config', 'rtk_localization.yaml'),
                 LaunchConfiguration('rtk_params'), clock,
                 {'publish_selected': False, 'position_alpha': 1.0}], output='screen'),
        Node(package='agri_global_localization', executable='aoa_map_transform',
             name='aoa_utm_to_map', parameters=[LaunchConfiguration('aoa_params'), clock], output='screen'),
        Node(package='agri_localization_selector', executable='localization_selector',
             name='localization_selector',
             parameters=[LaunchConfiguration('selector_params'), clock], output='screen'),
    ])
