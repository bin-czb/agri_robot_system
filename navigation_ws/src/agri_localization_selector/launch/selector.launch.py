from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('agri_localization_selector'),
                          'config', 'selector.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='agri_localization_selector', executable='localization_selector',
             name='localization_selector', output='screen', parameters=[
                 LaunchConfiguration('params_file'),
                 {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
             ]),
    ])
