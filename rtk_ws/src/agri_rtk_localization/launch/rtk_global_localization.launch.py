from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('agri_rtk_localization'),
        'config',
        'rtk_localization.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('agri_rtk_localization'),
        'rviz',
        'rtk_localization.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock when replaying a rosbag or simulation.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Open the RTK position and trajectory RViz view.',
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='local_origin',
            description='Local ENU frame used by RTK navigation outputs.',
        ),
        DeclareLaunchArgument(
            'publish_selected',
            default_value='true',
            description='Publish /global_pose/selected in RTK mode.',
        ),
        Node(
            package='agri_rtk_localization',
            executable='rtk_localizer',
            name='rtk_global_localizer',
            output='screen',
            parameters=[
                config,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'output_frame': LaunchConfiguration('output_frame'),
                    'publish_selected': ParameterValue(
                        LaunchConfiguration('publish_selected'),
                        value_type=bool),
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rtk_localization_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
