from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    filter_config = PathJoinSubstitution([
        FindPackageShare('agri_global_localization'),
        'config', 'aoa_global_filter.yaml',
    ])
    map_config = PathJoinSubstitution([
        FindPackageShare('agri_global_localization'),
        'config', 'aoa_uav_test_map.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_uav_gps',
            default_value='true',
            description='Start the UAV MAVLink UTM position source.',
        ),
        DeclareLaunchArgument(
            'start_aoa_source',
            default_value='true',
            description='Start the external AOA localization source.',
        ),
        DeclareLaunchArgument(
            'mavlink_port',
            default_value='/dev/ttyUSB0',
            description='Serial device used by the UAV MAVLink receiver.',
        ),
        DeclareLaunchArgument(
            'aoa_serial_port',
            default_value='/dev/ttyACM0',
            description='Serial device used by the AOA base station.',
        ),
        DeclareLaunchArgument(
            'heading_mode',
            default_value='imu_relative',
            description='Ground rig ENU heading source used by AOA.',
        ),
        DeclareLaunchArgument(
            'initial_heading_enu_deg',
            default_value='0.0',
            description='Initial rig heading in ENU: east=0, north=90.',
        ),
        DeclareLaunchArgument(
            'base_imu_topic',
            default_value='/camera/imu/data',
            description='Filtered IMU topic rigidly attached to the AOA rig.',
        ),
        DeclareLaunchArgument(
            'base_odometry_topic',
            default_value='/odometry/filtered',
            description='Continuous local odometry used for AOA prediction.',
        ),
        DeclareLaunchArgument(
            'use_odometry',
            default_value='false',
            description=(
                'Predict between AOA corrections using local odometry.'),
        ),
        DeclareLaunchArgument(
            'start_map_transform',
            default_value='true',
            description='Convert the absolute AOA UTM pose into map.',
        ),
        DeclareLaunchArgument(
            'publish_selected',
            default_value='true',
            description='Publish AOA on /global_pose/selected.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_aoa_driver'),
                    'launch',
                    'aoa_driver.launch.py',
                ])
            ),
            launch_arguments={
                'start_uav_gps': LaunchConfiguration('start_uav_gps'),
                'start_aoa_source': LaunchConfiguration('start_aoa_source'),
                'mavlink_port': LaunchConfiguration('mavlink_port'),
                'aoa_serial_port': LaunchConfiguration('aoa_serial_port'),
                'heading_mode': LaunchConfiguration('heading_mode'),
                'initial_heading_enu_deg': LaunchConfiguration(
                    'initial_heading_enu_deg'),
                'base_imu_topic': LaunchConfiguration('base_imu_topic'),
                'base_odometry_topic': LaunchConfiguration(
                    'base_odometry_topic'),
            }.items(),
        ),
        Node(
            package='agri_global_localization',
            executable='aoa_map_transform',
            name='aoa_utm_to_map',
            output='screen',
            parameters=[map_config],
            condition=IfCondition(
                LaunchConfiguration('start_map_transform')),
        ),
        Node(
            package='agri_global_localization',
            executable='aoa_pose_filter',
            name='aoa_global_pose_filter',
            output='screen',
            parameters=[
                filter_config,
                {
                    'use_odometry': ParameterValue(
                        LaunchConfiguration('use_odometry'),
                        value_type=bool,
                    ),
                    'odometry_topic': LaunchConfiguration(
                        'base_odometry_topic'),
                    'publish_selected': ParameterValue(
                        LaunchConfiguration('publish_selected'),
                        value_type=bool),
                },
            ],
        ),
    ])
