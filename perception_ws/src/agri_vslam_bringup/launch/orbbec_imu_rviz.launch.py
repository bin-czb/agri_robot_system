import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('agri_vslam_bringup')
    rviz_config = os.path.join(package_share, 'rviz', 'orbbec_imu.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the Orbbec camera and synchronized IMU streams.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Open the dedicated IMU RViz view.',
        ),
        DeclareLaunchArgument(
            'start_filter',
            default_value='true',
            description='Start Madgwick. Disable when /camera/imu/data already exists.',
        ),
        DeclareLaunchArgument(
            'raw_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='Raw synchronized Orbbec IMU topic.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/imu/data',
            description='Madgwick orientation output topic.',
        ),
        DeclareLaunchArgument(
            'fixed_frame',
            default_value='imu_world',
            description='Visualization-only world frame.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'orbbec_gemini_imu.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='orbbec_imu_filter',
            output='screen',
            parameters=[{
                'stateless': False,
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'gain': 0.1,
                'zeta': 0.0,
                'orientation_stddev': 0.05,
            }],
            remappings=[
                ('imu/data_raw', LaunchConfiguration('raw_imu_topic')),
                ('imu/data', LaunchConfiguration('filtered_imu_topic')),
            ],
            condition=IfCondition(LaunchConfiguration('start_filter')),
        ),
        Node(
            package='agri_vslam_bringup',
            executable='imu_visualizer.py',
            name='orbbec_imu_visualizer',
            output='screen',
            parameters=[{
                'fixed_frame': LaunchConfiguration('fixed_frame'),
                'input_topic': LaunchConfiguration('filtered_imu_topic'),
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_visual_reference_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', LaunchConfiguration('fixed_frame'),
                '--child-frame-id', 'imu_visual_reference',
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_orbbec_imu',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
