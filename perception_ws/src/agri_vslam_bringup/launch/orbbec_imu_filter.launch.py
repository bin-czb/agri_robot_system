from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'raw_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='Synchronized raw accelerometer and gyroscope topic from Orbbec.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/imu/data',
            description='Madgwick orientation output consumed by RTAB-Map and EKF.',
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
        ),
    ])
