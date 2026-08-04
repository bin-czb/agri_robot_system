from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the Orbbec RGB-D and synchronized raw IMU source.',
        ),
        DeclareLaunchArgument(
            'filter_delay',
            default_value='3.0',
            description='Delay the corrector and Madgwick until the camera is initialized.',
        ),
        DeclareLaunchArgument(
            'calibration_profile',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                '.ros',
                'agri_rig_calibration.yaml',
            ]),
            description='Accepted six-position IMU calibration profile.',
        ),
        DeclareLaunchArgument(
            'raw_imu_topic',
            default_value='/camera/gyro_accel/sample',
            description='Synchronized raw Orbbec accelerometer and gyroscope.',
        ),
        DeclareLaunchArgument(
            'calibrated_imu_topic',
            default_value='/camera/imu/calibrated_raw',
            description='Bias, gain, and covariance corrected raw IMU.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/imu/data',
            description='Madgwick orientation output for later EKF use.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'orbbec_gemini_slam.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        TimerAction(
            period=LaunchConfiguration('filter_delay'),
            actions=[
                Node(
                    package='agri_vslam_bringup',
                    executable='imu_bias_corrector.py',
                    name='orbbec_imu_bias_corrector',
                    output='screen',
                    arguments=[
                        '--profile', LaunchConfiguration('calibration_profile'),
                        '--input-topic', LaunchConfiguration('raw_imu_topic'),
                        '--output-topic', LaunchConfiguration('calibrated_imu_topic'),
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agri_vslam_bringup'),
                            'launch',
                            'orbbec_imu_filter.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'raw_imu_topic': LaunchConfiguration('calibrated_imu_topic'),
                        'filtered_imu_topic': LaunchConfiguration('filtered_imu_topic'),
                    }.items(),
                ),
            ],
        ),
    ])
