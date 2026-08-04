from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the Orbbec Gemini camera wrapper.',
        ),
        DeclareLaunchArgument(
            'start_rtabmap',
            default_value='true',
            description='Start production RTAB-Map RGB-D SLAM using external /odometry/filtered.',
        ),
        DeclareLaunchArgument(
            'rtabmap_delay',
            default_value='3.0',
            description='Delay RTAB-Map startup so camera and EKF topics can appear first.',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'orbbec_imu_filter.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        TimerAction(
            period=LaunchConfiguration('rtabmap_delay'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agri_vslam_bringup'),
                            'launch',
                            'rtabmap_rgbd.launch.py',
                        ])
                    ),
                    condition=IfCondition(LaunchConfiguration('start_rtabmap')),
                ),
            ],
        ),
    ])
