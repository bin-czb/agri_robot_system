from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rtabmap',
            default_value='true',
            description='Start RTAB-Map RGB-D SLAM with temporary visual odometry.',
        ),
        DeclareLaunchArgument(
            'rtabmap_delay',
            default_value='5.0',
            description='Delay RTAB-Map startup so camera topics and TF can appear first.',
        ),
        DeclareLaunchArgument(
            'approx_sync_max_interval',
            default_value='0.06',
            description='Reject RGB/depth pairs farther apart than this interval.',
        ),
        DeclareLaunchArgument(
            'odom_args',
            default_value='--Vis/MinInliers 8 --Vis/MaxFeatures 1200 --OdomF2M/MaxSize 1200 --Odom/ResetCountdown 1 --Odom/GuessMotion true',
            description='More tolerant visual odometry arguments for low-texture camera-only tests.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_robot_description'),
                    'launch',
                    'description.launch.py',
                ])
            ),
            launch_arguments={
                'frame_prefix': 'bb_robot/',
                'use_sim_time': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'orbbec_gemini_fast.launch.py',
                ])
            ),
        ),
        TimerAction(
            period=LaunchConfiguration('rtabmap_delay'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agri_vslam_bringup'),
                            'launch',
                            'rtabmap_rgbd_vo_test.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                        'odom_args': LaunchConfiguration('odom_args'),
                        'rtabmap_viz': 'false',
                        'rviz': 'false',
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('start_rtabmap')),
                ),
            ],
        ),
    ])
