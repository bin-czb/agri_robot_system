from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_description',
            default_value='true',
            description='Publish bb_robot/base_link to camera frames from the robot description.',
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='true',
            description='Start the Orbbec Gemini camera wrapper.',
        ),
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
            condition=IfCondition(LaunchConfiguration('start_description')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'orbbec_gemini.launch.py',
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
                            'rtabmap_rgbd_vo_test.launch.py',
                        ])
                    ),
                    condition=IfCondition(LaunchConfiguration('start_rtabmap')),
                ),
            ],
        ),
    ])
