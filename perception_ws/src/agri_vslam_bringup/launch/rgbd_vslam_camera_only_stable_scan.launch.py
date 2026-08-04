from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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
            description='Accept the normal Gemini RGB/depth offset while rejecting worse pairings.',
        ),
        DeclareLaunchArgument(
            'odom_args',
            default_value='--Vis/MinInliers 12 --Vis/MaxFeatures 1500 --OdomF2M/MaxSize 1500 --Odom/ResetCountdown 1 --Odom/GuessMotion true',
            description='Moderately tolerant visual odometry arguments for camera-only SLAM validation.',
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='--delete_db_on_start --Reg/Force3DoF true --Grid/Sensor 2 --Grid/RayTracing true --Grid/3D false --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.05 --Grid/RangeMin 0.35 --Grid/RangeMax 5.0 --Grid/MaxObstacleHeight 2.0 --Optimizer/GravitySigma 0',
            description='RTAB-Map arguments for camera-only RGB-D plus fake scan mapping.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Output OccupancyGrid topic for RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'map_frame_id',
            default_value='map',
            description='RTAB-Map global frame.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                '.ros',
                'rtabmap_agri_rgbd_vo_test.db',
            ]),
            description='RTAB-Map database used by the camera-only mapping session.',
        ),
        DeclareLaunchArgument(
            'start_scan',
            default_value='true',
            description='Start depthimage_to_laserscan for a temporary Nav2-style /scan.',
        ),
        DeclareLaunchArgument(
            'scan_delay',
            default_value='4.0',
            description='Delay fake scan startup until camera depth topics are available.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic.',
        ),
        DeclareLaunchArgument(
            'scan_output_frame',
            default_value='camera_color_optical_frame',
            description='LaserScan frame for registered Orbbec depth.',
        ),
        DeclareLaunchArgument(
            'start_imu_filter',
            default_value='true',
            description='Publish a diagnostic gravity-aligned Orbbec IMU orientation.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/imu/data',
            description='Filtered Orbbec IMU topic.',
        ),
        DeclareLaunchArgument(
            'rtabmap_imu_topic',
            default_value='/imu/data_disabled',
            description='Optional RTAB-Map IMU input; disabled by default for planar camera-only SLAM.',
        ),
        DeclareLaunchArgument(
            'wait_imu_to_init',
            default_value='false',
            description='Wait for rtabmap_imu_topic before visual odometry starts.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'rgbd_vslam_camera_only_stable.launch.py',
                ])
            ),
            launch_arguments={
                'start_rtabmap': LaunchConfiguration('start_rtabmap'),
                'rtabmap_delay': LaunchConfiguration('rtabmap_delay'),
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                'odom_args': LaunchConfiguration('odom_args'),
                'rtabmap_args': LaunchConfiguration('rtabmap_args'),
                'map_topic': LaunchConfiguration('map_topic'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'database_path': LaunchConfiguration('database_path'),
                'subscribe_scan': LaunchConfiguration('start_scan'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'start_imu_filter': LaunchConfiguration('start_imu_filter'),
                'filtered_imu_topic': LaunchConfiguration('filtered_imu_topic'),
                'rtabmap_imu_topic': LaunchConfiguration('rtabmap_imu_topic'),
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
            }.items(),
        ),
        TimerAction(
            period=LaunchConfiguration('scan_delay'),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('agri_vslam_bringup'),
                            'launch',
                            'fake_scan_from_depth.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'output_frame': LaunchConfiguration('scan_output_frame'),
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('start_scan')),
                ),
            ],
        ),
    ])
