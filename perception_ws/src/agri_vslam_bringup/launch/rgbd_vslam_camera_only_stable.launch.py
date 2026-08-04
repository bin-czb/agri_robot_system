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
            default_value='--delete_db_on_start --Reg/Force3DoF true --Grid/Sensor 1 --Grid/RayTracing true --Grid/3D false --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.05 --Grid/RangeMin 0.35 --Grid/RangeMax 5.0 --Grid/MaxObstacleHeight 2.0 --Optimizer/GravitySigma 0',
            description='RTAB-Map mapping arguments for camera-only validation.',
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
            'subscribe_scan',
            default_value='false',
            description='Let RTAB-Map also use a LaserScan for 2D grid generation.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic used when subscribe_scan is true.',
        ),
        DeclareLaunchArgument(
            'start_imu_filter',
            default_value='true',
            description='Publish a diagnostic gravity-aligned Orbbec IMU orientation.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/imu/data',
            description='Madgwick-filtered Orbbec IMU output topic.',
        ),
        DeclareLaunchArgument(
            'rtabmap_imu_topic',
            default_value='/imu/data_disabled',
            description='Optional IMU input for RTAB-Map; disabled by default in planar camera-only mode.',
        ),
        DeclareLaunchArgument(
            'wait_imu_to_init',
            default_value='false',
            description='Wait for rtabmap_imu_topic before visual odometry starts.',
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
                    'orbbec_gemini_slam.launch.py',
                ])
            ),
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
                'filtered_imu_topic': LaunchConfiguration('filtered_imu_topic'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_imu_filter')),
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
                        'rtabmap_args': LaunchConfiguration('rtabmap_args'),
                        'map_topic': LaunchConfiguration('map_topic'),
                        'map_frame_id': LaunchConfiguration('map_frame_id'),
                        'database_path': LaunchConfiguration('database_path'),
                        'subscribe_scan': LaunchConfiguration('subscribe_scan'),
                        'scan_topic': LaunchConfiguration('scan_topic'),
                        'imu_topic': LaunchConfiguration('rtabmap_imu_topic'),
                        'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                        'rtabmap_viz': 'false',
                        'rviz': 'false',
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('start_rtabmap')),
                ),
            ],
        ),
    ])
