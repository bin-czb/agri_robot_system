from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use wall time for the real RGB-D camera path.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='bb_robot/base_link',
            description='Robot base frame used by RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'vo_frame_id',
            default_value='bb_robot/odom',
            description='Temporary odom frame published by RTAB-Map visual odometry.',
        ),
        DeclareLaunchArgument(
            'map_frame_id',
            default_value='map',
            description='RTAB-Map global frame. Use vslam_map with a georeferenced prior map.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/rtabmap/odom',
            description='Temporary odometry topic published by RTAB-Map visual odometry.',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='Orbbec RGB image topic.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Orbbec registered depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Orbbec RGB camera info topic matching registered depth.',
        ),
        DeclareLaunchArgument(
            'approx_sync',
            default_value='true',
            description='Use approximate sync for RGB and depth.',
        ),
        DeclareLaunchArgument(
            'approx_sync_max_interval',
            default_value='0.04',
            description='Reject RGB/depth pairs farther apart than this interval.',
        ),
        DeclareLaunchArgument(
            'wait_for_transform',
            default_value='0.5',
            description='TF wait timeout for RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Filtered IMU topic used by visual odometry and RTAB-Map.',
        ),
        DeclareLaunchArgument(
            'wait_imu_to_init',
            default_value='false',
            description='Wait for a valid IMU orientation before visual odometry starts.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Start RTAB-Map visualization.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz from rtabmap_launch.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Output OccupancyGrid topic for RTAB-Map.',
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
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                '.ros',
                'rtabmap_agri_rgbd_vo_test.db',
            ]),
            description='RTAB-Map test database path.',
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='--delete_db_on_start --Reg/Force3DoF true --Grid/Sensor 1 --Grid/RayTracing true --Grid/3D false --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.05 --Grid/RangeMin 0.35 --Grid/RangeMax 5.0 --Grid/MaxObstacleHeight 2.0 --Optimizer/GravitySigma 0',
            description='Extra RTAB-Map arguments, including a 2D occupancy grid from RGB-D depth.',
        ),
        DeclareLaunchArgument(
            'odom_args',
            default_value='',
            description='Extra RTAB-Map visual odometry arguments for camera-only tests.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': LaunchConfiguration('frame_id'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'map_topic': LaunchConfiguration('map_topic'),
                'odom_frame_id': '',
                'vo_frame_id': LaunchConfiguration('vo_frame_id'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'subscribe_scan': LaunchConfiguration('subscribe_scan'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'approx_sync': LaunchConfiguration('approx_sync'),
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                'wait_for_transform': LaunchConfiguration('wait_for_transform'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'log_level': 'info',
                'database_path': LaunchConfiguration('database_path'),
                'rtabmap_args': LaunchConfiguration('rtabmap_args'),
                'odom_args': LaunchConfiguration('odom_args'),
                'visual_odometry': 'true',
                'publish_tf_odom': 'true',
            }.items(),
        ),
    ])
