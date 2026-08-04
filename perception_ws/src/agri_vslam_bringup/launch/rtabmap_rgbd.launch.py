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
            'odom_frame_id',
            default_value='',
            description='Leave empty to make RTAB-Map use odom_topic. The odom topic header.frame_id should be bb_robot/odom.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odometry/filtered',
            description='Filtered odometry from robot_localization.',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='Orbbec RGB image topic.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Orbbec depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Orbbec RGB camera info topic.',
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
            default_value='/imu/data_disabled',
            description='Optional filtered IMU topic for 6DoF gravity alignment.',
        ),
        DeclareLaunchArgument(
            'wait_imu_to_init',
            default_value='false',
            description='Wait for a valid filtered IMU orientation before processing RGB-D data.',
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
            'database_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                '.ros',
                'rtabmap_agri_rgbd.db',
            ]),
            description='RTAB-Map database path.',
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description='Extra RTAB-Map arguments, for example --delete_db_on_start.',
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
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
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
                'visual_odometry': 'false',
            }.items(),
        ),
    ])
