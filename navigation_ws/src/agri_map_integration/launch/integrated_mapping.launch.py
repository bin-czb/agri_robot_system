from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_vslam = LaunchConfiguration('start_vslam')
    start_calibrated_imu = LaunchConfiguration('start_calibrated_imu')
    start_rtk_localizer = LaunchConfiguration('start_rtk_localizer')
    start_rviz = LaunchConfiguration('start_rviz')

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agri_map_integration'),
                'launch',
                'map_visualization.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_map_server': 'true',
            'start_rtk_localizer': start_rtk_localizer,
            'start_vslam_aligner': start_vslam,
            'start_rviz': start_rviz,
        }.items(),
    )

    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agri_vslam_bringup'),
                'launch',
                'rgbd_vslam_camera_only_stable_scan.launch.py',
            ])
        ),
        launch_arguments={
            'map_frame_id': 'vslam_map',
            'map_topic': '/rtabmap/grid_map',
            'start_rtabmap': 'true',
            'start_scan': 'true',
            'start_imu_filter': 'false',
        }.items(),
        condition=IfCondition(start_vslam),
    )

    calibrated_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agri_vslam_bringup'),
                'launch',
                'orbbec_calibrated_imu.launch.py',
            ])
        ),
        launch_arguments={
            'start_camera': 'false',
        }.items(),
        condition=IfCondition(start_calibrated_imu),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use wall time with the live camera and RTK.',
        ),
        DeclareLaunchArgument(
            'start_vslam',
            default_value='true',
            description='Start the Gemini camera-only RTAB-Map test chain.',
        ),
        DeclareLaunchArgument(
            'start_calibrated_imu',
            default_value='true',
            description='Apply the accepted IMU profile and publish /camera/imu/data.',
        ),
        DeclareLaunchArgument(
            'start_rtk_localizer',
            default_value='true',
            description='Start map-frame RTK localization; the UM982 driver is separate.',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Open the integrated RViz view.',
        ),
        map_launch,
        vslam_launch,
        calibrated_imu_launch,
    ])
