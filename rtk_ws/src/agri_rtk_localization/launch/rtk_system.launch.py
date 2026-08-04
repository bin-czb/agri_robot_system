from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('um982_ntrip'),
                'launch',
                'um982_ntrip.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('agri_rtk_localization'),
                'launch',
                'rtk_global_localization.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'output_frame': LaunchConfiguration('output_frame'),
            'publish_selected': LaunchConfiguration('publish_selected'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_localizer')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_driver',
            default_value='true',
            description='启动 UM982 串口和 NTRIP 驱动。',
        ),
        DeclareLaunchArgument(
            'start_localizer',
            default_value='true',
            description='启动 RTK 地图坐标转换与轨迹发布。',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='打开 RTK 专用 RViz。联合系统中通常关闭。',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='回放 rosbag 时使用仿真时间。',
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='map',
            description='RTK 定位输出坐标系。',
        ),
        DeclareLaunchArgument(
            'publish_selected',
            default_value='true',
            description='将 RTK 发布为当前全局定位源。',
        ),
        driver_launch,
        localization_launch,
    ])
