from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('agri_aoa_driver'),
        'config',
        'aoa_driver.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_uav_gps',
            default_value='true',
            description='启动无人机 MAVLink UTM 位置源。',
        ),
        DeclareLaunchArgument(
            'start_aoa_source',
            default_value='true',
            description='启动地面 AOA 串口定位节点。',
        ),
        DeclareLaunchArgument(
            'mavlink_port',
            default_value='/dev/ttyUSB0',
            description='飞行器 MAVLink 串口。',
        ),
        DeclareLaunchArgument(
            'aoa_serial_port',
            default_value='/dev/ttyACM0',
            description='AOA 基站串口。',
        ),
        DeclareLaunchArgument(
            'heading_mode',
            default_value='imu_relative',
            description='地面刚性传感器组件的航向输入模式。',
        ),
        DeclareLaunchArgument(
            'initial_heading_enu_deg',
            default_value='0.0',
            description='启动时组件的 ENU 绝对航向，东为 0 度。',
        ),
        DeclareLaunchArgument(
            'base_imu_topic',
            default_value='/camera/imu/data',
            description='与 AOA 刚性连接的滤波后 IMU 话题。',
        ),
        DeclareLaunchArgument(
            'base_odometry_topic',
            default_value='/odometry/filtered',
            description='后续底盘接入时使用的连续局部里程计。',
        ),
        Node(
            package='agri_aoa_driver',
            executable='uav_gps_node',
            name='uav_gps_node',
            output='screen',
            parameters=[
                config,
                {'port': LaunchConfiguration('mavlink_port')},
            ],
            condition=IfCondition(LaunchConfiguration('start_uav_gps')),
        ),
        Node(
            package='agri_aoa_driver',
            executable='aoa_localization_node',
            name='aoa_localization_node',
            output='screen',
            parameters=[
                config,
                {
                    'serial_port': LaunchConfiguration('aoa_serial_port'),
                    'base_heading_mode': LaunchConfiguration('heading_mode'),
                    'base_initial_heading_enu_deg': LaunchConfiguration(
                        'initial_heading_enu_deg'),
                    'base_imu_topic': LaunchConfiguration('base_imu_topic'),
                    'base_odometry_topic': LaunchConfiguration(
                        'base_odometry_topic'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('start_aoa_source')),
        ),
    ])
