from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.05',
            description='联调前后运动速度，单位 m/s。',
        ),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.2',
            description='联调转向角速度，单位 rad/s。',
        ),
        DeclareLaunchArgument(
            'command_duration',
            default_value='1.0',
            description='每次服务指令的最长持续时间。',
        ),
        Node(
            package='chassis_control',
            executable='chassis_serial_teleop',
            name='chassis_serial_teleop',
            output='screen',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'command_duration': LaunchConfiguration('command_duration'),
            }],
        ),
    ])
