from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    listen_only = LaunchConfiguration("listen_only")
    start_joy = LaunchConfiguration("start_joy")
    joy_device = LaunchConfiguration("joy_device")

    default_params = PathJoinSubstitution([
        FindPackageShare("agri_chassis_can"), "config", "td48150b.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "listen_only",
            default_value="true",
            description="Keep true for first hardware inspection.",
        ),
        DeclareLaunchArgument("start_joy", default_value="false"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        Node(
            package="agri_chassis_can",
            executable="chassis_can_node",
            name="agri_chassis_can",
            output="screen",
            parameters=[params_file, {"listen_only": listen_only}],
        ),
        Node(
            package="agri_chassis_can",
            executable="chassis_mode_teleop",
            name="chassis_mode_teleop",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=IfCondition(start_joy),
            parameters=[{
                "dev": joy_device,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }],
        ),
    ])
