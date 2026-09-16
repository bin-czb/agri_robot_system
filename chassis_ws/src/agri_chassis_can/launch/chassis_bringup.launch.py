from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    listen_only = LaunchConfiguration("listen_only")
    start_joy = LaunchConfiguration("start_joy")
    joy_device = LaunchConfiguration("joy_device")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat_rate = LaunchConfiguration("joy_autorepeat_rate")

    default_params = PathJoinSubstitution([
        FindPackageShare("agri_chassis_can"), "config", "td48150b.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "listen_only",
            default_value="true",
            description=(
                "Receive CAN but do not transmit. Keep true until the real TD48150B "
                "IDs, A/B mapping, signs and feedback units are verified."
            ),
        ),
        DeclareLaunchArgument(
            "start_joy",
            default_value="false",
            description=(
                "Start ROS joy_node. Keep false for CAN-only bring-up; set true for "
                "Taizhou-style A-button AUTO/MANUAL testing."
            ),
        ),
        DeclareLaunchArgument(
            "joy_device",
            default_value="/dev/input/js0",
            description="Linux joystick device used by joy_node.",
        ),
        DeclareLaunchArgument(
            "joy_deadzone",
            default_value="0.05",
            description="joy_node axis deadzone.",
        ),
        DeclareLaunchArgument(
            "joy_autorepeat_rate",
            default_value="20.0",
            description=(
                "joy_node repeat rate. A nonzero repeat rate also lets the manual "
                "source watchdog detect joystick disconnects reliably."
            ),
        ),

        # The selector is always present, including AUTO-only operation.  That
        # guarantees that Nav2 and a joystick can never become two simultaneous
        # publishers directly feeding the CAN driver.
        Node(
            package="agri_chassis_can",
            executable="chassis_mode_teleop",
            name="chassis_mode_teleop",
            output="screen",
            parameters=[params_file],
        ),

        # joy_node only converts the Linux joystick device into sensor_msgs/Joy
        # on /joy.  It does NOT publish Twist and does NOT select AUTO/MANUAL.
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            condition=IfCondition(start_joy),
            parameters=[{
                "dev": joy_device,
                "deadzone": ParameterValue(joy_deadzone, value_type=float),
                "autorepeat_rate": ParameterValue(
                    joy_autorepeat_rate, value_type=float
                ),
            }],
        ),

        # The CAN driver has one and only one velocity command input:
        # /chassis/cmd_vel, published by chassis_mode_teleop.
        Node(
            package="agri_chassis_can",
            executable="chassis_can_node",
            name="agri_chassis_can",
            output="screen",
            parameters=[
                params_file,
                {
                    "listen_only": ParameterValue(
                        listen_only,
                        value_type=bool,
                    )
                },
            ],
        ),
    ])
