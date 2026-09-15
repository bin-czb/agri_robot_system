from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    listen_only = LaunchConfiguration("listen_only")

    default_params = PathJoinSubstitution([
        FindPackageShare("agri_chassis_can"), "config", "td48150b.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument(
            "listen_only",
            default_value="true",
            description=(
                "Keep true during first hardware inspection. This launch only starts "
                "the TD48150B CAN driver; verified Taizhou joystick control will be "
                "ported separately once its exact source file is available."
            ),
        ),
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
