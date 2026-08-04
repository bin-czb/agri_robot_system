from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_meshes = LaunchConfiguration('use_meshes')
    frame_prefix = LaunchConfiguration('frame_prefix')
    publish_joint_states = LaunchConfiguration('publish_joint_states')

    model_file = PathJoinSubstitution([
        FindPackageShare('agri_robot_description'),
        'urdf',
        'agri_robot.urdf.xacro',
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            model_file,
            ' use_meshes:=',
            use_meshes,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'use_meshes',
            default_value='true',
            description='Use the visual meshes from trunk_gazebo_worlds.',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='Optional TF frame prefix, for example bb_robot/ to match Gazebo odometry frames.',
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='true',
            description='Start joint_state_publisher for the wheel joints.',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(publish_joint_states),
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                robot_description,
                {
                    'use_sim_time': use_sim_time,
                    'frame_prefix': frame_prefix,
                },
            ],
            output='screen',
        ),
    ])
