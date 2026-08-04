from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    start_description = LaunchConfiguration('start_description')
    bridge_gazebo_odom = LaunchConfiguration('bridge_gazebo_odom')
    bridge_gazebo_clock = LaunchConfiguration('bridge_gazebo_clock')
    frame_prefix = LaunchConfiguration('frame_prefix')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('agri_robot_bringup'),
        'config',
        'ekf_sim.yaml',
    ])

    description_launch = PathJoinSubstitution([
        FindPackageShare('agri_robot_description'),
        'launch',
        'description.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='robot_localization EKF parameter file.',
        ),
        DeclareLaunchArgument(
            'start_description',
            default_value='true',
            description='Start robot_state_publisher and joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='bb_robot/',
            description='Frame prefix for the robot description. Use empty string for standard odom/base_link.',
        ),
        DeclareLaunchArgument(
            'bridge_gazebo_odom',
            default_value='true',
            description='Bridge Gazebo odometry into ROS 2. Do not bridge Gazebo TF when EKF publish_tf is true.',
        ),
        DeclareLaunchArgument(
            'bridge_gazebo_clock',
            default_value='true',
            description='Bridge Gazebo /clock into ROS 2 when use_sim_time is true.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            condition=IfCondition(start_description),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'frame_prefix': frame_prefix,
            }.items(),
        ),
        ExecuteProcess(
            condition=IfCondition(bridge_gazebo_odom),
            cmd=[
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                '/model/bb_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
            output='screen',
        ),
        ExecuteProcess(
            condition=IfCondition(bridge_gazebo_clock),
            cmd=[
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
