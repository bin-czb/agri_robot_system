from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Registered Orbbec depth image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera info topic matching the registered depth image.',
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Output LaserScan topic for Nav2 local costmap.',
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='camera_color_optical_frame',
            description='LaserScan frame. With depth_registration=true, color optical frame is expected.',
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'scan_height': 10,
                'range_min': 0.35,
                'range_max': 8.0,
                'output_frame': LaunchConfiguration('output_frame'),
            }],
            remappings=[
                ('depth', LaunchConfiguration('depth_topic')),
                ('depth_camera_info', LaunchConfiguration('camera_info_topic')),
                ('scan', LaunchConfiguration('scan_topic')),
            ],
        ),
    ])
