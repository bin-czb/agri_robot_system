from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('agri_vslam_bringup'),
                    'launch',
                    'orbbec_gemini.launch.py',
                ])
            ),
            launch_arguments={
                'depth_registration': 'true',
                'enable_point_cloud': 'true',
                'enable_colored_point_cloud': 'false',
                'point_cloud_qos': 'SENSOR_DATA',
                'device_preset': 'High Density',
                'ordered_pc': 'false',
                'color_width': '640',
                'color_height': '480',
                'color_fps': '15',
                'depth_width': '640',
                'depth_height': '480',
                'depth_fps': '15',
                'enable_left_ir': 'false',
                'enable_right_ir': 'false',
                'enable_soft_filter': 'true',
                'enable_noise_removal_filter': 'true',
                'enable_decimation_filter': 'false',
                'enable_spatial_filter': 'true',
                'enable_temporal_filter': 'true',
                'enable_hole_filling_filter': 'true',
                'enable_threshold_filter': 'true',
                'threshold_filter_min': '350',
                'threshold_filter_max': '5000',
            }.items(),
        ),
    ])
