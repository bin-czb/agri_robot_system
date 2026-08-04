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
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
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
                'enable_sync_output_accel_gyro': 'true',
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'accel_rate': '200hz',
                'gyro_rate': '200hz',
                'accel_range': '4g',
                'gyro_range': '1000dps',
                'accel_qos': 'SENSOR_DATA',
                'gyro_qos': 'SENSOR_DATA',
                'publish_synced_imu_tf': 'true',
            }.items(),
        ),
    ])
