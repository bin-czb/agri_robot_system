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
