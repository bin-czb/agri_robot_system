from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orbbec_launch_file = LaunchConfiguration('orbbec_launch_file')
    camera_name = LaunchConfiguration('camera_name')
    robot_camera_frame = LaunchConfiguration('robot_camera_frame')
    orbbec_camera_link_frame = LaunchConfiguration('orbbec_camera_link_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'orbbec_launch_file',
            default_value='gemini_330_series.launch.py',
            description='Orbbec launch file. Gemini 335L uses the Gemini 330 series launch.',
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Orbbec namespace and camera name.',
        ),
        DeclareLaunchArgument(
            'depth_registration',
            default_value='true',
            description='Align depth to color for RGB-D SLAM.',
        ),
        DeclareLaunchArgument(
            'enable_point_cloud',
            default_value='true',
            description='Enable Orbbec point cloud output for later debugging or mapping.',
        ),
        DeclareLaunchArgument(
            'enable_colored_point_cloud',
            default_value='true',
            description='Enable registered color point cloud output.',
        ),
        DeclareLaunchArgument(
            'point_cloud_qos',
            default_value='default',
            description='QoS profile for Orbbec point cloud topics.',
        ),
        DeclareLaunchArgument(
            'cloud_frame_id',
            default_value='',
            description='Optional frame id override for Orbbec point cloud output.',
        ),
        DeclareLaunchArgument(
            'ordered_pc',
            default_value='false',
            description='Publish organized point clouds when true. This is heavier than the default compact cloud.',
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value='0',
            description='Color stream width. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='0',
            description='Color stream height. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'color_fps',
            default_value='0',
            description='Color stream FPS. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value='0',
            description='Depth stream width. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value='0',
            description='Depth stream height. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value='0',
            description='Depth stream FPS. Use 0 for Orbbec SDK default.',
        ),
        DeclareLaunchArgument(
            'enable_left_ir',
            default_value='true',
            description='Enable left IR stream.',
        ),
        DeclareLaunchArgument(
            'enable_right_ir',
            default_value='true',
            description='Enable right IR stream.',
        ),
        DeclareLaunchArgument(
            'enable_soft_filter',
            default_value='true',
            description='Enable Orbbec SDK soft depth filter.',
        ),
        DeclareLaunchArgument(
            'enable_noise_removal_filter',
            default_value='true',
            description='Enable Orbbec SDK noise removal filter.',
        ),
        DeclareLaunchArgument(
            'enable_decimation_filter',
            default_value='false',
            description='Enable Orbbec SDK decimation filter. Keep false when dense contours are needed.',
        ),
        DeclareLaunchArgument(
            'enable_spatial_filter',
            default_value='false',
            description='Enable Orbbec SDK spatial depth filter.',
        ),
        DeclareLaunchArgument(
            'enable_temporal_filter',
            default_value='false',
            description='Enable Orbbec SDK temporal depth filter.',
        ),
        DeclareLaunchArgument(
            'enable_hole_filling_filter',
            default_value='false',
            description='Enable Orbbec SDK hole filling filter.',
        ),
        DeclareLaunchArgument(
            'enable_threshold_filter',
            default_value='false',
            description='Enable Orbbec SDK depth range threshold filter.',
        ),
        DeclareLaunchArgument(
            'threshold_filter_min',
            default_value='-1',
            description='Minimum depth kept by the threshold filter, in millimeters when enabled.',
        ),
        DeclareLaunchArgument(
            'threshold_filter_max',
            default_value='-1',
            description='Maximum depth kept by the threshold filter, in millimeters when enabled.',
        ),
        DeclareLaunchArgument(
            'enable_3d_reconstruction_mode',
            default_value='false',
            description='Enable Orbbec 3D reconstruction mode when supported by the device.',
        ),
        DeclareLaunchArgument(
            'device_preset',
            default_value='Default',
            description='G330 depth preset, for example Default, High Accuracy, High Density, or Medium Density.',
        ),
        DeclareLaunchArgument(
            'enable_sync_output_accel_gyro',
            default_value='false',
            description='Publish synchronized accel+gyro as /camera/gyro_accel/sample.',
        ),
        DeclareLaunchArgument(
            'enable_accel',
            default_value='false',
            description='Enable Orbbec accelerometer stream.',
        ),
        DeclareLaunchArgument(
            'accel_rate',
            default_value='200hz',
            description='Accelerometer sample rate. 200hz is the Gemini 330 series stable default.',
        ),
        DeclareLaunchArgument(
            'accel_range',
            default_value='4g',
            description='Accelerometer full-scale range.',
        ),
        DeclareLaunchArgument(
            'accel_qos',
            default_value='SENSOR_DATA',
            description='QoS profile for accelerometer/combined IMU output.',
        ),
        DeclareLaunchArgument(
            'enable_gyro',
            default_value='false',
            description='Enable Orbbec gyroscope stream.',
        ),
        DeclareLaunchArgument(
            'gyro_rate',
            default_value='200hz',
            description='Gyroscope sample rate. 200hz is the Gemini 330 series stable default.',
        ),
        DeclareLaunchArgument(
            'gyro_range',
            default_value='1000dps',
            description='Gyroscope full-scale range.',
        ),
        DeclareLaunchArgument(
            'gyro_qos',
            default_value='SENSOR_DATA',
            description='QoS profile for gyroscope/combined IMU output.',
        ),
        DeclareLaunchArgument(
            'liner_accel_cov',
            default_value='0.01',
            description='Linear acceleration covariance passed to the Orbbec IMU message.',
        ),
        DeclareLaunchArgument(
            'angular_vel_cov',
            default_value='0.01',
            description='Angular velocity covariance passed to the Orbbec IMU message.',
        ),
        DeclareLaunchArgument(
            'publish_synced_imu_tf',
            default_value='false',
            description='Publish camera_link -> camera_accel_gyro_optical_frame for synchronized IMU messages.',
        ),
        DeclareLaunchArgument(
            'orbbec_synced_imu_frame',
            default_value='camera_accel_gyro_optical_frame',
            description='Frame id used by /camera/gyro_accel/sample when camera_name is camera.',
        ),
        DeclareLaunchArgument(
            'align_mode',
            default_value='SW',
            description='Depth-to-color alignment mode. SW is safest; HW can be tested if supported.',
        ),
        DeclareLaunchArgument(
            'publish_orbbec_tf',
            default_value='true',
            description='Let Orbbec publish its internal camera_link to optical-frame TFs.',
        ),
        DeclareLaunchArgument(
            'publish_mount_tf',
            default_value='true',
            description='Publish a static TF from the robot camera mount to Orbbec camera_link.',
        ),
        DeclareLaunchArgument(
            'robot_camera_frame',
            default_value='bb_robot/camera_link',
            description='Camera mount frame from agri_robot_description.',
        ),
        DeclareLaunchArgument(
            'orbbec_camera_link_frame',
            default_value='camera_link',
            description='Root camera frame used by the Orbbec wrapper.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('orbbec_camera'),
                    'launch',
                    orbbec_launch_file,
                ])
            ),
            launch_arguments={
                'camera_name': camera_name,
                'depth_registration': LaunchConfiguration('depth_registration'),
                'enable_point_cloud': LaunchConfiguration('enable_point_cloud'),
                'enable_colored_point_cloud': LaunchConfiguration('enable_colored_point_cloud'),
                'point_cloud_qos': LaunchConfiguration('point_cloud_qos'),
                'cloud_frame_id': LaunchConfiguration('cloud_frame_id'),
                'ordered_pc': LaunchConfiguration('ordered_pc'),
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'color_fps': LaunchConfiguration('color_fps'),
                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'depth_fps': LaunchConfiguration('depth_fps'),
                'enable_left_ir': LaunchConfiguration('enable_left_ir'),
                'enable_right_ir': LaunchConfiguration('enable_right_ir'),
                'enable_soft_filter': LaunchConfiguration('enable_soft_filter'),
                'enable_noise_removal_filter': LaunchConfiguration('enable_noise_removal_filter'),
                'enable_decimation_filter': LaunchConfiguration('enable_decimation_filter'),
                'enable_spatial_filter': LaunchConfiguration('enable_spatial_filter'),
                'enable_temporal_filter': LaunchConfiguration('enable_temporal_filter'),
                'enable_hole_filling_filter': LaunchConfiguration('enable_hole_filling_filter'),
                'enable_threshold_filter': LaunchConfiguration('enable_threshold_filter'),
                'threshold_filter_min': LaunchConfiguration('threshold_filter_min'),
                'threshold_filter_max': LaunchConfiguration('threshold_filter_max'),
                'enable_3d_reconstruction_mode': LaunchConfiguration('enable_3d_reconstruction_mode'),
                'device_preset': LaunchConfiguration('device_preset'),
                'enable_sync_output_accel_gyro': LaunchConfiguration('enable_sync_output_accel_gyro'),
                'enable_accel': LaunchConfiguration('enable_accel'),
                'accel_rate': LaunchConfiguration('accel_rate'),
                'accel_range': LaunchConfiguration('accel_range'),
                'accel_qos': LaunchConfiguration('accel_qos'),
                'enable_gyro': LaunchConfiguration('enable_gyro'),
                'gyro_rate': LaunchConfiguration('gyro_rate'),
                'gyro_range': LaunchConfiguration('gyro_range'),
                'gyro_qos': LaunchConfiguration('gyro_qos'),
                'liner_accel_cov': LaunchConfiguration('liner_accel_cov'),
                'angular_vel_cov': LaunchConfiguration('angular_vel_cov'),
                'align_mode': LaunchConfiguration('align_mode'),
                'publish_tf': LaunchConfiguration('publish_orbbec_tf'),
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orbbec_synced_imu_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.57079632679',
                '--pitch', '0',
                '--yaw', '-1.57079632679',
                '--frame-id', orbbec_camera_link_frame,
                '--child-frame-id', LaunchConfiguration('orbbec_synced_imu_frame'),
            ],
            condition=IfCondition(LaunchConfiguration('publish_synced_imu_tf')),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='orbbec_mount_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                robot_camera_frame,
                orbbec_camera_link_frame,
            ],
            condition=IfCondition(LaunchConfiguration('publish_mount_tf')),
            output='screen',
        ),
    ])
