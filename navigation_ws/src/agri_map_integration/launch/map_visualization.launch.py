import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('agri_map_integration')
    rtk_share = get_package_share_directory('agri_rtk_localization')

    map_yaml = os.path.join(
        package_share, 'config', 'navigation_map.yaml')
    metadata_yaml = os.path.join(
        package_share, 'config', 'map_georeference.yaml')
    rtk_base_yaml = os.path.join(
        rtk_share, 'config', 'rtk_localization.yaml')
    rtk_map_yaml = os.path.join(
        package_share, 'config', 'rtk_uav_test_map.yaml')
    orthophoto = os.path.join(
        package_share, 'maps', 'orthophoto.tif')
    tree_csv = os.path.join(
        package_share, 'maps', 'tree_landmarks.csv')
    rviz_config = os.path.join(
        package_share, 'rviz', 'geospatial_mapping.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_map_server = LaunchConfiguration('start_map_server')
    start_rtk_localizer = LaunchConfiguration('start_rtk_localizer')
    start_vslam_aligner = LaunchConfiguration('start_vslam_aligner')
    start_vslam_grid_overlay = LaunchConfiguration(
        'start_vslam_grid_overlay')
    start_rviz = LaunchConfiguration('start_rviz')
    start_orthophoto = LaunchConfiguration('start_orthophoto')
    orthophoto_pixel_stride = LaunchConfiguration(
        'orthophoto_pixel_stride')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for rosbag or Gazebo tests.',
        ),
        DeclareLaunchArgument(
            'start_map_server',
            default_value='true',
            description='Publish the tested navigation occupancy grid on /map.',
        ),
        DeclareLaunchArgument(
            'start_rtk_localizer',
            default_value='true',
            description='Convert /fix into map-frame RTK pose and trajectory.',
        ),
        DeclareLaunchArgument(
            'start_vslam_aligner',
            default_value='true',
            description='Wait for RViz 2D Pose Estimate to align vslam_map.',
        ),
        DeclareLaunchArgument(
            'start_vslam_grid_overlay',
            default_value='true',
            description=(
                'Show occupied RTAB-Map cells without the gray unknown area.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Open the integrated geospatial RViz view.',
        ),
        DeclareLaunchArgument(
            'start_orthophoto',
            default_value='false',
            description='Optionally publish the color orthophoto for map QA.',
        ),
        DeclareLaunchArgument(
            'orthophoto_pixel_stride',
            default_value='2',
            description='Publish every Nth orthophoto pixel; 2 balances detail and RViz load.',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'yaml_filename': map_yaml,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
            condition=IfCondition(start_map_server),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_geospatial_map',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'autostart': True,
                'node_names': ['map_server'],
            }],
            condition=IfCondition(start_map_server),
        ),
        Node(
            package='agri_map_integration',
            executable='orthophoto_publisher',
            name='orthophoto_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'image_path': orthophoto,
                'metadata_path': metadata_yaml,
                'topic': '/orthophoto/cloud',
                'frame_id': 'map',
                'z_offset': -0.05,
                'pixel_stride': ParameterValue(
                    orthophoto_pixel_stride, value_type=int),
            }],
            condition=IfCondition(start_orthophoto),
        ),
        Node(
            package='agri_map_integration',
            executable='tree_marker_publisher',
            name='tree_marker_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'csv_path': tree_csv,
                'topic': '/tree_markers',
                'frame_id': 'map',
            }],
        ),
        Node(
            package='agri_rtk_localization',
            executable='rtk_localizer',
            name='rtk_global_localizer',
            output='screen',
            parameters=[
                rtk_base_yaml,
                rtk_map_yaml,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                },
            ],
            condition=IfCondition(start_rtk_localizer),
        ),
        Node(
            package='agri_map_integration',
            executable='vslam_map_aligner',
            name='vslam_map_aligner',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'map_frame': 'map',
                'vslam_frame': 'vslam_map',
                'base_frame': 'bb_robot/base_link',
                'initialpose_topic': '/initialpose',
            }],
            condition=IfCondition(start_vslam_aligner),
        ),
        Node(
            package='agri_map_integration',
            executable='occupancy_grid_overlay',
            name='vslam_occupancy_grid_overlay',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool),
                'input_topic': '/rtabmap/grid_map',
                'output_topic': '/rtabmap/occupied_cells',
                'occupied_threshold': 50,
                'z_offset': 0.10,
            }],
            condition=IfCondition(start_vslam_grid_overlay),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='geospatial_mapping_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(start_rviz),
        ),
    ])
