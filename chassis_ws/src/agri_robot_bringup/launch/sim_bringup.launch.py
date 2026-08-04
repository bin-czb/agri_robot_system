import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    world = context.launch_configurations['world']
    partition = context.launch_configurations['partition']
    ip = context.launch_configurations['ip']
    render_engine_gui = context.launch_configurations['render_engine_gui']
    gui_config = context.launch_configurations['gui_config']
    use_sim_time = context.launch_configurations['use_sim_time']
    frame_prefix = context.launch_configurations['frame_prefix']
    gui_gl_integration = context.launch_configurations['gui_gl_integration']
    localization_delay = float(context.launch_configurations['localization_delay'])

    start_gazebo = _as_bool(context.launch_configurations['start_gazebo'])
    headless = _as_bool(context.launch_configurations['headless'])
    gui_software_rendering = _as_bool(context.launch_configurations['gui_software_rendering'])
    start_localization = _as_bool(context.launch_configurations['start_localization'])
    start_gazebo_cmd_camera_bridge = _as_bool(context.launch_configurations['start_gazebo_cmd_camera_bridge'])
    bridge_gazebo_odom = context.launch_configurations['bridge_gazebo_odom']
    bridge_gazebo_clock = context.launch_configurations['bridge_gazebo_clock']

    trunk_share = get_package_share_directory('trunk_gazebo_worlds')
    agri_bringup_share = get_package_share_directory('agri_robot_bringup')
    models_dir = os.path.join(trunk_share, 'models')
    world_path = os.path.join(trunk_share, 'worlds', world)
    agri_gui_config_path = os.path.join(agri_bringup_share, 'config', gui_config)
    trunk_gui_config_path = os.path.join(trunk_share, 'config', gui_config)
    if os.path.isabs(gui_config):
        gui_config_path = gui_config
    elif os.path.exists(agri_gui_config_path):
        gui_config_path = agri_gui_config_path
    else:
        gui_config_path = trunk_gui_config_path
    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    existing_model_path = os.environ.get('IGN_GAZEBO_MODEL_PATH', '')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    if partition in ('', 'auto'):
        partition = f"trunk_orchard_nav_{os.getpid()}"

    resource_path = os.pathsep.join([p for p in [trunk_share, existing_resource_path] if p])
    model_path = os.pathsep.join([p for p in [models_dir, existing_model_path] if p])
    gz_resource_path = os.pathsep.join([p for p in [trunk_share, models_dir, existing_gz_resource_path] if p])
    qt_qpa_platform = os.environ.get('QT_QPA_PLATFORM', 'xcb')

    actions = [
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value=qt_qpa_platform),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='IGN_PARTITION', value=partition),
        SetEnvironmentVariable(name='GZ_PARTITION', value=partition),
        SetEnvironmentVariable(name='IGN_IP', value=ip),
        SetEnvironmentVariable(name='GZ_IP', value=ip),
    ]

    if gui_gl_integration and gui_gl_integration != 'auto':
        actions.append(
            SetEnvironmentVariable(name='QT_XCB_GL_INTEGRATION', value=gui_gl_integration)
        )

    if gui_software_rendering:
        actions.extend([
            SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
            SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3'),
        ])

    if start_gazebo:
        actions.append(
            ExecuteProcess(
                cmd=['ign', 'gazebo', '-v', '4', '-s', '-r', world_path],
                output='screen',
            )
        )

        if not headless:
            actions.append(
                TimerAction(
                    period=1.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ign',
                                'gazebo',
                                '-v',
                                '4',
                                '-g',
                                '--render-engine-gui',
                                render_engine_gui,
                                '--gui-config',
                                gui_config_path,
                            ],
                            output='screen',
                        )
                    ],
                )
            )

        if start_gazebo_cmd_camera_bridge:
            actions.append(
                ExecuteProcess(
                    cmd=[
                        'bash',
                        '-lc',
                        'set +u; '
                        'source /opt/ros/humble/setup.bash; '
                        'ros2 run ros_gz_bridge parameter_bridge '
                        '"/model/bb_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" '
                        '"/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image" '
                        '"/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo" '
                        '--ros-args -r /model/bb_robot/cmd_vel:=/cmd_vel',
                    ],
                    output='screen',
                )
            )

    if start_localization:
        actions.append(
            TimerAction(
                period=localization_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(agri_bringup_share, 'launch', 'localization.launch.py')
                        ),
                        launch_arguments={
                            'use_sim_time': use_sim_time,
                            'frame_prefix': frame_prefix,
                            'bridge_gazebo_odom': bridge_gazebo_odom,
                            'bridge_gazebo_clock': bridge_gazebo_clock,
                        }.items(),
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='hexarotor_forest_orchard.sdf',
            description='World SDF passed to trunk_gazebo_worlds.',
        ),
        DeclareLaunchArgument(
            'partition',
            default_value='auto',
            description='Gazebo transport partition. auto creates a unique partition per launch.',
        ),
        DeclareLaunchArgument(
            'ip',
            default_value='127.0.0.1',
            description='Gazebo transport IP.',
        ),
        DeclareLaunchArgument(
            'render_engine_gui',
            default_value='ogre2',
            description='Gazebo GUI render engine when headless is false.',
        ),
        DeclareLaunchArgument(
            'gui_config',
            default_value='simple_gui_ogre2.config',
            description='Gazebo GUI config. Relative paths are searched in agri_robot_bringup/config first, then trunk_gazebo_worlds/config.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='true',
            description='Start Gazebo.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Start only the Gazebo server. Set false to include the existing Gazebo GUI launch.',
        ),
        DeclareLaunchArgument(
            'gui_software_rendering',
            default_value='false',
            description='Force Mesa software OpenGL rendering for Gazebo GUI.',
        ),
        DeclareLaunchArgument(
            'gui_gl_integration',
            default_value='auto',
            description='Qt XCB OpenGL integration for Gazebo GUI: auto, xcb_glx, xcb_egl, or none.',
        ),
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Start robot description, Gazebo odometry bridge, and EKF.',
        ),
        DeclareLaunchArgument(
            'start_gazebo_cmd_camera_bridge',
            default_value='true',
            description='Start cmd_vel and camera bridges. This launch does not bridge Gazebo TF.',
        ),
        DeclareLaunchArgument(
            'bridge_gazebo_odom',
            default_value='true',
            description='Bridge /model/bb_robot/odometry into ROS 2 for robot_localization.',
        ),
        DeclareLaunchArgument(
            'bridge_gazebo_clock',
            default_value='true',
            description='Bridge Gazebo /clock into ROS 2 for sim time.',
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='bb_robot/',
            description='Simulation frame prefix. Keep bb_robot/ while using the current Gazebo model.',
        ),
        DeclareLaunchArgument(
            'localization_delay',
            default_value='3.0',
            description='Delay before starting localization after Gazebo launch.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
