import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share_dir = get_package_share_directory('trunk_gazebo_worlds')
    world = LaunchConfiguration('world')
    partition = LaunchConfiguration('partition')
    ip = LaunchConfiguration('ip')
    render_engine_gui = LaunchConfiguration('render_engine_gui')
    gui_config = LaunchConfiguration('gui_config')
    use_ros_gz_bridge = LaunchConfiguration('use_ros_gz_bridge')
    world_path = PathJoinSubstitution([
        TextSubstitution(text=share_dir),
        TextSubstitution(text='worlds'),
        world,
    ])
    
    gui_config_path = PathJoinSubstitution([
        TextSubstitution(text=share_dir),
        TextSubstitution(text='config'),
        gui_config,
    ])

    # Gazebo Sim (Ignition) uses these env vars to find worlds/models.
    # We prepend our package share dir so `model://tree_trunk` resolves.
    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    existing_model_path = os.environ.get('IGN_GAZEBO_MODEL_PATH', '')
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    models_dir = os.path.join(share_dir, 'models')

    resource_path = os.pathsep.join([p for p in [share_dir, existing_resource_path] if p])
    model_path = os.pathsep.join([p for p in [models_dir, existing_model_path] if p])
    # Gazebo Sim (Fortress+ / Gazebo 6) primarily uses GZ_SIM_RESOURCE_PATH for
    # both worlds and models. Add both the package share dir and its models dir.
    gz_resource_path = os.pathsep.join([p for p in [share_dir, models_dir, existing_gz_resource_path] if p])

    # On Ubuntu 22.04, Wayland + Qt can result in "no window / blank window".
    # Force X11 backend by default. Users can override in their shell if needed.
    qt_qpa_platform = os.environ.get('QT_QPA_PLATFORM', 'xcb')
    default_partition = f"trunk_orchard_{os.getpid()}"

    server_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-s', '-r', world_path],
        output='screen'
    )

    gui_process = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-g', '--render-engine-gui', render_engine_gui, '--gui-config', gui_config_path],
        output='screen'
    )

    # Optional: ROS 2 <-> Gazebo bridge.
    # IMPORTANT: Use one-way bridges to avoid feedback loops on /cmd_vel.
    bridge_cmd_vel = ExecuteProcess(
        condition=IfCondition(use_ros_gz_bridge),
        cmd=[
            'bash', '-lc',
            'set +u; '
            'source /opt/ros/humble/setup.bash; '
            # NOTE: Ignition Gazebo 6 uses ignition.msgs.* (not gz.msgs.*)
            'ros2 run ros_gz_bridge parameter_bridge '
            # cmd_vel: ROS -> Gazebo (])
            '"/model/bb_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" '
            # camera topics: Gazebo -> ROS ([)
            '"/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image" '
            '"/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo" '
            '--ros-args -r /model/bb_robot/cmd_vel:=/cmd_vel'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            # Default to OGRE1 variant for stability on more GPUs. Users can switch to OGRE2 when needed.
            default_value='hexarotor_forest_orchard_ogre.sdf',
            description='World SDF to load: hexarotor_forest_orchard_ogre.sdf (OGRE1, most stable), hexarotor_forest_orchard.sdf (OGRE2 sensors, nicer PBR), fuel_hilly_orchard.sdf (Fuel terrain + Fuel trees), hilly_orchard.sdf (offline hills + simple trees), hilly_orchard_pine_fuel.sdf (hills + Fuel Pine Tree), orchard.sdf (flat), orchard_pine_fuel.sdf (flat + Fuel Pine Tree)'
        ),
        DeclareLaunchArgument(
            'partition',
            default_value=default_partition,
            description='Gazebo transport partition. Use a unique name to avoid connecting to old running servers.'
        ),
        DeclareLaunchArgument(
            'ip',
            default_value='127.0.0.1',
            description='Gazebo transport IP (bind / advertise). Using 127.0.0.1 avoids multi-NIC mismatch.'
        ),
        DeclareLaunchArgument(
            'render_engine_gui',
            default_value='ogre',
            description='GUI render engine: ogre (OGRE1, most compatible) or ogre2 (OGRE2, faster but may crash on some drivers).'
        ),
        DeclareLaunchArgument(
            'gui_config',
            default_value='simple_gui.config',
            description='GUI layout config file under share/trunk_gazebo_worlds/config/. Use simple_gui.config for OGRE1; use simple_gui_ogre2.config for OGRE2 / PBR Fuel models.'
        ),
        DeclareLaunchArgument(
            'use_ros_gz_bridge',
            default_value='false',
            description='If true, starts ros_gz_bridge to bridge ROS 2 /cmd_vel to Gazebo /model/bb_robot/cmd_vel.'
        ),
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value=qt_qpa_platform),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_MODEL_PATH', value=model_path),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        # Ensure Server and GUI talk to each other (and not to any old running Gazebo instance)
        SetEnvironmentVariable(name='IGN_PARTITION', value=partition),
        SetEnvironmentVariable(name='GZ_PARTITION', value=partition),
        SetEnvironmentVariable(name='IGN_IP', value=ip),
        SetEnvironmentVariable(name='GZ_IP', value=ip),

        # Run Server first, then GUI shortly after to avoid race conditions.
        server_process,
        TimerAction(period=1.0, actions=[gui_process]),
        bridge_cmd_vel,
    ])


