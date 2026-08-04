# agri_nav2_config

This package is the Nav2 configuration home for footprint, SLAM/localization, costmaps, planners, controllers, maps, and RViz.

## Current Files

- `config/slam_toolbox_sim.yaml`: simulation SLAM parameters for `map -> bb_robot/odom`.
- `config/nav2_minimal_params.yaml`: first formal Nav2 skeleton for EKF + RTAB-Map + fake `/scan`.
- `launch/slam_sim.launch.py`: starts `slam_toolbox` with the simulation frame policy.
- `launch/nav2_minimal_formal.launch.py`: starts Nav2 navigation servers only, using an existing TF/localization chain.

## Minimal Formal Nav2 Skeleton

This package now has a first Nav2 parameter set, but it is a preparation step, not a validated navigation loop yet.

Required inputs before launching:

```text
/odometry/filtered
/map
/scan
map -> bb_robot/odom -> bb_robot/base_link
```

Frame and ownership policy:

```text
bb_robot/odom -> bb_robot/base_link: EKF / robot_localization
map -> bb_robot/odom: RTAB-Map
local obstacles: fake /scan from depth, then a better obstacle source later
```

Launch only after those inputs are stable:

```bash
ros2 launch agri_nav2_config nav2_minimal_formal.launch.py
```

This launch includes Nav2's `navigation_launch.py`; it does not start the camera, EKF, RTAB-Map, or a map server. RTAB-Map is expected to publish `/map`, and the global costmap subscribes to that topic.

Preflight checks:

```bash
ros2 topic echo /odometry/filtered --once
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-durability transient_local
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map bb_robot/base_link
```

The final velocity command leaves Nav2 on `/cmd_vel` through `nav2_velocity_smoother`. With only the camera connected, this topic may exist but no chassis will consume it. That state is useful for configuration checks, but it is not a closed-loop navigation test.

## Important Sensor Note

The current `bb_robot` Gazebo model has camera and odometry topics, but no 2D LiDAR / `/scan` topic. The `slam_toolbox` files are ready, but they should be launched only after one of these is true:

- a 2D LiDAR is added to the simulation and bridged to `/scan`;
- another LaserScan topic exists and `scan_topic` is remapped to it.

Check before launching SLAM:

```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
ros2 topic hz /scan
```

After `/scan` exists:

```bash
ros2 launch agri_nav2_config slam_sim.launch.py
```

Expected TF chain:

```text
map -> bb_robot/odom -> bb_robot/base_link
```
