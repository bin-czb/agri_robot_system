# 导航与路径规划工作空间

## 职责

本工作空间是最高层 overlay，负责正射占据地图、树木地标、RTK/AOA/VSLAM 可视化融合、Nav2 全局规划、路径跟踪和系统集成入口。

## ROS 2 包

- `agri_map_integration`：正射占据地图、树坐标、轨迹和 RViz。
- `agri_nav2_config`：Nav2 costmap、规划器、控制器和行为树参数。
- `navigation_controller`：历史树行跟随控制器，保留用于对照测试。
- `trunk_tracking_launch`：历史系统组合入口，后续逐步收口到统一 bringup。

## 构建

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
cd /home/czb/agri_robot_system/navigation_ws
colcon build --symlink-install
```

## 地图与 RViz

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_map_integration integrated_mapping.launch.py \
  start_vslam:=true \
  start_calibrated_imu:=true \
  start_rtk_localizer:=false \
  start_rviz:=true
```

当前占据地图约定：白色可通行，黑色禁止通行。树坐标来自 `tree_landmarks.csv`，所有全局定位源必须先转换到同一个 `map` 坐标。

## Nav2 前置条件

```text
/map                         nav_msgs/OccupancyGrid
/scan                        sensor_msgs/LaserScan
/odometry/filtered           nav_msgs/Odometry
map -> bb_robot/odom         全局定位融合器
bb_robot/odom -> bb_robot/base_link  EKF
```

前置检查：

```bash
ros2 topic echo /map --once --qos-durability transient_local
ros2 topic hz /scan
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo map bb_robot/base_link
```

启动：

```bash
ros2 launch agri_nav2_config nav2_minimal_formal.launch.py
```

当前配置仍是 NavFn + DWB 基线。下一阶段按已确定路线升级为 Smac Planner 2D + RPP；底盘真实尺寸和 footprint 核实后，再评估 State Lattice。

没有底盘时只能检查地图、TF、全局路径和 `/cmd_vel` 输出，不能称为导航闭环。
