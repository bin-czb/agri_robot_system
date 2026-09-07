# 导航与路径规划工作空间

## 职责

本工作空间是最高层 overlay，负责正射占据地图、树木地标、RTK/AOA/VSLAM 可视化融合、Nav2 全局规划、路径跟踪和系统集成入口。

## ROS 2 包

- `agri_map_integration`：正射占据地图、树坐标、轨迹和 RViz。
- `agri_nav2_config`：Nav2 costmap、规划器、控制器和行为树参数。
- 历史 `navigation_controller` 和 `trunk_tracking_launch` 已从活动工作空间移除；其原始版本仅保留在旧工程中，不参与新系统构建。

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

## Gazebo 路径规划验证状态

2026-08-05 无界面实测已经确认以下链路可用：

- Gazebo 果园世界可以加载；
- `/clock`、`/model/bb_robot/odometry` 和 `/odometry/filtered` 正常；
- `bb_robot/odom -> bb_robot/base_link` 由 EKF 唯一发布；
- RGB 相机、`/cmd_vel` 和差速插件的桥接能够建立。

当前还不能直接完成 Nav2 闭环，必须先处理：

1. 修正 Gazebo 模型驱动轮的接地高度。当前固定后支撑先接地，左右驱动轮悬空，差速插件收到速度后车体仍不移动。
2. 增加仿真二维 LiDAR 或深度相机，并桥接为 `/scan`。当前模型只有 RGB 相机。
3. 提供与 Gazebo 果园一致的占据地图和 `map -> bb_robot/odom`。可以先用 LiDAR + SLAM Toolbox 建图，也可以制作仿真静态地图并增加仿真定位源。
4. 使用 `use_sim_time:=true` 启动 Nav2，并增加一个统一的 `nav2_sim_bringup.launch.py`。
5. 在 RViz 中显示 `/plan`、`/local_plan`、全局/局部 costmap、目标点和实际轨迹。

AOA、RTK 和无人机链不参与这一阶段。仿真路径规划的最小目标链为：

```text
仿真地图 + 仿真定位 + /scan
  -> NavFn 全局规划
  -> DWB 局部跟踪和避障
  -> /cmd_vel
  -> Gazebo DiffDrive
  -> 仿真里程计和实际轨迹
```

全局路线支持两种验证方式：

- 自动规划：在 RViz 发送目标点，由 NavFn 在占据地图上生成 `/plan`。
- 人工规划：用户给出树行中心线、航点或 `nav_msgs/Path`，系统将其转换为可跟踪路径；局部控制器仍负责跟踪和动态避障。

人工路径不应直接绕过安全层发布 `/cmd_vel`。后续应通过 Nav2 `FollowPath` 动作或专用路线适配节点交给控制器。
