# 农业机器人 ROS 2 系统

本目录是农业机器人系统的唯一有效开发根目录。系统使用 ROS 2 Humble，按硬件和职责拆分为六个工作空间，避免驱动、定位、感知和导航代码相互混放。

## 目录结构

```text
agri_robot_system/
├── camera_ws/       Gemini 335L 驱动与相机启动
├── aoa_ws/          AOA 驱动、无人机位置源与 AOA 全局定位
├── rtk_ws/          UM982、NTRIP 与 RTK 地图定位
├── chassis_ws/      公共消息、机器人模型、底盘通信、控制和仿真
├── perception_ws/   RTAB-Map、深度障碍、树干检测和点云处理
├── navigation_ws/   正射地图、树坐标、Nav2、路径规划和系统集成
├── scripts/         统一构建、环境加载和检查脚本
├── data/            标定结果、实验数据和 rosbag
├── docs/            架构与迁移审计
└── archive/         已退出主链但需要保留的历史资料
```

YDLidar、Livox、HWT601、visual-crop-row-navigation 和旧 FAST-LIO/MPPI 不属于当前系统，不进入六个活动工作空间。

## 环境要求

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Fortress / ros_gz
Nav2
robot_localization
RTAB-Map ROS 2
Orbbec Gemini 335L SDK
```

## 首次构建

```bash
cd /home/czb/agri_robot_system
bash scripts/build_all.bash
```

构建顺序固定为：

```text
camera_ws
  -> chassis_ws
  -> aoa_ws
  -> rtk_ws
  -> perception_ws
  -> navigation_ws
```

## 每个新终端加载环境

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
```

不要再混合 source 旧的 `trunk_tracking_ws`、`ROS_RTK/ros2_ws` 或旧 Orbbec 工作空间，否则可能加载同名包的旧版本。

## 常用启动

### 相机

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_camera_bringup gemini335l.launch.py \
  enable_point_cloud:=true \
  enable_colored_point_cloud:=true \
  enable_sync_output_accel_gyro:=true \
  enable_accel:=true \
  enable_gyro:=true \
  accel_rate:=200hz \
  gyro_rate:=200hz
```

### AOA 地图定位

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_global_localization aoa_global_localization.launch.py \
  aoa_serial_port:=/dev/ttyACM0 \
  mavlink_port:=/dev/ttyUSB0 \
  heading_mode:=imu_relative \
  initial_heading_enu_deg:=0.0
```

### RTK

第一次使用先保存 NTRIP 凭据：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_rtk_localization configure_ntrip_credentials
```

启动驱动、NTRIP 和地图定位：

```bash
ros2 run agri_rtk_localization start_um982_ntrip
```

### 相机 VSLAM

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_stable_scan.launch.py
```

### 正射地图、VSLAM 和 RViz

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_map_integration integrated_mapping.launch.py \
  start_vslam:=true \
  start_calibrated_imu:=true \
  start_rtk_localizer:=false \
  start_rviz:=true
```

AOA 与 RTK 是两种互斥全局定位模式。正式系统后续由定位源选择器统一发布 `map -> odom`，Nav2 不直接区分定位源。

### Gazebo 仿真

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_robot_bringup sim_bringup.launch.py
```

### Nav2

Nav2 启动前必须存在 `/map`、`/scan`、`/odometry/filtered` 和完整 TF：

```text
map -> bb_robot/odom -> bb_robot/base_link
```

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_nav2_config nav2_minimal_formal.launch.py
```

## Git 管理

本目录作为独立项目管理，建议远端仓库名使用 `agri_robot_system_ros2`，不要绑定或强推到已有项目。Orbbec SDK 保持独立上游仓库，通过 `camera_ws/orbbec.repos` 固定版本，不纳入主仓库历史。

## 安全规则

- 不将 NTRIP 密码提交到 Git。
- 不提交 `build/install/log` 和 rosbag。
- AOA 与 RTK 不同时发布 `/global_pose/selected`。
- 在底盘实车尺寸确认前，不进行自主运动测试。
- 底盘遥控和 Nav2 `/cmd_vel` 后续必须经过命令仲裁与急停层。
