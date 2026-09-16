# 底盘工作空间

## 职责

`chassis_ws` 负责公共接口、机器人 URDF、实车底盘通信与控制、轮式里程计、命令仲裁、EKF 和 Gazebo 机器人环境。

本次 TD48150B-2E 新底盘开发严格限制在 `chassis_ws` 内进行，不修改 `navigation_ws`、`perception_ws`、`rtk_ws`、`aoa_ws`、`camera_ws` 的现有结构。

当前主要包：

- `agri_chassis_can`：TD48150B-2E SocketCAN 驱动、差速/滑移转向运动学、AUTO/MANUAL 命令选择、`joy_node` 启动、实际转速反馈、`/wheel/odometry` 与安全看门狗。
- `agri_robot_description`：机器人 URDF、传感器安装 TF。
- `agri_robot_bringup`：EKF、仿真底盘和统一局部定位入口。
- `trunk_gazebo_worlds`：果园 Gazebo 世界和机器人资源。
- `trunk_interfaces`：树干、AOA 以及旧底盘接口共用消息。
- `agri_chassis_serial`、`serial_bridge_ros2`、`chassis_control`：旧 RS485/Modbus 底盘链路，保留历史兼容，不删除、不重构，不与新 CAN 实车链路同时启动。

## Taizhou 逻辑如何适配到新系统

Taizhou 已经实机验证了“标准 ROS 速度入口 -> 底盘节点 -> SocketCAN -> 底盘”和轮速里程计链路。用户确认 Taizhou 实际使用的 `chassis_bringup.launch.py` 中还启动了 `joy_node`，并通过 A 键在自动与手柄模式间切换。

当前公开的 Taizhou GitHub 镜像没有保存该 `chassis_bringup.launch.py`，因此这里不声称逐字复制原文件；但使用逻辑严格保持：A 键切换 AUTO/MANUAL，且手柄和自动导航最终必须经过同一个 ROS 底盘节点，手柄不能绕开 ROS 节点直接发 CAN。

同时必须适配当前 `agri_robot_system` 的真实 Nav2 信息流。ROS 2 Humble `navigation_launch.py` 中，`controller_server` 输出被重映射为 `/cmd_vel_nav`，现有 `velocity_smoother` 读取它并把最终平滑结果发布为 `/cmd_vel`。因此当前系统使用：

```text
AUTO:
controller_server
   -> /cmd_vel_nav
   -> velocity_smoother
   -> /cmd_vel
   -> chassis_mode_teleop
   -> /chassis/cmd_vel
   -> agri_chassis_can
   -> TD48150B

MANUAL:
手柄
   -> joy_node
   -> /joy
   -> chassis_mode_teleop
   -> /chassis/cmd_vel
   -> agri_chassis_can
   -> TD48150B
```

不能让 `agri_chassis_can` 直接订阅 `/cmd_vel` 并同时再接一个手柄 `Twist` 发布者，否则 ROS 2 不提供“谁优先”的天然仲裁，两个发布源会在底盘节点前竞争。现在 CAN 节点只订阅 `/chassis/cmd_vel`，由 `chassis_mode_teleop` 保证任意时刻只有一个控制源能够进入下游。

A 键切换采用上升沿，切换时先发零速度，并要求新模式在切换后收到一帧新数据才恢复运动，避免旧 Nav2 命令或旧摇杆位置跨模式生效。AUTO 和 MANUAL 都有 0.5 s 源超时，CAN 节点还有独立的 `cmd_vel` 超时作为第二层安全保护。

## 构建

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建新 CAN 包：

```bash
colcon build --symlink-install --packages-select agri_chassis_can
```

需要 ROS 2 `joy` 包：

```bash
sudo apt install ros-humble-joy
```

## TD48150B 首次联调

先配置 CAN：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_chassis_can setup_can.sh can0 250000
candump can0
```

第一次必须保持只监听：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true \
  start_joy:=false
```

检查手柄时仍然保持 CAN 只监听：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true \
  start_joy:=true \
  joy_device_id:=0
```

观察：

```bash
ros2 topic echo /joy
ros2 topic echo /chassis/control_mode
ros2 topic echo /chassis/cmd_vel
```

ROS 2 `joy_node` 使用 `device_id` 选择 SDL 手柄设备。当前映射默认预期 `button[0]` 是 A、`axis[1]` 是前后、`axis[0]` 是左右，但当前公开 Taizhou 仓库没有原始 launch 文件可核对这些数字，因此第一次实际连接手柄必须通过 `/joy` 确认按钮、轴编号和正负方向。确认之前不能关闭 `listen_only`。

## 未确认数据

当前仍不能视为正式值的内容包括：CAN command/feedback/heartbeat ID、驱动器地址、A/B 对应左右履带关系、左右控制与反馈符号、减速比、有效驱动轮半径、履带滑移等效轮距、驱动器实际最大/额定转速、CAN 转速反馈最终单位、里程计协方差、正式最大线速度和角速度，以及实际手柄的按钮/轴编号和方向。

这些临时值均在 `agri_chassis_can/config/td48150b.yaml` 中有 `UNCONFIRMED` 或验证说明，更详细的来源和验证方法统一写在：

```text
chassis_ws/src/agri_chassis_can/README.md
```

特别地，当前 `track_width_m=1.22916` 只是由现有 URDF 左右履带 joint 横向坐标推得，不是已标定的履带滑移等效轮距；`driver_max_rpm=3000` 是厂家说明书示例值，不代表当前驱动器最终参数。

## 里程计与原系统关系

新 CAN 节点发布：

```text
/wheel/odometry
```

现有 `robot_localization` 继续融合轮式里程计和相机 IMU：

```text
TD48150B wheel odom + camera IMU
               |
               v
       robot_localization EKF
               |
               v
       /odometry/filtered
               |
               v
         odom -> base_link
```

`agri_chassis_can` 不发布 `odom -> base_link`，所以不会与现有 EKF 抢 TF，也不要求为了换底盘修改导航、VSLAM、RTK 或 AOA 工作空间。

## 安全顺序

首次实车应依次完成：被动确认 CAN 心跳和 ID；`listen_only=true` 检查手柄 `/joy` 和 AUTO/MANUAL ROS 信息流；只做非运动 CAN 查询；履带架空确认 A/B、方向与反馈单位；标定减速比、有效轮径与等效轮距；验证 `/wheel/odometry -> EKF -> /odometry/filtered`；最后才关闭 `listen_only` 并进行 Nav2 实车闭环。

默认 `0.10 m/s` 和 `0.30 rad/s` 只是首次联调限速。软件急停接口存在，但不能替代实体急停或可切断驱动动力/使能的硬件安全措施。
