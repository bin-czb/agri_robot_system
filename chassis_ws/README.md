# 底盘工作空间

## 职责

`chassis_ws` 负责公共接口、机器人 URDF、实车底盘通信与控制、轮式里程计、EKF 和 Gazebo 机器人环境。

本次 TD48150B-2E 新底盘开发严格限制在 `chassis_ws` 内进行，不修改 `navigation_ws`、`perception_ws`、`rtk_ws`、`aoa_ws`、`camera_ws` 的现有结构。

## ROS 2 包

- `agri_chassis_can`：新 TD48150B-2E SocketCAN 驱动、差速/滑移转向运动学、实际转速反馈、`/wheel/odometry` 与安全看门狗。
- `agri_robot_description`：机器人 URDF、传感器安装 TF。
- `agri_robot_bringup`：EKF、仿真底盘和统一局部定位入口。
- `trunk_gazebo_worlds`：果园 Gazebo 世界和机器人资源。
- `trunk_interfaces`：树干、AOA 以及旧底盘接口共用消息。
- `agri_chassis_serial`、`serial_bridge_ros2`、`chassis_control`：旧 RS485/Modbus 底盘链路，保留用于历史兼容，不删除、不重构，不与新 CAN 实车链路同时启动。

## 与 Taizhou 的原则

Taizhou 已经完成并验证过底盘 ROS 通信链，因此新系统优先复用其**已经验证的 ROS 层逻辑**，而不是重新设计一套类似方案。

目前从 GitHub 上的 Taizhou 镜像可以确认并借鉴：

```text
标准 /cmd_vel 作为底盘速度入口
SocketCAN 负责 CAN 收发
轮速反馈生成底盘里程计
cmd_vel 超时停车
```

### 手柄逻辑

用户要求手柄必须使用 Taizhou 已经调通的逻辑：A 键切换自动/手柄模式，并且手柄模式必须经过与自动导航相同的 ROS 2 底盘控制链路，以真实验证 ROS -> 底盘节点 -> CAN 是否正常。

此前本分支曾临时写过一个功能相似的 `chassis_mode_teleop.py`，但它不是从 Taizhou 原始已验证源码直接移植。为避免引入未经验证的新逻辑，该临时节点已经删除。

当前 GitHub 的 Taizhou 镜像中没有找到 `/joy` / `sensor_msgs/Joy` / A 键 AUTO-MANUAL 切换的原始源文件，因此在原文件出现之前：

```text
不自己重写 A 键模式切换
不猜手柄 axes/buttons 映射
不猜 AUTO/MANUAL 切换时 Nav2 的暂停/恢复行为
不自行增加新的 cmd_vel mux 逻辑
```

后续找到 Taizhou 原始手柄源码后，只在 `chassis_ws` 内做必要的 ROS 2/新底盘适配，保留其原有控制逻辑。

## 构建

完整 chassis 工作空间：

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

只构建新 CAN 包：

```bash
colcon build --symlink-install --packages-select agri_chassis_can
source install/setup.bash
```

## TD48150B 实车 CAN 首次联调

说明书默认 CAN 波特率为 `250 kbit/s`，采用扩展帧。第一次连接实车必须先被动监听：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_chassis_can setup_can.sh can0 250000
candump can0
```

然后只监听启动：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true
```

此时不允许使能和运动。

当前临时 CAN ID：

```text
控制 ID: 0x06000001
反馈 ID: 0x05800001
心跳 ID: 0x07000001
```

这些值根据厂家说明书默认地址 1 推导，但说明书不同页面存在 ID 补零写法差异，因此仍标记为**未实机确认**。实际值必须以真实驱动器 `candump` 和查询响应为准。

## 未确认数据

以下参数当前均不能视为最终数据：

```text
CAN command / feedback / heartbeat ID
驱动器实际地址
A/B 通道对应左/右哪一侧
左右控制方向符号
左右反馈方向符号
电机到驱动轮实际减速比
有效驱动轮/链轮半径
履带滑移转向等效轮距
驱动器实际设置的最大/额定转速
CAN 转速查询的最终单位确认
轮式里程计协方差
正式最大线速度与角速度
Taizhou 手柄按键/轴映射与 AUTO/MANUAL 源码
```

`agri_chassis_can/config/td48150b.yaml` 对这些临时值逐项写有 `UNCONFIRMED` 注释；更详细的来源、风险和确认方法见：

```text
chassis_ws/src/agri_chassis_can/README.md
```

特别说明：当前 `track_width_m=1.22916` 只由现有 URDF 左右履带 joint 的横向坐标推得，不是已经标定的滑移转向等效轮距；`driver_max_rpm=3000` 只是厂家说明书示例值；二者都不能直接用于最终实车导航标定。

## ROS 速度链路

当前阶段保持标准底盘输入：

```text
Nav2 /cmd_vel
      |
      v
agri_chassis_can
      |
      v
TD48150B-2E
```

`agri_chassis_can` 的默认 YAML 使用 `/cmd_vel`，与 Taizhou 已验证底盘节点和 ROS 2 Nav2 常规接口一致。

手柄逻辑移植完成后，手柄必须进入**同一底盘 ROS 控制入口**，不能绕过 ROS 底盘节点直接发送 CAN。具体切换方式严格以 Taizhou 原始代码为准，不在此处先行假设。

## 里程计与 EKF

新 CAN 节点查询 TD48150B A/B 实际转速，经通道映射、符号、减速比、轮径和等效轮距换算，发布：

```text
/wheel/odometry
```

现有 `agri_robot_bringup/config/ekf_real.yaml` 已把 `/wheel/odometry` 作为轮式里程计输入，因此不需要为了新 CAN 底盘去改定位/导航工作空间。

TF 职责保持不变：

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

`agri_chassis_can` **不发布** `odom -> base_link`，避免破坏现有 TF 所有权。

## 安全约束

首次实车必须按以下顺序：

1. `listen_only=true`，只观察心跳和实际 CAN ID；
2. 确认 ID 后只测试非运动查询；
3. 履带架空，单路低速确认 A/B 左右映射与方向；
4. 确认反馈单位、减速比和最大转速；
5. 低速直线标定有效轮径；
6. 原地/定半径转向标定等效轮距；
7. 验证 `/wheel/odometry -> EKF -> /odometry/filtered`；
8. 最后才接 Nav2 与 Taizhou 原始手柄逻辑。

默认速度上限 `0.10 m/s`、`0.30 rad/s` 只用于首次安全联调，不代表底盘最终性能。

软件急停：

```bash
ros2 service call /agri_chassis_can/estop std_srvs/srv/Trigger '{}'
```

清除软件急停后驱动器仍保持未使能：

```bash
ros2 service call /agri_chassis_can/clear_estop std_srvs/srv/Trigger '{}'
```

详细协议、参数状态和实机确认流程统一维护在 `agri_chassis_can/README.md`，避免在系统其他工作空间复制未确认参数。
