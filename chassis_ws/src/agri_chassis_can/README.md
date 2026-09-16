# agri_chassis_can

`agri_chassis_can` 是 `agri_robot_system/chassis_ws` 内为新 TD48150B-2E 双驱伺服底盘建立的 ROS 2 Humble 实车包。底盘通信、手柄输入、AUTO/MANUAL 命令选择和轮式里程计都放在 `chassis_ws` 内，不改变 `navigation_ws`、`perception_ws`、`rtk_ws`、`aoa_ws`、`camera_ws` 的原有职责。

## 1. 系统信息流：先看这一节

新系统不能简单把 Taizhou 文件逐行复制过来，因为两套系统的 CAN 协议不同，而且当前 `agri_robot_system` 的 Nav2 已经带有 `velocity_smoother`。因此保留 Taizhou 已经验证过的使用逻辑——A 键切换 AUTO/MANUAL、手柄和自动导航共用最终底盘 ROS 链路——同时按照当前系统真实话题关系接线。

当前 `agri_robot_system` 使用 ROS 2 Humble 的 `nav2_bringup/navigation_launch.py`。其中 `controller_server` 的速度输出被重映射到 `/cmd_vel_nav`，随后 `velocity_smoother` 接收 `/cmd_vel_nav`，并把最终平滑速度发布为 `/cmd_vel`。所以本系统的 AUTO 输入必须取 `/cmd_vel`，不能直接取 `/cmd_vel_nav`，否则会绕过现有速度平滑器。

```text
AUTO
====
Nav2 controller_server
        |
        |  /cmd_vel_nav
        v
Nav2 velocity_smoother
        |
        |  /cmd_vel       <- 当前导航系统最终速度
        v
chassis_mode_teleop
        |
        |  /chassis/cmd_vel
        v
agri_chassis_can
        |
        |  TD48150B 29-bit CAN
        v
TD48150B-2E -> 左/右履带


MANUAL
======
Linux / SDL 手柄
        |
        v
joy_node
        |
        |  /joy  (sensor_msgs/msg/Joy)
        v
chassis_mode_teleop
        |
        |  /chassis/cmd_vel
        v
agri_chassis_can
        |
        |  TD48150B 29-bit CAN
        v
TD48150B-2E -> 左/右履带


反馈
====
TD48150B A/B 实际转速
        |
        v
agri_chassis_can
        |
        |  /wheel/odometry
        v
robot_localization EKF  <--- camera IMU
        |
        |  /odometry/filtered
        v
odom -> base_link
```

这里最重要的约束是：`agri_chassis_can` **只订阅 `/chassis/cmd_vel`**。Nav2 和手柄都不能直接同时向 CAN 节点发速度。`chassis_mode_teleop` 是唯一命令选择点，因此不会出现两个 ROS 发布者同时争抢底盘控制权。

这样 MANUAL 模式真正验证的是：

```text
手柄 -> joy_node -> ROS 2 -> 命令选择 -> 底盘 ROS 节点
     -> 运动学 -> TD48150B 协议 -> SocketCAN -> 驱动器 -> 电机
```

而不是绕开 ROS 底盘节点直接控制 CAN。

## 2. 与 Taizhou 的对应关系

Taizhou 当前 GitHub 镜像中可以确认的底盘核心行为包括：标准 `/cmd_vel` 输入、SocketCAN 收发、轮速反馈生成里程计以及 `cmd_vel` 超时停车。用户确认 Taizhou 实机使用的 `chassis_bringup.launch.py` 还启动了 `joy_node`，并使用 A 键切换自动/手柄模式。

当前公开的 `bin-czb/Taizhou` GitHub 树和提交历史中没有保存这个 `chassis_bringup.launch.py` 文件，因此这里不能声称是“逐字复制原文件”。本分支按已经确认的 Taizhou 使用逻辑实现，并把当前 `agri_robot_system` 的 Nav2 信息流重新核对后接入。凡是无法从当前仓库确认的手柄数字映射都保留为参数，并明确标记为待实机确认。

手柄的 ROS 驱动仍使用标准 `joy` 包的 `joy_node`。在 ROS 2 版本中该节点使用 `device_id` 选择 SDL 手柄设备，并发布 `/joy`；它自身不发布 `Twist`，也不负责 AUTO/MANUAL 仲裁。

## 3. AUTO/MANUAL 行为

`chassis_mode_teleop` 默认从 `AUTO` 启动。A 键采用上升沿切换，因此一直按住 A 不会连续翻转模式。切换瞬间先向 `/chassis/cmd_vel` 发布一次零速度，然后要求新模式的数据源在切换之后重新产生新消息，旧缓存速度不会被带到新模式。

例如从 AUTO 切到 MANUAL：

```text
Nav2 正在发布速度
      |
按 A
      |
立即发布 Twist() = 0
      |
进入 MANUAL
      |
A 键所在的那一帧 Joy 不作为运动命令
      |
等待下一帧 /joy
      |
才允许摇杆速度进入 /chassis/cmd_vel
```

从 MANUAL 切回 AUTO 同理：先零速，再等待 Nav2 `velocity_smoother` 在切换后发布新的 `/cmd_vel`。

两路输入都有独立 0.5 s 超时；选中的数据源过期时，选择节点持续发布零速度。CAN 节点自身还有第二层 `cmd_vel_timeout_s`，因此上层选择器和底层驱动各自有独立看门狗。

## 4. 手柄映射：当前仍需确认的项目

当前 YAML 采用常见 Xbox/Taizhou 使用习惯作为联调默认值：

```yaml
toggle_button: 0
linear_axis: 1
angular_axis: 0
linear_axis_sign: 1.0
angular_axis_sign: 1.0
```

其中 `toggle_button: 0` 预期为 A 键，`linear_axis: 1` 预期为左摇杆纵向，`angular_axis: 0` 预期为左摇杆横向。由于当前公开 Taizhou 仓库没有原始 `chassis_bringup.launch.py`，这些**不是硬件无关常量**。第一次连接实际手柄时必须先在 `listen_only=true` 下检查：

```bash
ros2 topic echo /joy
```

逐个按 A 键、推前后和左右摇杆，确认 `buttons[]`、`axes[]` 的实际编号和正负方向，再改 `config/td48150b.yaml`。这一过程不会向底盘发送 CAN，因为首次调试保持 `listen_only=true`。

## 5. launch 使用方式

CAN 单独联调，不启动手柄：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true \
  start_joy:=false
```

检查手柄和 AUTO/MANUAL ROS 链路，但仍禁止 CAN 主动发送：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true \
  start_joy:=true \
  joy_device_id:=0
```

另开终端观察：

```bash
ros2 topic echo /joy
ros2 topic echo /chassis/control_mode
ros2 topic echo /chassis/cmd_vel
```

`joy_device_id` 是 ROS 2 `joy_node` 的 SDL 设备编号，不是 `/dev/input/js0` 字符串路径。默认值为 `0`。

## 6. 不启动 Nav2也能先验证命令选择

在 `listen_only=true` 时，可以用 ROS 话题模拟 Nav2 最终 `/cmd_vel`，验证 AUTO 通路而不让实车运动：

```bash
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

此时 AUTO 模式应能在 `/chassis/cmd_vel` 看到相同命令。按 A 进入 MANUAL 后，模拟 Nav2 的 `/cmd_vel` 应不再通过；只有 `/joy` 转换出的手柄命令可以进入 `/chassis/cmd_vel`。再按 A 回 AUTO 后，手柄速度不再通过，等待新的 `/cmd_vel` 后恢复自动链路。

这一步是纯 ROS 2 信息流测试，不需要关闭 `listen_only`。

## 7. TD48150B-2E 已由说明书确认的 CAN 事实

说明书确认：

```text
默认波特率: 250 kbit/s
CAN 帧: 29 bit 扩展帧
驱动器默认地址: 1
驱动器约 1 s 自动发送一次心跳
控制命令间隔不能超过 1000 ms
速度控制标幺范围: -10000 .. +10000
```

A/B 使能：

```text
A: 23 0D 20 01 00 00 00 00
B: 23 0D 20 02 00 00 00 00
```

A/B 失能：

```text
A: 23 0C 20 01 00 00 00 00
B: 23 0C 20 02 00 00 00 00
```

A/B 速度设定：

```text
A: 23 00 20 01 XX XX XX XX
B: 23 00 20 02 XX XX XX XX
```

后 4 Byte 按有符号 32 bit 大端二补码编码。

转速查询：

```text
TX: 40 03 21 01 00 00 00 00
RX: 60 03 21 01 A_H A_L B_H B_L
```

## 8. 所有未确认参数

以下内容在实车确认前都不能当成最终标定值；`config/td48150b.yaml` 中逐项保留了对应注释。

| 参数 | 当前临时值 | 状态 | 确认方法 |
|---|---:|---|---|
| `command_can_id` | `0x06000001` | 未确认 | `candump can0` + 实际查询 |
| `feedback_can_id` | `0x05800001` | 未确认 | 非运动查询后核对返回 ID |
| `heartbeat_can_id` | `0x07000001` | 未确认 | 上电观察约 1 Hz 心跳 |
| 驱动器地址 | `1` | 未确认本机 | 实际 CAN ID/上位机设置 |
| `left_channel` | `A` | 未确认 | 履带架空单路低速点动 |
| `right_channel` | `B` | 未确认 | 履带架空单路低速点动 |
| 左右控制符号 | `+1/+1` | 未确认 | 低速正命令检查方向 |
| 左右反馈符号 | `+1/+1` | 未确认 | 正向点动检查反馈符号 |
| `wheel_radius_m` | `0.20` | 未确认 | 实测直线距离标定有效半径 |
| `track_width_m` | `1.22916` | 未确认 | 原地/定半径转向标定等效轮距 |
| `gear_ratio` | `1.0` | 未确认 | 机械资料 + 转速实测 |
| `driver_max_rpm` | `3000` | 未确认本机 | 驱动器参数/铭牌 |
| CAN 转速反馈单位 | `normalized` | 未最终确认 | 已知低速给定与反馈对比 |
| 里程计协方差 | 初始保守值 | 未标定 | 重复直线/转向统计 |
| 最大线/角速度 | `0.10/0.30` | 仅联调限制 | 实车能力和 Nav2 调参 |
| A 键编号 | `0` | 待手柄确认 | `ros2 topic echo /joy` |
| 前后轴编号 | `1` | 待手柄确认 | `ros2 topic echo /joy` |
| 左右轴编号 | `0` | 待手柄确认 | `ros2 topic echo /joy` |
| 摇杆正负方向 | `+1/+1` | 待手柄确认 | `/joy` + 纯 ROS 链路检查 |

当前 `track_width_m=1.22916` 只由现有 URDF 左右履带 joint 的横向位置推得，不是已标定的履带滑移等效轮距。`driver_max_rpm=3000` 来自厂家说明书示例，也不能视为当前驱动器最终设置值。

## 9. 里程计与 TF 所有权

`agri_chassis_can` 根据 TD48150B 两路实际转速计算 `/wheel/odometry`，但**不发布** `odom -> base_link`。现有 `robot_localization` 继续融合轮式里程计和相机 IMU，并作为该 TF 的唯一发布者。这样换底盘不会破坏当前定位、VSLAM、RTK/AOA 和 Nav2 的 TF 结构。

## 10. 首次实车联调顺序

必须先把 CAN 和 ROS 命令链分开验证。推荐顺序是：先 `candump` 被动确认心跳和 ID；再用 `listen_only=true` 验证 `/joy`、A 键、AUTO/MANUAL 和 `/chassis/cmd_vel`；然后只测试非运动 CAN 查询；接着履带架空确认 A/B 左右关系、方向和反馈单位；之后标定轮径、减速比和等效轮距；验证 `/wheel/odometry -> EKF -> /odometry/filtered`；最后才允许把 `listen_only` 关闭并进行 Nav2 实车闭环。

软件急停接口：

```bash
ros2 service call /agri_chassis_can/estop std_srvs/srv/Trigger '{}'
ros2 service call /agri_chassis_can/clear_estop std_srvs/srv/Trigger '{}'
ros2 service call /agri_chassis_can/disable std_srvs/srv/Trigger '{}'
```

软件急停不能替代实体急停或可切断驱动器动力/使能的硬件安全措施。

## 11. 当前开发状态

代码层已经完成 TD48150B 协议封装、29-bit SocketCAN、`/chassis/cmd_vel` 到双路速度控制、轮速反馈到 `/wheel/odometry`、故障/电流/电压/温度诊断、AUTO/MANUAL 命令选择、可选 `joy_node` 启动和软件安全看门狗。

仍未完成的是实车 CAN ID 确认、A/B 映射、方向、减速比、轮径、等效轮距、反馈单位、手柄实际索引以及整车 Nav2 闭环验证。因此这个分支仍是联调分支，PR 保持 Draft，在这些项目实测完成前不应当视为最终正式配置。
