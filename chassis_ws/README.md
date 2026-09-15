# 底盘工作空间

## 职责

本工作空间负责公共接口、机器人 URDF、实车底盘通信与控制、轮式里程计、AUTO/MANUAL 手柄切换、EKF 和 Gazebo 机器人环境。

新底盘正式链路采用 TD48150B-2E 双驱伺服驱动器的 SocketCAN 协议。旧的 RS485/Modbus 包暂时保留用于历史兼容，但不再作为新底盘的正式控制入口。

## ROS 2 包

- `agri_chassis_can`：TD48150B-2E 扩展帧 CAN 通信、速度控制、实际转速反馈、`/wheel/odometry`、A 键 AUTO/MANUAL 手柄切换和安全看门狗。
- `agri_robot_description`：机器人 URDF、传感器安装 TF。
- `agri_robot_bringup`：EKF、仿真底盘和统一局部定位入口。
- `trunk_gazebo_worlds`：果园 Gazebo 世界和机器人资源。
- `trunk_interfaces`：树干、AOA 以及旧底盘接口共用消息。
- `agri_chassis_serial`、`serial_bridge_ros2`、`chassis_control`：旧 RS485/Modbus 底盘链路，仅保留兼容，不与新 CAN 底盘同时启动。

## 构建

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## TD48150B 实车 CAN 首次联调

说明书默认 CAN 波特率为 `250 kbit/s`，使用 29 bit 扩展帧。第一次连接实车时必须先被动监听，确认实际心跳 ID，再允许发送控制帧。

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_chassis_can setup_can.sh can0 250000
candump can0
```

然后以只监听模式启动：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true \
  start_joy:=true
```

当前按驱动器地址 1 配置：

```text
控制 ID: 0x06000001
反馈 ID: 0x05800001
心跳 ID: 0x07000001
```

由于厂家说明书文字与示例的 ID 补零写法存在差异，实机必须以 `candump` 的实际扩展帧 ID 为准。确认前不要关闭 `listen_only`。

## A 键切换 AUTO / MANUAL

新手柄链路保留 Taizhou 项目的使用习惯：默认 A 键（button index 0）在 `AUTO` 和 `MANUAL` 之间切换。

```text
AUTO:
Nav2 /cmd_vel
      |
      v
chassis_mode_teleop
      |
      v
/chassis/cmd_vel
      |
      v
agri_chassis_can -> CAN

MANUAL:
/joy -- A切换 + 摇杆
      |
      v
chassis_mode_teleop
      |
      v
/chassis/cmd_vel
      |
      v
agri_chassis_can -> CAN
```

因此手柄模式和自动导航最终都进入同一个 `/chassis/cmd_vel` 订阅入口，手柄可以真实验证 ROS 2 底盘通信链路，而不是绕过底盘节点直接控制 CAN。

查看当前模式：

```bash
ros2 topic echo /chassis/control_mode
```

没有手柄时也可以切换模式：

```bash
ros2 service call /chassis_mode_teleop/set_auto std_srvs/srv/Trigger '{}'
ros2 service call /chassis_mode_teleop/set_manual std_srvs/srv/Trigger '{}'
ros2 service call /chassis_mode_teleop/toggle std_srvs/srv/Trigger '{}'
```

不同手柄的按键和轴编号可能不同，首次连接先执行 `ros2 topic echo /joy`，再按实际映射修改 `agri_chassis_can/config/td48150b.yaml` 中的 `toggle_button`、`linear_axis` 和 `angular_axis`。

## 允许运动前必须确认

1. CAN 心跳、控制、反馈 ID；
2. A/B 通道对应左/右哪一侧；
3. 左右电机控制正负号和反馈正负号；
4. 转速反馈究竟是实际 RPM 还是 `-10000..10000` 标幺值；
5. 驱动轮/链轮有效半径、履带有效中心距、减速比；
6. 驱动器中设置的最大转速。

确认后再以低速、履带架空状态启动主动模式：

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=false \
  start_joy:=true

ros2 service call /agri_chassis_can/enable std_srvs/srv/Trigger '{}'
```

默认速度限制为 `0.10 m/s` 和 `0.30 rad/s`，用于首次联调。

软件急停与失能：

```bash
ros2 service call /agri_chassis_can/estop std_srvs/srv/Trigger '{}'
ros2 service call /agri_chassis_can/clear_estop std_srvs/srv/Trigger '{}'
ros2 service call /agri_chassis_can/disable std_srvs/srv/Trigger '{}'
```

## 里程计与 EKF

`agri_chassis_can` 查询 TD48150B A/B 两路实际转速，经轮径、减速比和有效履带中心距换算后发布 `/wheel/odometry`。

CAN 节点不发布 `odom -> base_link` TF。现有 `robot_localization` EKF 继续作为该 TF 的唯一发布者：

```text
/wheel/odometry + /camera/gyro_accel/sample
                    |
                    v
             /odometry/filtered
                    |
                    v
              odom -> base_link
```

这样底盘驱动、局部状态估计和上层 RTK/AOA/VSLAM 的职责保持分离。
