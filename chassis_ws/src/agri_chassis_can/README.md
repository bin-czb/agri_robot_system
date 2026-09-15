# agri_chassis_can

`agri_chassis_can` 是 `agri_robot_system/chassis_ws` 内部为新 TD48150B-2E 双驱伺服底盘建立的 ROS 2 Humble 实车驱动包。

本包只负责：

```text
ROS 2 速度指令
    -> 差速/滑移转向运动学
    -> TD48150B-2E CAN 扩展帧
    -> A/B 两路电机

TD48150B-2E 实际转速反馈
    -> 左右履带速度
    -> 轮式里程计
    -> /wheel/odometry
```

本包**不修改**定位、感知、Nav2、RTK、AOA、VSLAM 等工作空间，也**不发布** `odom -> base_link` TF。现有 `robot_localization` EKF 继续作为该 TF 的唯一发布者。

---

## 1. 与 Taizhou 项目的关系

Taizhou 项目中已经实机调试过的底盘节点采用标准 ROS 2 `/cmd_vel` 输入、SocketCAN 通信、轮速反馈和里程计输出。新底盘包借鉴这条已经验证过的 ROS 层职责划分，但 TD48150B-2E 的 CAN 协议与 Taizhou 旧底盘协议不同，因此协议编码必须按 TD48150B-2E 说明书重新实现。

当前仓库中的 Taizhou GitHub 镜像可以确认以下内容：

```text
/chassis CAN 节点订阅标准 /cmd_vel
SocketCAN 负责底层 CAN 收发
实际轮速反馈用于里程计
底盘节点具有 cmd_vel 超时停车逻辑
```

### 1.1 手柄 AUTO/MANUAL 逻辑的重要说明

用户已明确要求：**手柄逻辑必须直接参照 Taizhou 已经调试通过的实现，不允许重新设计一套“功能相似”的代码。**

因此本分支已删除此前临时编写的 `chassis_mode_teleop.py`。原因不是功能目标改变，而是当前 GitHub 上的 Taizhou 镜像中没有找到 `/joy`、`sensor_msgs/Joy`、A 键切换 AUTO/MANUAL 的源文件；现有 Taizhou 上传清单也只明确包含底盘 CAN 与 odom，没有包含该手柄节点源码。

在找到 Taizhou **原始、已验证** 的手柄源码之前：

```text
不重新实现 A 键切换逻辑
不猜测按键编号
不猜测摇杆轴编号
不猜测模式切换时 Nav2 的暂停/恢复行为
不引入新的 cmd_vel 仲裁策略
```

待 Taizhou 原始手柄文件可用后，只在 `chassis_ws` 内进行适配：保留其已经验证的模式切换和 ROS 话题逻辑，仅把最终底盘执行端替换为本包的 TD48150B CAN 节点。

这意味着当前阶段的正式速度入口保持为：

```text
Nav2 /cmd_vel
      |
      v
agri_chassis_can
      |
      v
TD48150B-2E
```

后续加入 Taizhou 手柄逻辑时，**仍必须让手柄测试覆盖同一 ROS 2 -> 底盘节点 -> CAN 链路**，不能绕过 `agri_chassis_can` 直接发 CAN。

---

## 2. 当前目录结构

```text
chassis_ws/src/agri_chassis_can/
├── agri_chassis_can/
│   ├── __init__.py
│   ├── chassis_can_node.py
│   ├── differential_kinematics.py
│   ├── socketcan_transport.py
│   └── td48150b_protocol.py
├── config/
│   └── td48150b.yaml
├── launch/
│   └── chassis_bringup.launch.py
├── scripts/
│   └── setup_can.sh
├── test/
│   ├── test_differential_kinematics.py
│   └── test_td48150b_protocol.py
├── package.xml
├── setup.cfg
└── setup.py
```

所有新增底盘代码均位于 `chassis_ws` 内，没有向 `navigation_ws`、`perception_ws`、`rtk_ws`、`aoa_ws` 或 `camera_ws` 写入底盘实现。

---

## 3. TD48150B-2E 已由说明书确认的 CAN 事实

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

后 4 Byte 按有符号 32 bit 大端二补码编码，程序不手抄厂家文档中的负数十六进制示例。

转速查询：

```text
TX: 40 03 21 01 00 00 00 00
RX: 60 03 21 01 A_H A_L B_H B_L
```

故障查询：

```text
TX: 40 12 21 01 00 00 00 00
RX: 60 12 21 01 A_H A_L B_H B_L
```

另外还实现了电流、母线电压与温度查询，供 `/diagnostics` 使用。

---

## 4. 未确认参数清单

**下表中的任何“未确认”项目，在实车确认前都不能当成最终标定值。** `config/td48150b.yaml` 中也逐项保留了同样的 `UNCONFIRMED` 注释。

| 参数 | 当前临时值 | 状态 | 来源/原因 | 实车确认方式 |
|---|---:|---|---|---|
| `command_can_id` | `0x06000001` | 未确认 | 说明书地址 1 示例 | `candump can0` + 只监听确认 |
| `feedback_can_id` | `0x05800001` | 未确认 | 按说明书反馈 ID 规则推导 | 发非运动查询后核对实际返回 |
| `heartbeat_can_id` | `0x07000001` | 未确认 | 按说明书心跳 ID 规则推导 | 驱动器上电、不使能，观察约 1 Hz 扩展帧 |
| 驱动器地址 | `1` | 未确认本机 | 厂家出厂默认值 | 上位机设置/实机 CAN ID 反推 |
| `left_channel` | `A` | 未确认 | 仅临时假设 | 履带架空，单路低速点动 |
| `right_channel` | `B` | 未确认 | 仅临时假设 | 履带架空，单路低速点动 |
| 左控制符号 | `+1` | 未确认 | 电机安装方向未知 | 单路低速正指令，看车辆前进方向 |
| 右控制符号 | `+1` | 未确认 | 左右电机通常镜像安装 | 同上 |
| 左反馈符号 | `+1` | 未确认 | 编码器方向未知 | 正向点动并检查反馈符号 |
| 右反馈符号 | `+1` | 未确认 | 编码器方向未知 | 同上 |
| `wheel_radius_m` | `0.20` | 未确认 | 当前模型/设计估计 | 直线行驶实测距离标定有效半径 |
| `track_width_m` | `1.22916` | 未确认 | 当前 URDF 两履带 joint y 坐标之差 | 原地/定半径转向标定滑移等效轮距 |
| `gear_ratio` | `1.0` | 未确认 | 当前没有可靠传动比资料 | 机械资料 + 电机/驱动轮转速实测 |
| `driver_max_rpm` | `3000` | 未确认本机 | 说明书仅用 3000 rpm 举例 | 驱动器上位机参数/铭牌确认 |
| CAN 转速反馈单位 | `normalized` | 未最终确认 | 串口查询示例强烈表明 1000=额定转速 10%；CAN 页未明确写单位 | 架空轮给定已知标幺值并比较反馈 |
| 里程计协方差 | 代码内为初始保守值 | 未标定 | 尚无重复实验统计 | 多次直线/转向试验后按误差统计更新 |
| 最大线/角速度 | `0.10 m/s`, `0.30 rad/s` | 仅联调限制 | 安全起步值 | 底盘确认后按机械能力和 Nav2 需求调整 |

### 4.1 为什么 `track_width_m=1.22916` 不能直接当真值

当前 URDF 中左右履带 joint 的横向位置约为：

```text
left:  +0.61468 m
right: -0.61448 m
```

几何中心距约为：

```text
1.22916 m
```

但履带/滑移转向车辆在转弯时存在明显横向滑移，因此轮式里程计需要的是**等效运动学轮距**，不一定等于 CAD/URDF 几何中心距。

### 4.2 为什么转速反馈暂用 `normalized`

TD48150B 说明书另一查询示例中，额定转速 3000 rpm 时：

```text
raw = 0x03E8 = 1000
实际转速 = 300 rpm
```

即 raw=1000 对应额定转速的 10%。这与 `-10000..10000` 标幺定义一致。但 CAN 查询章节只写“实际转速值”，没有明确写出单位。因此代码支持：

```text
normalized
rpm
```

两种解析模式，当前 YAML 临时选择 `normalized`，仍要求实机验证。

---

## 5. ROS 2 接口与原系统边界

正式输入：

```text
/cmd_vel                 geometry_msgs/msg/Twist
```

底盘反馈：

```text
/wheel/odometry          nav_msgs/msg/Odometry
/chassis/left_motor_rpm  std_msgs/msg/Float32
/chassis/right_motor_rpm std_msgs/msg/Float32
/chassis/fault_a         std_msgs/msg/Int32
/chassis/fault_b         std_msgs/msg/Int32
/chassis/can/state       std_msgs/msg/String
/chassis/can/raw_rx      std_msgs/msg/String
/chassis/can/raw_tx      std_msgs/msg/String
/diagnostics             diagnostic_msgs/msg/DiagnosticArray
```

控制服务：

```text
/agri_chassis_can/enable
/agri_chassis_can/disable
/agri_chassis_can/estop
/agri_chassis_can/clear_estop
```

### 5.1 TF 所有权不改变

当前链路保持：

```text
TD48150B 实际轮速
       |
       v
/wheel/odometry
       |
       +------ camera IMU
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

`agri_chassis_can` 不发布 `odom -> base_link`，避免与现有 EKF 抢 TF。

### 5.2 不改上层系统

本分支不修改：

```text
navigation_ws
perception_ws
rtk_ws
aoa_ws
camera_ws
```

也不删除旧的 `chassis_control` / `agri_chassis_serial`。旧串口底盘代码仍保留在原位置，便于历史回溯；新 TD48150B 包作为独立包存在，不与旧链路同时启动。

---

## 6. 构建

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agri_chassis_can
source install/setup.bash
```

协议与运动学测试：

```bash
colcon test --packages-select agri_chassis_can
colcon test-result --verbose
```

---

## 7. 第一次实车联调顺序

### 阶段 0：绝对不动车

先安装 `can-utils`：

```bash
sudo apt install can-utils
```

配置默认 250 kbit/s：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_chassis_can setup_can.sh can0 250000
```

只看 CAN：

```bash
candump can0
```

此时不要使能驱动器，不要发送速度。

### 阶段 1：ROS 节点只监听

```bash
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true
```

检查：

```bash
ros2 topic echo /chassis/can/raw_rx
ros2 topic echo /diagnostics
```

首先确认：

```text
实际心跳 ID
是否为扩展帧
心跳周期
```

### 阶段 2：只做非运动查询

在确认真实 CAN ID 后，才能考虑关闭 `listen_only`。第一次主动发送应先验证状态/故障/转速查询响应，不立即给非零速度。

### 阶段 3：履带架空，单路低速点动

确认：

```text
A/B -> 左/右
控制正负号
反馈正负号
反馈单位
driver_max_rpm
```

### 阶段 4：直线标定

确认：

```text
gear_ratio
wheel_radius_m
直线 /wheel/odometry 比例
```

### 阶段 5：转向标定

通过原地转向和定半径转向确定：

```text
track_width_m（等效值）
角速度比例
里程计 yaw 漂移
```

### 阶段 6：接 EKF

确认 `/wheel/odometry` 与 IMU 能稳定生成现有：

```text
/odometry/filtered
odom -> base_link
```

### 阶段 7：最后接 Nav2 和 Taizhou 手柄逻辑

只有底盘、里程计、EKF 均正确后，才允许接自主导航。手柄节点必须从 Taizhou 已验证源码移植，不在本包中重新发明模式切换逻辑。

---

## 8. 安全机制

当前 CAN 节点包含：

```text
默认 listen_only=true
无新 /cmd_vel 时超时发送 0 速度
固定周期控制，避免依赖上层消息恰好小于驱动器 1000 ms 看门狗
使能前可要求先收到心跳
心跳超时后停止并失能
软件 E-stop 锁存
节点退出前发送 0 速度并失能
速度命令限制在 -10000..10000
左右轮同时按比例饱和，避免破坏期望转弯曲率
```

软件急停不是硬件急停的替代品。首次实机测试仍应具备可直接切断驱动动力/使能的物理安全手段。

---

## 9. 当前开发状态

已完成：

```text
TD48150B 纯协议编码/解析
29 bit SocketCAN 收发
/cmd_vel -> 差速运动学 -> A/B 速度命令
速度/故障/电流/电压/温度查询
实际轮速 -> /wheel/odometry
listen-only 与软件安全状态
协议/运动学单元测试
```

尚未宣称完成：

```text
真实 CAN ID 确认
A/B 左右映射确认
电机方向确认
反馈单位确认
实际减速比/有效轮径/等效轮距标定
里程计协方差统计标定
Taizhou 原始 A 键 AUTO/MANUAL 手柄节点移植
实车 Nav2 闭环验证
```

在以上项目完成前，本包属于**实车联调分支**，不能视为最终量产/正式导航配置。
