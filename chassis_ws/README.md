# 底盘工作空间

## 职责

本工作空间负责公共接口、机器人 URDF、底盘 Modbus/串口通信、速度控制、低速联调遥控、EKF 和 Gazebo 机器人环境。

## ROS 2 包

- `trunk_interfaces`：底盘、树干和 AOA 共用消息接口。
- `agri_chassis_serial`：不依赖绝对路径的共享 Modbus RTU 实现。
- `serial_bridge_ros2`：标准 `/cmd_vel` 到串口桥接。
- `chassis_control`：现有底盘协议控制、状态和联调遥控。
- `agri_robot_description`：机器人 URDF、传感器安装 TF。
- `agri_robot_bringup`：EKF、仿真底盘和统一局部定位入口。
- `trunk_gazebo_worlds`：果园 Gazebo 世界和机器人资源。

## 构建

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Gazebo 仿真

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_robot_bringup sim_bringup.launch.py
```

## 实车底盘通信

底盘到货并核对通信点表后才能执行：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch chassis_control chassis_control.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  slave_id:=1 \
  test_mode:=normal
```

## 低速联调遥控

```bash
ros2 launch chassis_control chassis_teleop.launch.py
```

另一个终端调用一次性服务：

```bash
ros2 service call /chassis_test/forward std_srvs/srv/Trigger '{}'
ros2 service call /chassis_test/turn_left std_srvs/srv/Trigger '{}'
ros2 service call /chassis_test/stop std_srvs/srv/Trigger '{}'
```

当前遥控只用于架空轮或空旷场地低速联调。正式遥控器、Nav2 和树行控制器接入后，必须增加 `/cmd_vel` 仲裁、急停和超时保护，禁止多个控制源直接同时驱动底盘。
