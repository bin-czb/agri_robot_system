# 底盘工作空间

本工作空间保留机器人描述、公共 ROS 2 接口、EKF 和 Gazebo 仿真；**实车底盘控制只使用 `agri_chassis_can`**，对应 TD48150B-2E 双驱伺服控制器。旧串口/Modbus 和旧 CAN 控制包已从源码移除。

## 实车信息流

```text
Nav2 controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
                                                               |
手柄 -> joy_node -> /joy --------------------------------------|-> chassis_mode_teleop
                                                                    -> /chassis/cmd_vel
                                                                    -> agri_chassis_can
                                                                    -> TD48150B-2E

TD48150B-2E 转速反馈 -> /wheel/odometry -> robot_localization EKF
                                            -> /odometry/filtered 与 odom -> base_link
```

`chassis_mode_teleop` 是自动/手动速度的唯一选择点；CAN 驱动只订阅 `/chassis/cmd_vel`。实车驱动不发布 `odom -> base_link`，由 EKF 独占该 TF。仿真启动文件和 Gazebo 模型保留，其 `/cmd_vel` 桥接只用于仿真，不应与实车底盘启动文件同时运行。

## 构建与安全联调

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch agri_chassis_can chassis_bringup.launch.py listen_only:=true start_joy:=false
```

首次联调保持 `listen_only:=true`，不发送 CAN 控制帧。先核对真实 CAN ID、通道 A/B 与左右履带对应、方向、反馈单位、减速比、有效轮径、等效轮距和手柄映射；确认前不要切换为主动控制。TD48150B 参数、话题、手柄测试和分阶段验证步骤见 [新底盘包说明](src/agri_chassis_can/README.md)，实际参数在 [`td48150b.yaml`](src/agri_chassis_can/config/td48150b.yaml)。

## 其他包

- `trunk_interfaces`：树干、AOA 等共享消息接口。
- `agri_robot_description`：机器人模型与传感器安装 TF。
- `agri_robot_bringup`：EKF 和仿真启动。
- `trunk_gazebo_worlds`：果园仿真场景。

新底盘接入不会自动补齐导航路径跟踪、RTK/AOA 坐标标定或实车安全验证；这些仍需分别完成。
