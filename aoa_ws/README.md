# AOA 工作空间

## 职责

本工作空间负责接收 AOA 串口测量、飞行器 MAVLink/RTK 位置、计算地面刚性组件的 UTM 坐标，并转换和滤波为 `map` 坐标。

## ROS 2 包

- `agri_aoa_driver`：AOA 串口节点、飞行器位置节点、协议和几何计算。
- `agri_global_localization`：UTM 到 `map` 转换、鲁棒滤波和轨迹输出。

旧的普通 Python 脚本已封装为标准 `ament_python` 包，launch 不再引用 `/home/czb/pythonProject01`。

## 构建

AOA 消息依赖 `chassis_ws` 中的 `trunk_interfaces`：

```bash
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/chassis_ws/install/setup.bash
cd /home/czb/agri_robot_system/aoa_ws
colcon build --symlink-install
```

## 启动

先启动相机及滤波后的 IMU，再执行：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_global_localization aoa_global_localization.launch.py \
  aoa_serial_port:=/dev/ttyACM0 \
  mavlink_port:=/dev/ttyUSB0 \
  heading_mode:=imu_relative \
  initial_heading_enu_deg:=0.0 \
  use_odometry:=false \
  start_map_transform:=true \
  publish_selected:=true
```

ENU 航向定义：东 `0` 度、北 `90` 度、西 `180` 度、南 `-90` 度。`initial_heading_enu_deg` 必须与启动时刚性组件真实方向一致。

## 数据链

```text
/uav/utm_pose + /aoa/measurement + /camera/imu/data
  -> /ground_station/absolute_pose_utm_cov
  -> /global_pose/aoa_raw_map
  -> /global_pose/aoa
  -> /global_pose/selected
```

## 检查

```bash
ros2 topic echo /aoa/measurement --once
ros2 topic echo /ground_station/absolute_pose_utm_cov --once
ros2 topic echo /global_pose/aoa_raw_map --once
ros2 topic echo /global_pose/aoa --once
ros2 topic hz /global_pose/aoa
```

AOA 模式与 RTK 模式互斥。当前节点只发布地图坐标位置，不抢占 `map -> odom` TF；后续由统一定位融合器发布导航 TF。
