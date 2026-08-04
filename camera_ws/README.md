# 相机工作空间

## 职责

本工作空间只负责 Gemini 335L 的硬件驱动、ROS 2 消息、相机 TF 和标准启动入口。RTAB-Map、深度障碍和树干识别属于 `perception_ws`。

## ROS 2 包

- `orbbec_camera`：Orbbec 官方 ROS 2 驱动。
- `orbbec_camera_msgs`：相机自定义消息。
- `orbbec_description`：官方相机模型。
- `agri_camera_bringup`：本系统 Gemini 335L 参数和统一 launch。

官方仓库固定在提交 `fe738fed932fb6e1a82a83536d4481716338f25e`。本地默认点云和双目 IR 修改记录在 `patches/`，不要直接覆盖上游历史。

## 构建

```bash
cd /home/czb/agri_robot_system/camera_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动相机

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

## 检查

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/gyro_accel/sample
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

相机内置 IMU 的最高请求频率保持为当前已经验证稳定的 `200 Hz`。AOA 使用的航向输入应是滤波后的 `/camera/imu/data`，不能直接把原始加速度当作绝对航向。

## 标定工具

标定脚本位于 `tools/rig_calibration/`，接受结果统一保存在系统根目录 `data/calibration/`。Orbbec 出厂相机-IMU 外参和时间标定仍是权威值，六面法只修正 IMU 轴向零偏和增益。
