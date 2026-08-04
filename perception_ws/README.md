# 感知工作空间

## 职责

本工作空间负责 RGB-D VSLAM、深度点云、深度转二维障碍、树干检测和局部感知。相机硬件驱动在 `camera_ws`，地图与 Nav2 在 `navigation_ws`。

## ROS 2 包

- `agri_vslam_bringup`：RTAB-Map、相机内置 IMU 校正、点云和 fake `/scan`。
- `trunk_detection`：YOLO 树干检测、树行拟合和数据采集。
- `lidar_processor`：保留的通用障碍处理节点，不包含 YDLidar/Livox 驱动。

## 构建

```bash
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source /home/czb/agri_robot_system/chassis_ws/install/setup.bash
cd /home/czb/agri_robot_system/perception_ws
colcon build --symlink-install
```

## Camera-only VSLAM

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_stable_scan.launch.py
```

检查：

```bash
ros2 topic hz /rtabmap/odom
ros2 topic echo /rtabmap/map --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map bb_robot/base_link
```

Camera-only 只用于相机、VO、建图和障碍输入验证。正式导航必须切换到 EKF 管理 `odom -> base_link` 的 formal 模式。

## 树干检测

模型已经安装到 `trunk_detection` 包的 `share/models/best.pt`，不再依赖主目录绝对路径：

```bash
ros2 launch trunk_detection trunk_detection.launch.py \
  image_topic:=/camera/color/image_raw \
  model_path:=best.pt
```

## 输出接口

```text
/rtabmap/odom
/rtabmap/map
/rtabmap/cloud_map
/scan
/trunk_detection/detections
/trunk_detection/trunk_line
/trunk_detection/detection_image
```
