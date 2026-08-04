# RTK 工作空间

## 职责

本工作空间包含 UM982 串口驱动、NTRIP 差分接入、RTK 自定义消息、经纬度输出、地图坐标转换和 RViz 轨迹。

## ROS 2 包

- `rtk_interfaces`：完整 RTK 状态消息。
- `um982_ntrip`：UM982 串口、NMEA/RTCM 与 NTRIP 客户端。
- `agri_rtk_localization`：经纬度到地图坐标转换、质量门限、滤波和轨迹。

## 构建

```bash
cd /home/czb/agri_robot_system/rtk_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## NTRIP 凭据

密码只保存在 `~/.config/agri_rtk/ntrip.env`，权限必须是 `600`：

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_rtk_localization configure_ntrip_credentials
```

## 统一启动

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_rtk_localization start_um982_ntrip
```

该命令同时启动 UM982/NTRIP 和 RTK 地图定位。只启动 ROS launch 时，必须先自行加载 `NTRIP_USERNAME` 和 `NTRIP_PASSWORD`。

## 检查 Fix 和坐标

```bash
ros2 topic echo /fix --once
ros2 topic echo /rtk/fix --once
ros2 topic echo /global_pose/rtk --once
ros2 topic echo /global_pose/rtk/status
ros2 topic hz /global_pose/rtk
```

`sensor_msgs/NavSatFix.status.status = 2` 表示 GBAS/RTK 级增强状态，但最终是否为固定解应同时检查驱动日志或 `/rtk/fix` 中的 RTK 状态，不能只根据协方差判断。

RTK 模式与 AOA 模式互斥，二者不能同时发布 `/global_pose/selected`。
