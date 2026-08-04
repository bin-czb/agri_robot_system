# 农业机器人 RTK 全局定位包

`agri_rtk_localization` 位于农业导航大目录内，负责把外部 UM982 驱动发布的
WGS-84 定位转换为导航可使用的本地 ENU 位姿，并提供标准 RViz 位置和移动
轨迹。它不负责串口和 NTRIP，也不直接控制底盘。

## 系统边界

```text
/home/czb/ROS_RTK/ros2_ws
  UM982串口 + NTRIP
      |
      +--> /fix                  sensor_msgs/NavSatFix
      +--> /rtk/fix_quality      std_msgs/UInt8
      |
      v
agri_rtk_localization
  RTK FIX质量检查
  WGS-84 -> 本地ENU
  自动/人工地理datum
  跳点门控和轻量平滑
      |
      +--> /global_pose/rtk      PoseWithCovarianceStamped
      +--> /global_pose/selected PoseWithCovarianceStamped
      +--> /odometry/rtk         Odometry
      +--> /rtk/path             Path
      +--> /rtk/marker           Marker
      +--> /rtk/datum            GeoPointStamped
      +--> /rtk/diagnostics      DiagnosticArray
```

本包默认不发布任何 TF。正式系统中的 TF 所有权保持：

```text
odom -> base_link：局部 EKF
map -> odom：全局融合/定位层
```

RTK 单天线只能提供全局位置，静止时不能提供可靠航向，也不能替代轮速里程计
和 IMU。`/odometry/rtk` 是全局位置测量及可视化接口，不是底盘局部里程计。

## 编译

本包只依赖标准 ROS 2 消息，不要求编译时 source 外部 RTK 工作空间：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select agri_rtk_localization

source install/setup.bash
```

## 启动硬件驱动

先查找稳定串口名称：

```bash
ls -l /dev/serial/by-id/
```

首次使用时安全保存 NTRIP 凭据。账号会正常显示，密码输入时终端不会回显：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agri_rtk_localization configure_ntrip_credentials
```

凭据保存在：

```text
~/.config/agri_rtk/ntrip.env
```

文件权限为 `600`，不会写入工作空间、YAML 或 launch 文件。配置一次后，终端 1
可直接自动加载凭据并启动 `/home/czb/ROS_RTK` 中的 UM982 驱动：

```bash
ros2 run agri_rtk_localization start_um982_ntrip
```

也可以继续使用手动环境变量方式：

```bash
cd /home/czb/ROS_RTK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export NTRIP_USERNAME='填写账号'
export NTRIP_PASSWORD='填写密码'

ros2 launch um982_ntrip um982_ntrip.launch.py
```

先确认原始输入：

```bash
ros2 topic echo /fix --once
ros2 topic echo /rtk/fix_quality --once
ros2 topic hz /fix
ros2 topic echo /rtk/status
```

当前 UM982 驱动包含 RTCM 停流看门狗。即使 TCP 仍显示连接，只要连续
10 秒没有收到新的 RTCM 数据，驱动也会报告：

```text
RTCM stream stalled for 10.0s
```

随后关闭失效连接并自动重连。判断链路健康时必须同时检查
`/rtk/rtcm_bytes` 持续增加且 `/rtk/correction_age` 保持在数秒内，不能只看
一次 `NTRIP connected` 日志。

新版驱动还会校验 RTCM3 CRC，并把消息类型统计发布到：

```bash
ros2 topic echo /rtk/rtcm_status
ros2 topic echo /rtk/receiver_response
```

当前地面平台启动时使用 `MODE ROVER SURVEY`，避免 UM982 默认 UAV 动态模型
影响低动态固定，并使用 `CONFIG PVTALG MULTI` 启用双频 PVT 解算。
`/rtk/rtcm_status` 应持续看到 `1005/1006` 和至少一个星座的
`107x/108x/109x/112x` 观测消息。若这些输入正常却长期为 `RTK_FLOAT`，再检查
ANT1 主天线、馈线、固件授权和频点配置，不要在定位适配层伪造 FIX。

正式导航默认只接受：

```text
/rtk/fix_quality = 4
RTK_FIXED
```

天线、馈线或 NTRIP 尚未正常时，不会向导航输出伪造的有效坐标。

## 启动定位与 RViz

终端 2：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agri_rtk_localization rtk_global_localization.launch.py
```

RViz 的 Fixed Frame 为 `local_origin`，显示：

- 绿色球体：当前 RTK FIX 位置。
- 绿色箭头及椭圆：当前位置、移动航向和位置协方差。
- 蓝色折线：有界保存的实际移动轨迹。
- 灰色网格：以 datum 为原点的 ENU 米制坐标。

只启动节点、不打开 RViz：

```bash
ros2 launch agri_rtk_localization rtk_global_localization.launch.py \
  start_rviz:=false
```

## 检查和维护

```bash
ros2 topic echo /global_pose/rtk --once
ros2 topic echo /global_pose/selected --once
ros2 topic echo /odometry/rtk --once
ros2 topic echo /rtk/datum --once
ros2 topic echo /rtk/diagnostics
ros2 topic hz /global_pose/rtk
```

清除移动轨迹：

```bash
ros2 service call /rtk_global_localizer/clear_path std_srvs/srv/Trigger {}
```

重新采集自动 datum：

```bash
ros2 service call /rtk_global_localizer/reset_datum std_srvs/srv/Trigger {}
```

## datum 模式

### 自动模式

默认 `datum_mode: auto`，只适合前期设备测试。节点等待 20 个稳定的
RTK FIX，要求位置簇半径不超过 0.20 m，然后将均值设为 ENU 原点。

每次重新启动得到的原点可能不同，因此自动模式不能用于跨天地图复用，也不能
直接对齐正射影像。

### 人工模式

接入带地理坐标的正射影像前，必须把
`config/rtk_localization.yaml` 改成：

```yaml
datum_mode: manual
datum_latitude: 30.000000000
datum_longitude: 114.000000000
datum_altitude: 0.0
```

这三个值必须来自正射地图坐标定义或经过测量的固定基准点。此时
`local_origin` 的 X 为东、Y 为北、Z 为上，单位为米。正射影像、树坐标、
规划路线、实际轨迹和 RTK 位置才能落在同一坐标口径中。

## AOA 与 RTK 互斥

AOA 和 RTK 后续是两种全局定位模式，不应同时发布
`/global_pose/selected`：

- RTK 模式：本包 `publish_selected:=true`，停止 AOA 全局定位入口。
- AOA 模式：本包不启动，或设置 `publish_selected:=false`。

## 后续接入导航

底盘可用后，推荐采用局部/全局分层融合：

```text
轮速 + IMU
    -> 局部EKF
    -> odom -> base_link

RTK全局位置 + 局部EKF + 已标定的gps_link外参
    -> robot_localization全局融合或等价因子图
    -> map -> odom

RTAB-Map
    -> 视觉相对约束/局部地图
    -> 与同一map地理坐标配准

Nav2
    -> 使用 map -> odom -> base_link
```

正式融合前必须测量 `base_link -> gps_link` 天线杆臂，并统一时间戳。不要把
本节点当前位置直接伪装成 `map -> base_link` TF。
