# AOA 全局定位适配包

本包把现有AOA解算结果接入农业导航系统，负责UTM到正射地图`map`坐标
的转换，并在导航入口处增加第二级鲁棒滤波。当前AOA和RTK仍按互斥模式
启动。

## 当前数据链

```text
无人机位置 + AOA距离/方位 + 相机IMU相对航向
                    |
                    v
/ground_station/absolute_pose_utm_cov（地面base_link，UTM）
                    |
          UTM原点平移 + 地图yaw旋转
                    |
                    v
        /global_pose/aoa_raw_map（map）
                    |
  5帧一致初始化 + 运动学/协方差创新门控
        + 位置/航向平滑 + 健康状态
                    |
                    +--> /global_pose/aoa
                    +--> /global_pose/selected
```

AOA原节点已经包含距离/角度EMA、角度和距离突变门限、前后向分支判定。
本包再对最终坐标做独立门控，避免单次错误解算直接污染上层定位。超过
门限的数据只累计为重定位候选，不会因连续出现而自动切换输出。

`aoa_map_transform`直接使用AOA源发布的绝对UTM约束，不依赖每次启动可能
变化的`local_origin`。当前测试地图参数为：

```text
CRS: EPSG:32651
UTM原点: E=244721.8491, N=3356945.1346
map轴: x向东，y向北，yaw=0
```

转换关系为：

```text
p_map = Rz(-map_yaw) * (p_utm - p_utm_origin)
yaw_map = yaw_utm - map_yaw
```

位置、航向和完整`6x6`位姿协方差都会转换到`map`。当前为二维导航，输出
高度固定为`0`。本包不发布TF；`map -> odom`仍应由后续全局融合节点唯一
发布。

## 启动

先启动 Orbbec 相机及 IMU 姿态滤波，确保 `/camera/imu/data` 有数据，再运行：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source install/setup.bash

ros2 launch agri_global_localization aoa_global_localization.launch.py \
  aoa_serial_port:=/dev/ttyACM0 \
  mavlink_port:=/dev/ttyUSB0 \
  heading_mode:=imu_relative \
  initial_heading_enu_deg:=0.0 \
  use_odometry:=false
```

`initial_heading_enu_deg` 是启动时相机正前方在 ENU 中的绝对朝向：东为 `0`，北为 `90`。相机 IMU 没有磁力计，只能保持相对航向，不能自行确定这个初始绝对值。

该入口明确使用 AOA 原始软件参数：方位偏置 `0`、符号 `+1`、距离偏置 `0`、比例 `1`，不会加载当前标定记录。

启动前必须先构建 `trunk_interfaces`，因为 AOA 源节点会发布带时间戳的 `/aoa/observation`：

```bash
colcon build --symlink-install \
  --packages-select trunk_interfaces agri_global_localization
```

## 检查

```bash
ros2 topic echo /ground_station/absolute_pose_utm_cov --once
ros2 topic echo /global_pose/aoa_raw_map --once
ros2 topic echo /global_pose/aoa --once
ros2 topic echo /global_pose/selected --once
ros2 topic echo /global_pose/aoa/status
ros2 topic hz /global_pose/selected
ros2 topic echo /aoa/observation --once
```

以下三个输出的`header.frame_id`必须分别为：

```text
/ground_station/absolute_pose_utm_cov  -> utm
/global_pose/aoa_raw_map              -> map
/global_pose/aoa                      -> map
```

若AOA定位点位于当前正射地图范围内，`/global_pose/aoa_raw_map`的`x`应在
约`0~20.95 m`、`y`应在约`0~29.90 m`。超出范围不一定是程序错误，也可能
表示传感器实际位于测试地图外；应结合原始UTM坐标判断。

状态中的 `rejected` 增长且 `last_reason=position_jump`，表示坐标跳变已被导航侧门控拒绝。`health` 状态为：

```text
WAITING：等待5帧一致初始化
NORMAL：定位正常
DEGRADED：连续异常或有效校正超过1秒未更新
LOST：有效校正超过3秒未更新
```

当前普通更新门限为 `max_jump_m=0.75`、`max_speed_mps=2.0`、`jump_slack_m=0.20`，并同时执行 `3.5 sigma` 协方差创新检查。门限来自 2026-04 三包历史实验的离线分析，正式底盘实验后需要再次评估。

## 后续接入里程计

底盘和 EKF 可用后设置：

```bash
use_odometry:=true base_odometry_topic:=/odometry/filtered heading_mode:=odometry_relative
```

届时里程计负责高频、平滑的短时运动预测，AOA 负责低频全局位置修正。此互补接口已经实现，但正式导航前仍需用实车数据调整速度门限、协方差，并评估是否升级为双 EKF 或因子图融合。

里程计模式还会在 AOA 源端同时评估 `theta` 与 `theta+pi` 两个位置候选，优先选择接近里程计运动预测的分支。局部 EKF 继续拥有 `odom -> base_link`；本包不发布 TF。

## 验证结果

- AOA滤波和UTM到map纯算法测试：10项通过。
- UAV 时间插值、AOA 协议和几何自测通过。
- ROS 图测试中，正常坐标约 `(1.04, 2.00)`；连续注入 4 帧约 9 m 跳点后，输出未跳变，状态变为 `DEGRADED`，重定位候选计数为 4。

### 历史 rosbag 兼容回放

```bash
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 run agri_global_localization aoa_bag_replay \
  /home/czb/air_ground_cooperation_system/experiments/aoa_exp_20260422_101302 \
  /home/czb/air_ground_cooperation_system/experiments/aoa_exp_20260422_221348 \
  /home/czb/air_ground_cooperation_system/experiments/aoa_exp_20260423_105618
```

当前门限回放结果：

| 实验 | 原始最大步长 | 滤波后最大已接受步长 | 跳点拒绝率 |
|---|---:|---:|---:|
| `20260422_101302` | 11.075 m | 0.123 m | 99.7% |
| `20260422_221348` | 20.492 m | 0.055 m | 98.5% |
| `20260423_105618` | 1.165 m | 0.173 m | 45.0% |

前两包会安全进入定位失效状态，不能作为可用导航源。第三包仍能持续更新，已接受更新的95%间隔不超过约`0.101 s`，最大间隔约`0.802 s`。以用户修订参考点`(-7,0)`计算，第三包原始RMSE约`0.992 m`，滤波后约`0.773 m`。

这是旧格式兼容回放：历史包没有`/aoa/observation`、协方差和底盘里程计，只能验证最终坐标防跳，不能验证新时间同步和动态融合。完整验收仍需用修改后的系统重新录制静态与移动bag。
