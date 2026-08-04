# 农业导航正射地图集成包

`agri_map_integration`负责把正射影像处理得到的Nav2占据地图、树木坐标、
RTK定位和RTAB-Map结果放进同一套RViz视图。

当前地图是测试版本。地图坐标系为`EPSG:32651`，ROS的`map`坐标系采用
东、北、天方向，正射影像左下角为`map(0, 0)`。

## 当前接口

```text
/map                    Nav2静态占据地图
/orthophoto/cloud       可选彩色正射影像，仅用于地图校核
/tree_markers           树木、编号和禁入半径
/global_pose/rtk        RTK当前位置和协方差
/rtk/marker             RTK当前位置标记
/rtk/path               RTK移动轨迹
/rtabmap/mapData        RTAB-Map三维点云
/rtabmap/mapPath        RTAB-Map估计轨迹
/rtabmap/grid_map       RTAB-Map局部占据栅格
/rtabmap/occupied_cells 无未知背景的VSLAM障碍栅格，仅用于RViz
/global_costmap/costmap Nav2全局代价地图
/local_costmap/costmap  Nav2局部滚动代价地图
/plan                   Nav2全局规划路径
/local_plan             Nav2局部控制路径
/map_alignment/status   正射地图与VSLAM地图配准状态
```

TF按照以下职责划分：

```text
map
└── vslam_map                   RViz手动初始配准
    └── bb_robot/odom           RTAB-Map camera-only测试模式
        └── bb_robot/base_link  视觉里程计
```

RTK定位消息直接使用`map`作为`frame_id`，不会额外发布TF。

## RViz二维导航主视图

统一RViz当前以二维占据栅格和规划结果为主，不再默认渲染RTAB-Map三维
`MapCloud`。默认显示层的职责如下：

```text
/map
  空中建图生成的全局静态占据地图，白色可通行、黑色禁止通行

/rtabmap/grid_map
  VSLAM地图坐标系下实时更新的占据栅格，由局部深度感知逐步累积

/rtabmap/occupied_cells
  从RTAB-Map栅格提取的已占据单元，不包含未知和空闲单元

/local_costmap/costmap
  Nav2将实时障碍、机器人足迹和膨胀半径组合后的局部导航代价地图

/plan
  Nav2规划器给出的全局参考路径

/local_plan
  Nav2控制器当前实际采用的短距离局部路径
```

RViz中的`VSLAM三维点云`仍然保留，但默认关闭，需要诊断三维重建时可以
手动勾选。`Nav2全局代价地图`和`Nav2局部代价地图`也默认关闭，避免透明
代价地图覆盖黑白静态地图。局部路径和机器人足迹仍默认开启，Nav2发布
数据后可以直接显示。

RViz标准`Map`显示会把`OccupancyGrid`中的`-1`未知单元画成灰色，因此会
看到包围相机的矩形地图边界。统一入口现在启动
`occupancy_grid_overlay`，将RTAB-Map中占据值不小于`50`的单元转换为
`nav_msgs/GridCells`。RViz默认显示`VSLAM实时障碍栅格`，同时关闭
`VSLAM完整栅格（含未知区）`，从而保留实时障碍而不再绘制灰色方框。
该转换只用于可视化，不修改`/rtabmap/grid_map`，也不参与Nav2规划。

统一RViz默认增加`RGB实时图像`面板，订阅：

```text
/camera/color/image_raw
```

RTK默认只显示`/rtk/marker`定位点和`/rtk/path`轨迹。带协方差的
`/global_pose/rtk`显示项仍然保留，但默认关闭，因此不会再绘制过大的
不确定度圆。需要检查RTK协方差时，可以在显示列表中临时勾选
`RTK当前位置与协方差`。

只启动地图和VSLAM时，可以看到`/map`和从`/rtabmap/grid_map`提取的
`/rtabmap/occupied_cells`，但不会产生`/local_costmap/costmap`或
`/local_plan`。后两项只有在Nav2、连续里程计和完整TF链已经启动后才会
发布，因此“看到VSLAM障碍栅格”不等同于“已经完成局部路径规划”。

## 地图定义

- 地理母图：`maps/orthophoto.tif`，默认不进入导航RViz
- Nav2地图：`maps/navigation_map.png`
- Nav2参数：`config/navigation_map.yaml`
- 地理参考：`config/map_georeference.yaml`
- 树木坐标：`maps/tree_landmarks.csv`

占据语义已经由用户确认：

```text
白色：可通行
黑色：障碍、禁行区或地图外区域
```

正射影像左下角：

```text
UTM Easting:  244721.8491
UTM Northing: 3356945.1346
WGS84纬度:    30.3174149693
WGS84经度:    120.3451741109
```

## 编译

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash

colcon build --symlink-install --packages-select \
  agri_vslam_bringup agri_rtk_localization agri_map_integration

source install/setup.bash
```

## 每个终端的环境加载

不要在Conda的`base`环境中运行ROS 2。每打开一个新终端，先执行：

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash
```

RTK驱动终端不依赖Orbbec，但统一使用以上环境也不会产生冲突。

## 完整传感器启动

### RTK全局定位模式

RTK和AOA是互斥的全局定位源。该模式不要启动AOA。

终端1启动UM982、NTRIP和原始GNSS话题：

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 run agri_rtk_localization start_um982_ntrip
```

终端2启动以下组件：

```text
Gemini 335L RGB和注册深度
Orbbec同步原始IMU（200 Hz）
IMU六面标定校正和Madgwick
RTAB-Map camera-only VSLAM
深度图转/scan
正射处理占据地图和树干
RTK地图坐标转换
统一RViz
```

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 launch agri_map_integration integrated_mapping.launch.py \
  start_vslam:=true \
  start_calibrated_imu:=true \
  start_rtk_localizer:=true \
  start_rviz:=true
```

### AOA全局定位模式

该模式不要启动UM982/NTRIP，并关闭统一入口中的RTK适配器。终端1启动
地图、相机、IMU和VSLAM：

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 launch agri_map_integration integrated_mapping.launch.py \
  start_vslam:=true \
  start_calibrated_imu:=true \
  start_rtk_localizer:=false \
  start_rviz:=true
```

终端2启动AOA串口、无人机MAVLink位置、UTM到map转换和导航侧抗跳变滤波：

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 launch agri_global_localization aoa_global_localization.launch.py \
  aoa_serial_port:=/dev/ttyACM0 \
  mavlink_port:=/dev/ttyUSB0 \
  heading_mode:=imu_relative \
  initial_heading_enu_deg:=0.0 \
  use_odometry:=false
```

`initial_heading_enu_deg`必须替换为启动时相机正前方的ENU绝对航向：东为
`0`，北为`90`。AOA现在使用`/ground_station/absolute_pose_utm_cov`和当前
正射地图的EPSG:32651左下角原点，输出`/global_pose/aoa_raw_map`和滤波后
的`/global_pose/aoa`，二者的`frame_id`均为`map`。该转换只提供全局位姿
消息，不发布`map -> odom`，因此还不能单独构成Nav2正式定位TF链。

### 单独测试相机、深度和标定IMU

以下命令会独占相机，运行前先停止统一入口：

```bash
conda deactivate 2>/dev/null || true
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash

ros2 run orbbec_camera list_devices_node
ros2 launch agri_vslam_bringup orbbec_calibrated_imu.launch.py
```

在上述标定IMU链保持运行时，另开一个已加载相同环境的终端查看IMU。
关闭相机和滤波器启动开关，避免重复打开设备或重复发布：

```bash
ros2 launch agri_vslam_bringup orbbec_imu_rviz.launch.py \
  start_camera:=false \
  start_filter:=false
```

单独查看深度点云轮廓：

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_cloud.launch.py
```

另开一个已加载相同环境的终端：

```bash
ros2 launch agri_vslam_bringup rviz_depth_points.launch.py
```

## 只查看地图

此模式不需要相机和RTK：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agri_map_integration map_visualization.launch.py \
  start_rtk_localizer:=false \
  start_vslam_aligner:=false
```

应当看到处理后的黑白占据地图、树木标记和禁入半径。彩色正射影像作为
地图母文件保留，但默认不发布，也不进入导航RViz界面。

需要单独校核原始正射图和占据地图的几何关系时，可以临时发布彩色正射
影像话题：

```bash
ros2 launch agri_map_integration map_visualization.launch.py \
  start_rtk_localizer:=false \
  start_vslam_aligner:=false \
  start_orthophoto:=true \
  orthophoto_pixel_stride:=1
```

导航统一RViz仍只显示占据地图，不显示彩色话题。

## 查看地图和RTK

终端1启动UM982和NTRIP：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run agri_rtk_localization start_um982_ntrip
```

终端2启动地图、RTK转换和RViz：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agri_map_integration map_visualization.launch.py \
  start_rtk_localizer:=true \
  start_vslam_aligner:=false
```

只有`/rtk/fix_quality=4`时，地图上的RTK点和轨迹才会更新。检查：

```bash
ros2 topic echo /global_pose/rtk --once
ros2 topic echo /rtk/path --once
ros2 topic echo /rtk/diagnostics --once
```

## 地图、RTK和VSLAM统一启动

相机和RTK测试设备应近似刚性固定，并随同一个载体移动。

终端1先启动UM982/NTRIP：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agri_rtk_localization start_um982_ntrip
```

终端2启动相机、RTAB-Map、地图、RTK转换和RViz：

```bash
cd /home/czb/pythonProject01/trunk_tracking_ws
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source install/setup.bash

ros2 launch agri_map_integration integrated_mapping.launch.py
```

统一入口只保留一条IMU处理链：

```text
/camera/gyro_accel/sample（Orbbec同步原始IMU，200 Hz）
  -> 六面标定偏置/增益/协方差校正
  -> /camera/imu/calibrated_raw
  -> Madgwick无磁力计姿态
  -> /camera/imu/data
```

不要再并行启动`orbbec_calibrated_imu.launch.py`或
`orbbec_imu_filter.launch.py`，否则会重复发布`/camera/imu/data`。

### 初始配准

1. 等待相机画面、`/rtabmap/odom`和`/rtabmap/mapData`稳定发布。
2. 在RViz工具栏选择`2D Pose Estimate`。
3. 在正射影像上点击相机当前所在位置。
4. 按住鼠标向相机正前方拖动，箭头表示相机/载体的正前方。
5. 松开鼠标后，系统计算并发布唯一的`map -> vslam_map`。

检查配准状态：

```bash
ros2 topic echo /map_alignment/status --once \
  --qos-durability transient_local
ros2 run tf2_ros tf2_echo map vslam_map
```

状态应以`ALIGNED`开头。若位置或方向明显错误，重新使用
`2D Pose Estimate`即可覆盖本次测试配准。

## 设计约束

- `/map`只由Nav2静态地图服务器发布。
- RTAB-Map的二维栅格改为`/rtabmap/grid_map`，不再与`/map`冲突。
- RTAB-Map使用`vslam_map`，必须通过配准后才能正确叠加到正射地图。
- 测试阶段的手动配准不等同于生产标定。
- 正式安装后仍需测量`gps_link -> bb_robot/base_link`和相机外参。
- 当前地图没有航测精度报告和正式高程基准，只能作为集成测试地图。
- 没有底盘时可以查看地图、RTK和VSLAM，但不能验证Nav2控制闭环。

## 常用检查

```bash
ros2 topic echo /map --once --qos-durability transient_local
ros2 topic echo /orthophoto/cloud --once \
  --qos-durability transient_local --field width
ros2 topic echo /tree_markers --once --qos-durability transient_local
ros2 topic hz /rtabmap/odom
ros2 topic echo /global_pose/rtk --once
ros2 run tf2_ros tf2_echo map bb_robot/base_link
```

如果只看到正射影像而看不到VSLAM，先检查
`/map_alignment/status`，通常是尚未执行`2D Pose Estimate`，或
`vslam_map -> bb_robot/base_link`尚未由RTAB-Map建立。
