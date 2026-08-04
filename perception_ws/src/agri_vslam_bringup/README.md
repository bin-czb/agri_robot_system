# agri_vslam_bringup

## 一条命令启动实时建图与 RViz

`rviz_vslam_mapping.launch.py` 默认启动完整的 camera-only 验证链，而不再只
打开一个没有数据源的 RViz：

```text
Gemini 335L RGB-D + 内置 IMU
  -> RGB-D 视觉里程计
  -> RTAB-Map 实时建图
  -> 深度图生成临时 /scan
  -> RViz 地图、轨迹、当前位姿和 RGB 图像
```

启动：

```bash
cd /home/czb/agri_robot_system/perception_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch agri_vslam_bringup rviz_vslam_mapping.launch.py
```

如果完整 VSLAM 链已经在另一个终端运行，只附加 RViz：

```bash
ros2 launch agri_vslam_bringup rviz_vslam_mapping.launch.py \
  start_stack:=false rviz_delay:=0
```

RViz 默认延迟 8 秒打开，让相机与 RTAB-Map 先建立话题和 `map` TF。刚启动时
短暂灰屏属于初始化过程；若 10 秒后仍无图像，先检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /rtabmap/odom --once
```

`RTPS_TRANSPORT_SHM Error` 通常是 Fast DDS 共享内存端口冲突提示，不是本次
图像缺失的主因。本次无画面的直接原因是旧入口只启动了 RViz，且终端环境未能
发现外部 `orbbec_camera` 包。

### 避免相机被重复启动

同一台 Gemini 335L 同一时间只能由一个 Orbbec 相机容器占用。不要在
`orbbec_calibrated_imu.launch.py` 仍运行时，再启动默认
`start_stack:=true` 的完整建图入口。重复占用的典型日志是：

```text
Failed to initialize device uvc_open ... return res-6
```

遇到该错误时，回到旧相机 launch 的终端按 `Ctrl+C`，确认没有残留容器：

```bash
pgrep -af 'orbbec|component_container'
```

再重新执行统一建图命令。若相机与完整 VSLAM 链本来就在另一个终端正常运行，
只打开 RViz 时使用 `start_stack:=false`。

相机启动日志应显示 `Connection: USB3.x`。若显示 `USB2.x`，应改用电脑上的
USB 3.x 直连接口，避免 RGB、深度、点云和 IMU 同时传输时带宽不足。

## RGB-D 与 IMU 建图数据采集

本包提供 `record_rgbd_imu_bag.sh`，用于采集可供 RTAB-Map 离线建图和
视觉惯性算法复现实验使用的数据。脚本记录：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/camera/gyro_accel/sample
/tf
/tf_static
```

如果校正和姿态滤波节点已经启动，脚本还会自动记录：

```text
/camera/imu/calibrated_raw
/camera/imu/data
```

采集前先启动专用采集入口。它启动机器人静态 TF、轻量 RGB-D 相机、
原始同步 IMU、六面校正和 Madgwick 姿态滤波，但不启动 RTAB-Map：

```bash
cd /home/czb/agri_robot_system/perception_ws
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash

ros2 launch agri_vslam_bringup rgbd_imu_recording.launch.py
```

新开一个终端录制：

```bash
cd /home/czb/agri_robot_system/perception_ws
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash

ros2 run agri_vslam_bringup record_rgbd_imu_bag.sh --name indoor_mapping_01
```

脚本会等待并实际读取五个必需传感器话题。检查全部通过后才开始录制。
默认输出到 `~/agri_camera_bags/`，使用 zstd 文件压缩。采集结束时按一次
`Ctrl+C`，并等待脚本输出“采集完成”，不要直接关闭终端。

定时采集 120 秒：

```bash
ros2 run agri_vslam_bringup record_rgbd_imu_bag.sh \
  --name indoor_mapping_120s \
  --duration 120
```

指定输出目录：

```bash
ros2 run agri_vslam_bringup record_rgbd_imu_bag.sh \
  --output-dir /home/czb/data/rgbd_imu \
  --name orchard_row_01
```

采集后检查：

```bash
ros2 bag info /home/czb/agri_camera_bags/indoor_mapping_01
```

检查各话题消息数量不为零，并确认 RGB、深度约为 15 Hz，IMU 接近
200 Hz。移动相机采图时应缓慢、连续并保持场景纹理，避免快速旋转、
运动模糊、强逆光和长时间对着无纹理墙面。

This package brings up the real RGB-D visual SLAM chain.

The Orbbec SDK workspace is kept external at:

```text
/home/czb/agri_robot_system/camera_ws/src/OrbbecSDK_ROS2
```

Do not clone or rebuild it inside `trunk_tracking_ws`; source its existing `install/setup.bash` before launching this package.

## Role

This package owns:

- Gemini 335L camera startup through `orbbec_camera`.
- RTAB-Map RGB-D SLAM.
- Optional fake `/scan` generation from depth for later Nav2 local costmaps.

It does not own:

- `bb_robot/odom -> bb_robot/base_link`; that remains the EKF chain in `agri_robot_bringup`.
- Nav2 planner/controller configuration; that remains in `agri_nav2_config`.

## Environment

RTAB-Map is required for RGB-D SLAM:

```bash
sudo apt update
sudo apt install -y ros-humble-rtabmap ros-humble-rtabmap-ros
```

```bash
cd /home/czb/agri_robot_system/perception_ws
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash
```

## Camera Check

Always pass the device-level check before launching RTAB-Map or RViz. If the
camera is not enumerated, the later nodes will only report missing image topics.

Recommended clean environment:

```bash
cd /home/czb/agri_robot_system/perception_ws
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash
```

Device-level check:

```bash
ros2 run orbbec_camera list_devices_node
lsusb
```

If `list_devices_node` prints:

```text
Current found device(s): (0)
```

stop here. Do not start RTAB-Map, fake `/scan`, or RViz yet. Replug the Gemini
camera, prefer a direct USB 3.x port, wait a few seconds, and run the device
check again. The expected state is that the Orbbec SDK lists the camera before
any image topics are tested.

If `list_devices_node` reports `libusb_init failed` or cannot see the camera, install the Orbbec udev rules from the already-built SDK workspace:

```bash
cd /home/czb/agri_robot_system/camera_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then replug the camera.

List devices:

```bash
ros2 run orbbec_camera list_devices_node
```

Start Gemini 335L only:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini.launch.py
```

Start Gemini 335L with the built-in synchronized IMU enabled:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_imu.launch.py
```

This uses the Orbbec SDK synchronized IMU output:

```text
enable_sync_output_accel_gyro=true
enable_accel=true
enable_gyro=true
accel_rate=200hz
gyro_rate=200hz
/camera/gyro_accel/sample  sensor_msgs/msg/Imu
```

For Gemini 330 series, `200hz` is the official launch default and is the current stable system setting. The SDK accepts higher rate strings such as `500hz`, `1khz`, `2khz`, `4khz`, `8khz`, `16khz`, and `32khz`, but actual support is device-dependent. Raise the rate only after this check stays stable:

```bash
ros2 topic hz /camera/gyro_accel/sample
ros2 topic echo /camera/gyro_accel/sample --once
```

The synchronized IMU frame is:

```text
camera_accel_gyro_optical_frame
```

After the six-position calibration has produced an accepted
`~/.ros/agri_rig_calibration.yaml`, start the complete calibrated chain with one
command:

```bash
ros2 launch agri_vslam_bringup orbbec_calibrated_imu.launch.py
```

It owns this pipeline:

```text
/camera/gyro_accel/sample
  -> six-position bias/gain/covariance correction
  -> /camera/imu/calibrated_raw
  -> Madgwick without magnetometer
  -> /camera/imu/data
```

The corrector refuses old or rejected profiles. The launch delays correction and
filter startup for three seconds while the camera initializes. If the camera is
already owned by another launch, use `start_camera:=false`.

Verify data rather than waiting for per-frame console logs:

```bash
ros2 topic hz /camera/gyro_accel/sample
ros2 topic hz /camera/imu/calibrated_raw
ros2 topic hz /camera/imu/data
ros2 topic echo /camera/imu/data --once
```

All three topics should continue near the configured 200 Hz. The nodes are
normally silent after their startup message.

Accepted 2026-07-20 rig calibration:

```text
IMU maximum corrected gravity error: 0.00857 m/s^2
calibrated raw and Madgwick output: about 200 Hz, one publisher each
depth validation over 0.75-4.00 m: RMS 0.01201 m, max error 0.01468 m
```

The depth values are validation metadata only. RTAB-Map and the depth-derived
`/scan` continue to consume the Orbbec factory-calibrated registered depth image.
Do not add a second depth scale/bias correction unless a held-out repeatability
test proves that the physical measurement reference is the optical centre.

`orbbec_gemini_imu.launch.py` publishes a static TF from `camera_link` to this frame so downstream localization can transform the IMU measurement.

To inspect the IMU orientation and motion vectors in RViz, use the all-in-one diagnostic launch:

```bash
ros2 launch agri_vslam_bringup orbbec_imu_rviz.launch.py
```

It starts the synchronized Orbbec IMU, runs `imu_filter_madgwick` without a magnetometer, and opens RViz with:

```text
/camera/gyro_accel/sample  raw acceleration and angular velocity
/camera/imu/data           diagnostic Madgwick orientation
/camera/imu/pose           RViz orientation axes
/camera/imu/markers        acceleration and angular-velocity arrows
```

The Madgwick node has `publish_tf=false`, so this diagnostic view does not compete with EKF, RTAB-Map, or navigation TF publishers. Without a magnetometer, roll and pitch are gravity-corrected but yaw can drift; this view is for sensor validation, not an absolute heading source.

If the camera driver is already running, avoid opening the device twice:

```bash
ros2 launch agri_vslam_bringup orbbec_imu_rviz.launch.py start_camera:=false
```

如果校正后的`/camera/imu/data`也已经由统一入口或
`orbbec_calibrated_imu.launch.py`发布，则相机和Madgwick都不要重复启动：

```bash
ros2 launch agri_vslam_bringup orbbec_imu_rviz.launch.py \
  start_camera:=false \
  start_filter:=false
```

For a 3D contour check, use the dedicated cloud profile:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_cloud.launch.py
```

This profile follows the more stable Orbbec raw depth point-cloud path:

```text
enable_point_cloud=true
enable_colored_point_cloud=false
device_preset=High Density
/camera/depth/points
```

It keeps the stream at 640x480 / 15 FPS, disables the left/right IR image streams, keeps decimation off, and enables spatial/temporal/hole-filling depth filters. Use this profile when you need to judge whether the camera can see usable 3D contours of nearby objects.

The Orbbec SDK also supports colored registered point cloud:

```text
depth_registration=true
enable_colored_point_cloud=true
/camera/depth_registered/points
```

Use it only when you need color on the 3D cloud. It is heavier than `/camera/depth/points` and can be less responsive on this laptop.

For a raw lightweight point-cloud preview on this laptop, use the fast camera profile:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_fast.launch.py
```

The fast profile keeps depth point cloud enabled, but reduces the load by using 640x480 at 15 FPS, disabling left/right IR image streams, and disabling colored point-cloud generation. It is useful for responsiveness checks, but it is not the best profile for judging 3D surface shape.

Use `orbbec_gemini.launch.py` as the baseline camera entry for system validation. Use `orbbec_gemini_cloud.launch.py` for 3D contour inspection. Use `orbbec_gemini_fast.launch.py` only when you are checking live point-cloud responsiveness or isolating visualization load.

For RTAB-Map SLAM validation, prefer the SLAM camera profile:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_slam.launch.py
```

The SLAM profile keeps RGB, registered depth, camera info, and camera TF, but disables Orbbec PointCloud2 generation and left/right IR streams. RTAB-Map consumes the depth image directly, so the extra point-cloud stream is not needed for visual odometry and can overload the camera/CPU path.

Expected topics:

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
```

The Orbbec wrapper uses `gemini_330_series.launch.py` for Gemini 335L.

## RTAB-Map RGB-D SLAM

Production mode expects EKF to already publish `/odometry/filtered`. Start the camera and production RTAB-Map together:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_formal.launch.py
```

Or, if the camera is already running:

```bash
ros2 launch agri_vslam_bringup rtabmap_rgbd.launch.py
```

The older `rgbd_vslam.launch.py` remains as a compatibility wrapper for the same production chain. Prefer `rgbd_vslam_formal.launch.py` when doing navigation tests so it is not confused with `rgbd_vslam_vo_test.launch.py`.

Default inputs:

```text
rgb_topic: /camera/color/image_raw
depth_topic: /camera/depth/image_raw
camera_info_topic: /camera/color/camera_info
odom_topic: /odometry/filtered
frame_id: bb_robot/base_link
odom_frame_id: ""
```

In production mode, `odom_frame_id` is intentionally empty so RTAB-Map uses `/odometry/filtered` as its odometry source. The expected `/odometry/filtered` frames are:

```text
header.frame_id: bb_robot/odom
child_frame_id: bb_robot/base_link
```

Expected output:

```text
map -> bb_robot/odom -> bb_robot/base_link
```

If `/odometry/filtered` is not available yet, use the temporary visual-odometry test launch to validate the RGB-D RTAB-Map chain by itself:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_vo_test.launch.py
```

This test mode lets RTAB-Map publish temporary visual odometry:

```text
map -> bb_robot/odom -> bb_robot/base_link
```

Use it only to test the camera and RTAB-Map path. The production navigation chain should return to `/odometry/filtered` from EKF.

The RTAB-Map launches default to `approx_sync_max_interval:=0.04` and `wait_for_transform:=0.5`. The 0.04 s sync window keeps the normal Gemini RGB/depth offset around 0.033 s while rejecting worse pairings observed during tests.

Check:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map bb_robot/base_link
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-durability transient_local
```

Production success checklist:

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo bb_robot/odom bb_robot/base_link
ros2 run tf2_ros tf2_echo map bb_robot/odom
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-durability transient_local
ros2 topic list | grep rtabmap
```

Expected ownership:

```text
bb_robot/odom -> bb_robot/base_link: robot_localization / EKF
map -> bb_robot/odom: RTAB-Map
```

## Camera-Only Mode

Use this mode when the chassis/EKF is not available yet and only the Gemini camera is connected:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only.launch.py
```

This is the strict full-stream camera-only baseline. It uses the normal camera wrapper defaults and RTAB-Map's normal visual odometry thresholds. Use it for stress testing the full camera wrapper, not as the first navigation-chain SLAM validation.

For the normal camera-only SLAM validation gate, use:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_stable.launch.py
```

This profile is closer to how the navigation stack should run: it disables camera streams that are not required by RTAB-Map and uses moderately tolerant visual-odometry settings while still keeping the temporary camera-only TF role explicit.

The stable profile also enables the Gemini synchronized accelerometer/gyroscope at
200 Hz and filters it with `imu_filter_madgwick`:

```text
/camera/gyro_accel/sample -> imu_filter_madgwick -> /camera/imu/data
```

The filtered topic is available for diagnostics and later EKF/AOA integration. It is
not fed into RTAB-Map by default: without a magnetometer or another heading reference,
Madgwick roll and pitch are gravity-corrected but its absolute yaw is arbitrary. The
camera-only profile already uses `Reg/Force3DoF=true`, so RTAB-Map remains restricted
to planar x/y/yaw motion and estimates yaw visually.

Only enable RTAB-Map IMU initialization after the heading/frame policy is calibrated:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_stable_scan.launch.py \
  rtabmap_imu_topic:=/camera/imu/data \
  wait_imu_to_init:=true
```

To validate the full camera-only input chain needed before Nav2 local costmap
work, start stable RGB-D SLAM and fake `/scan` together:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_stable_scan.launch.py
```

This starts:

```text
Gemini 335L -> RGB/depth topics
RTAB-Map VO test -> map -> bb_robot/odom -> bb_robot/base_link
depthimage_to_laserscan -> /scan
```

Use this only while the chassis/EKF is unavailable. It is a validation chain,
not the production navigation TF ownership.

If the point cloud updates slowly or the desktop becomes hard to drag, use the fast camera-only profile as a separate diagnostic mode:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_camera_only_fast.launch.py
```

The fast camera-only profile uses a wider RGB/depth sync window and more tolerant visual-odometry defaults:

```text
Vis/MinInliers: 8
Vis/MaxFeatures: 1200
OdomF2M/MaxSize: 1200
Odom/ResetCountdown: 1
Odom/GuessMotion: true
```

If the log shows `Registration failed: Not enough inliers` and `Odom: quality=0`, RTAB-Map cannot find enough visual features in the current view. Point the camera at textured objects, avoid blank walls/floors, keep objects about 0.5-5 m away, and move the camera slowly.

Do not treat the fast profile as proof that the production chain is healthy. It is useful for interactive diagnosis, but the system-level milestone remains:

```text
EKF publishes bb_robot/odom -> bb_robot/base_link
RTAB-Map publishes map -> bb_robot/odom
Nav2 consumes /odometry/filtered, /map, and /scan
```

This is an alias of the VO test chain. It lets RTAB-Map temporarily publish:

```text
map -> bb_robot/odom -> bb_robot/base_link
```

Use camera-only mode to validate:

- Orbbec RGB-D stream;
- RTAB-Map visual odometry;
- `/map`;
- depth-derived fake `/scan`.

Latest camera-only validation result:

```text
/camera/color/image_raw: RGB image topic published
/camera/depth/image_raw: registered depth topic published
/rtabmap/odom: bb_robot/odom -> bb_robot/base_link
/map: nav_msgs/msg/OccupancyGrid, resolution 0.05 m
map -> bb_robot/base_link: connected
/scan: sensor_msgs/msg/LaserScan, about 9-10 Hz
```

Live hardware gate on 2026-07-18:

```text
Camera: Orbbec Gemini 335L, USB 3.2, firmware 1.4.60
RGB-D profile: 640x480 at 15 Hz
Raw synchronized IMU: /camera/gyro_accel/sample, configured at 200 Hz
Filtered IMU: /camera/imu/data, measured about 188-189 Hz
Validation database: 525 RTAB-Map nodes, 63.7 MiB
```

The chain remained connected and visual odometry remained healthy during the
stationary observation period. The database contains one main 482-node mapping
segment. Later movement while the professional MapCloud view was using software
rendering caused processing delay, low visual-feature inlier counts, and repeated
odometry resets; the final database therefore contains 20 map segments. This is a
passed sensor/topic/TF integration gate, but it is not yet a passed
navigation-grade motion test. Repeat the motion test with the lightweight RViz
view and then with working GPU acceleration before accepting VSLAM for Nav2.

Map check:

```bash
ros2 topic list -t | grep -E '(^/map|/scan|rtabmap/odom)'
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --qos-durability transient_local
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map bb_robot/base_link
```

Visualize the validated camera-only result chain:

```bash
ros2 launch agri_vslam_bringup rviz_camera_only_results.launch.py
```

This RViz view uses `map` as the fixed frame and displays `/map`, `/scan`,
`/rtabmap/odom`, and TF. Point cloud and RGB image displays are present but
disabled by default to keep the view responsive.

For the professional SLAM result view, use:

```bash
ros2 launch agri_vslam_bringup rviz_vslam_mapping.launch.py
```

This view uses `rtabmap_rviz_plugins` and displays:

```text
/rtabmap/mapData   accumulated RGB-D MapCloud
/rtabmap/mapGraph  keyframes, neighbor links, and loop closures
/rtabmap/mapPath   estimated trajectory
/map               2D navigation occupancy grid
/scan              depth-derived local obstacle scan
TF and RobotModel  frame ownership and sensor mounting
```

The clean default view intentionally enables only the accumulated 3D MapCloud,
the estimated green trajectory, the current visual-odometry pose, and the robot
model. The other layers remain available as diagnostics:

```text
Pose Graph and Loop Closures  RTAB-Map keyframes and graph constraints
Navigation Occupancy Grid     5 cm 2D cells consumed later by Nav2
Depth Fake Scan               current depth slice converted to LaserScan
TF                            all coordinate frames and parent-child arrows
```

Do not interpret the black/white occupancy cells as a 3D point cloud. They are a
top-down navigation raster: free, occupied, and unknown space. The accumulated
RGB-D MapCloud is the colored 3D reconstruction. Its default rendering uses
decimation 4, a 0.03 m voxel, and 2-pixel points so room-scale contours remain
visible without placing the full raw camera cloud load on RViz.

The MapCloud is decimated and voxelized in RViz to protect visual-odometry
responsiveness. The RGB image panel is available but disabled by default.

On the current laptop, the professional MapCloud view is an on-demand research
visualization, not the always-on navigation view. The live check on 2026-07-18 found
that `nvidia-smi` could not communicate with the NVIDIA driver and the desktop set
`QT_XCB_GL_INTEGRATION=none`. RViz therefore consumed multiple CPU cores while
rendering MapCloud. Use `rviz_camera_only.launch.py` as the always-on status view
during motion tests. Open the result or professional view briefly when inspecting
the occupancy grid, accumulated 3D geometry, trajectory, or loop closures. Fix the
host GPU driver before treating RViz rendering FPS as a VSLAM performance metric.

The professional view keeps the 2D OccupancyGrid display available but disabled by
default because this host currently reports an RViz GLSL sampler conflict for the
Map display. `/map` remains valid and continues to publish independently of that
rendering issue.

Do not use camera-only mode as the production navigation chain. When the chassis/EKF is available, switch back to:

```bash
ros2 launch agri_vslam_bringup rgbd_vslam_formal.launch.py
```

The production chain must keep this ownership:

```text
bb_robot/odom -> bb_robot/base_link: EKF
map -> bb_robot/odom: RTAB-Map
```

To reset the RTAB-Map database during early tests:

```bash
ros2 launch agri_vslam_bringup rtabmap_rgbd.launch.py \
  rtabmap_args:="--delete_db_on_start"
```

## Fake Scan From Depth

After RGB-D topics are stable, generate a temporary `/scan` for Nav2 local costmap tests:

```bash
ros2 launch agri_vslam_bringup fake_scan_from_depth.launch.py
```

The default camera info topic is `/camera/color/camera_info`, matching the registered depth image when `depth_registration:=true`.

This is only a first local-obstacle source. It does not replace visual SLAM or EKF odometry.

## RViz Camera-Only Visualization

If you only want to check the real-time depth geometry from the Gemini camera, use the lightweight point-cloud view first:

```bash
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
ros2 launch agri_vslam_bringup rviz_depth_points.launch.py
```

This view uses `camera_link` as the fixed frame and subscribes to the raw depth contour cloud:

```text
/camera/depth/points
```

It colors the cloud by depth axis, which makes 3D contour easier to see than a single flat color. It also leaves `/camera/depth_registered/points` available as a disabled optional display. Use it to tune camera placement and confirm nearby 3D geometry without the extra RTAB-Map/RViz rendering load.

Recommended point-cloud contour test:

Terminal 1:

```bash
cd /home/czb/agri_robot_system/perception_ws
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash

ros2 launch agri_vslam_bringup orbbec_gemini_cloud.launch.py
```

Terminal 2:

```bash
cd /home/czb/agri_robot_system/perception_ws
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/camera_ws/install/setup.bash
source install/setup.bash

ros2 launch agri_vslam_bringup rviz_depth_points.launch.py
```

Quick topic checks:

```bash
ros2 topic hz /camera/depth/points
ros2 topic echo /camera/depth/points sensor_msgs/msg/PointCloud2 --once --field width
ros2 topic echo /camera/depth/points sensor_msgs/msg/PointCloud2 --once --field height
```

For the best responsiveness, run only one visualization tool at a time. Use `rviz_depth_points.launch.py` for 3D geometry, and use `rqt_image_view` separately only when you need to inspect the color image.

If `rtabmap_viz` opens but shows a black/empty view, use the RViz camera-only SLAM status view:

```bash
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
ros2 launch agri_vslam_bringup rviz_camera_only.launch.py
```

The SLAM status view uses `bb_robot/odom` as the fixed frame and displays lightweight TF/odometry status without the RViz OccupancyGrid Map plugin. The raw point cloud and fake `/scan` displays are disabled by default so the visualizer does not overload RTAB-Map while you are validating odometry.

```text
/rtabmap/odom
TF
```

Use `rviz_depth_points.launch.py` separately for live point cloud. Start `fake_scan_from_depth.launch.py` separately if you want `/scan` and manually enable the Fake Scan display.

To view the raw color image directly:

```bash
source /home/czb/anaconda3/etc/profile.d/conda.sh
conda deactivate
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

Avoid launching ROS GUI tools from the Conda `base` environment. ROS Humble uses Python 3.10, while the Conda base on this machine may select Python 3.13 and break `rclpy`.

## 与正射占据地图集成

正射处理后的静态占据地图由`agri_map_integration`发布到`/map`。为避免
两个地图发布者冲突，RTAB-Map在统一地图模式下必须使用独立接口：

```text
RTAB-Map坐标帧：vslam_map
RTAB-Map二维栅格：/rtabmap/grid_map
Nav2静态地图：/map
```

`rgbd_vslam_camera_only_stable.launch.py`和
`rgbd_vslam_camera_only_stable_scan.launch.py`已经支持
`map_frame_id`参数。统一入口会自动传入正确参数：

```bash
ros2 launch agri_map_integration integrated_mapping.launch.py
```

不要在该模式下把RTAB-Map的`map_topic`重新设为`/map`。VSLAM启动后，
在统一RViz中使用`2D Pose Estimate`设置当前设备位置和正前方，地图配准
节点将发布唯一的`map -> vslam_map`。
