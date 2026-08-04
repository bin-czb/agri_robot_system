# Orbbec IMU, Camera, and AOA Rig Calibration

This directory calibrates and validates the rigid ground sensor rig formed by:

- Orbbec Gemini 335L RGB-D camera;
- its synchronized internal accelerometer and gyroscope;
- the ground AOA base station mounted above and parallel to the camera.

The generated default files are:

```text
~/.ros/agri_rig_calibration.yaml             complete measurement record
~/.ros/agri_aoa_calibration.params.yaml      runtime AOA ROS parameters
```

## Accepted Ground-Rig Baseline

The 2026-07-20 Gemini 335L baseline is archived at:

```text
calibration_results/accepted/agri_rig_calibration_20260720.yaml
```

Accepted measurements:

```text
IMU six-position validation: passed
maximum corrected gravity error: 0.00857 m/s^2
online corrected IMU rate: about 200.7 Hz
online stationary gravity norm: 9.81663 m/s^2
depth validation range: 0.75-4.00 m
depth fitted RMS error: 0.01201 m
depth fitted maximum absolute error: 0.01468 m
```

The depth fit is retained as an error model and research record. The runtime
camera path continues to use the Orbbec factory depth calibration because the
fitted `0.02776 m` bias may include the mechanical offset between the ruler's
reference plane and the optical centre, and the remaining error is already below
the navigation grid resolution.

## What Can And Cannot Be Estimated

The scripts estimate:

- an immutable snapshot of camera intrinsics and RGB/depth synchronization;
- IMU accelerometer bias, diagonal gain, gyro bias, and static noise;
- AOA azimuth sign and camera-forward mounting yaw offset;
- AOA range scale and bias;
- depth-center range scale/bias as a validation result.

The following quantities require a different method:

- Absolute ENU yaw cannot be obtained from the Orbbec six-axis IMU because it
  has no magnetometer. Use a surveyed direction, map alignment, or the later
  global registration estimator.
- Camera-to-IMU time offset and full six-degree-of-freedom extrinsic calibration
  require an AprilGrid/checkerboard motion dataset and a tool such as Kalibr.
  The initial navigation build uses the Orbbec factory camera/IMU calibration.
- The base-to-camera, base-to-AOA, and UAV RTK-to-AOA-tag lever arms cannot be
  uniquely inferred from static messages. Measure them after final rigid mounting.

## Coordinate Contract

All manually measured rig vectors use ROS FLU:

```text
x: forward, along camera forward and AOA zero direction
y: left
z: up
units: metres and radians
```

AOA lever arms must reference the antenna phase centre, not the plastic housing.
Camera translation should reference `camera_link`, not the optical image plane.

## Physical Preparation

1. Rigidly fasten the camera and AOA. Do not change their relative position after
   calibration.
2. Mark a sensor-rig origin and the camera-forward centreline on the mounting plate.
3. Warm up the camera and AOA for at least 10 minutes.
4. Use a direct USB 3.x connection and remove nearby moving objects.
5. For AOA tests, keep large metal surfaces and strong reflectors away from the
   direct base-to-tag line.
6. Measure all distances between sensor phase/reference centres, not enclosure edges.

## Start Calibration Sources

Terminal 1, camera and AOA:

```bash
cd /home/czb/pythonProject01/air_ground_cooperation_system_work
./tools/rig_calibration/start_calibration_sources.sh all /dev/ttyUSB0
```

`/dev/ttyUSB0` is the CH340 device detected during the 2026-07-19 check. Verify
the port again whenever another USB serial device or the UAV flight controller is
connected; device numbering can change after replugging.

For camera and IMU only:

```bash
./tools/rig_calibration/start_calibration_sources.sh camera
```

If the calibrated camera/IMU launch is already running, start only the AOA bench
source to avoid duplicate camera and IMU publishers:

```bash
./tools/rig_calibration/start_calibration_sources.sh aoa /dev/ttyUSB0
```

The AOA calibration source intentionally uses `fixed_east` and disables smoothing
and optimization. `/aoa/measurement` is still published before localization gates,
so UAV GPS is not required for the angle/range bench calibration.

Terminal 2, set up ROS for every acquisition command:

```bash
source /opt/ros/humble/setup.bash
source /home/czb/pythonProject01/OrbbecSDK_ROS2/install/setup.bash
source /home/czb/pythonProject01/trunk_tracking_ws/install/setup.bash
cd /home/czb/pythonProject01/air_ground_cooperation_system_work
```

## Stage 1: Device And Camera Check

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py check --duration 15
```

Confirm in the report:

- color and depth are close to the configured 15 Hz;
- synchronized IMU is close to the configured 200 Hz;
- both camera-info messages are present;
- RGB/depth timestamp offset is bounded and repeatable;
- frame IDs match the Orbbec TF tree.

This records the factory intrinsic matrices `K`, `D`, `R`, and `P`. Do not replace
them merely because the point cloud appears sparse; point-cloud density is an RViz
rendering parameter, not an intrinsic-calibration result.

## Stage 2: IMU Six-Position Calibration

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py imu-six \
  --duration 6
```

Use six axis-aligned stable orientations so each IMU message axis points once
with and once against gravity. Six arbitrary stationary angles are not valid for
this method. The script detects the dominant signed axis automatically and rejects
a face when that axis is more than 8 degrees from gravity. For each pose:

1. Put one flat face of the entire rigid rig flush on a level, stable surface.
2. Keep it motionless and press Enter.
3. Do not touch the table during the six-second acquisition.
4. Follow the script until `+x`, `-x`, `+y`, `-y`, `+z`, and `-z` are complete.

The result contains accelerometer bias/gain, gyro bias, noise variance, and the
corrected gravity norm for each pose. The script accepts the result only when all
six corrected norms are within `0.20 m/s^2` of `9.80665 m/s^2`; a rejected
candidate is stored separately and cannot be consumed by the IMU corrector.

Apply the profile without modifying the raw SDK topic:

```bash
/usr/bin/python3 tools/rig_calibration/imu_bias_corrector.py \
  --profile ~/.ros/agri_rig_calibration.yaml \
  --input-topic /camera/gyro_accel/sample \
  --output-topic /camera/imu/calibrated_raw
```

Then start Madgwick against the calibrated raw topic:

```bash
ros2 launch agri_vslam_bringup orbbec_imu_filter.launch.py \
  raw_imu_topic:=/camera/imu/calibrated_raw \
  filtered_imu_topic:=/camera/imu/data
```

Yaw from this filtered topic is relative and will drift. Do not use it as an
absolute East/North heading.

## Stage 3: Camera Depth-Range Validation

Use a large flat wall and accurately measure perpendicular distances from the
camera reference plane. Keep the wall in the centre of the depth image.

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py depth-range \
  0.75 1.50 2.50 4.00 --duration 5
```

The script fits measured depth against true distance and records residuals. This
is a validation measurement. Keep the Orbbec factory depth calibration unless a
repeatable residual exceeds the experiment requirement across several sessions.

## Stage 4: AOA Axis And Zero Calibration

Place the tag at the same height as the ground AOA phase centre and at least two
metres away. Use the camera-forward line as the body-frame zero direction.

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py aoa-axis \
  --duration 8
```

The wizard collects front, left, right, and back positions. It evaluates both
clockwise conventions and outputs:

```text
aoa_azimuth_sign
aoa_mount_yaw_offset_deg
rms_error_deg
max_abs_error_deg
```

Do not type or manually wrap angles. The fit uses circular statistics across the
`-180/180` boundary.

## Stage 5: AOA Range Calibration

Measure true slant range directly from the ground AOA phase centre to the tag
phase centre. Use at least four well-separated distances over the intended range.

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py aoa-range \
  1.00 2.00 4.00 6.00 --duration 8
```

The fitted runtime equation is:

```text
r_calibrated = (r_raw - range_bias_m) / range_scale
```

If residuals change strongly with direction, the error is likely multipath or
antenna pattern rather than a single scalar range calibration.

## Stage 6: Record Rigid Extrinsics

After final installation, measure:

- `base_to_aoa`: ground `base_link` to AOA phase centre;
- `tag_offset`: UAV RTK antenna to AOA tag centre, in UAV FLU;
- `base_to_camera`: ground `base_link` to Orbbec `camera_link`;
- `base_to_camera_rpy`: camera mounting roll, pitch, yaw in radians.

Replace the example numbers below with measured values:

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py set-extrinsics \
  --base-to-aoa X Y Z \
  --tag-offset X Y Z \
  --base-to-camera X Y Z \
  --base-to-camera-rpy ROLL PITCH YAW
```

Do not run this command with placeholder letters. Translation uncertainty should
be recorded in the experiment notebook, especially the vertical AOA and UAV tag
offsets used for slant-to-horizontal projection.

## Stage 7: Export And Use AOA Parameters

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py export-aoa
```

Start the formal moving-cart AOA chain with the generated file as argument 8:

```bash
cd /home/czb/pythonProject01/air_ground_cooperation_system_work/systems/aoa
./launch_aoa.sh \
  /dev/ttyACM0 /dev/ttyUSB0 enu \
  odometry_relative 0.0 /odometry/filtered /camera/imu/data \
  ~/.ros/agri_aoa_calibration.params.yaml
```

Replace `0.0` with the surveyed initial ENU heading of the rigid rig.

## Final Acceptance

1. Keep the rig stationary for two minutes. Confirm corrected gyro mean remains
   near zero and acceleration norm remains near gravity.
2. Put the AOA tag front, left, and right. Confirm `/aoa/measurement` field 3 is
   near `0`, `+90`, and `-90` degrees.
3. Recheck at one distance that was not used in fitting.
4. Rotate the complete ground rig without moving its reference point. The solved
   UTM position should remain stable after the heading source is connected.
5. Move slowly through a textured scene. Confirm VSLAM trajectory remains smooth
   and no second node publishes the same TF edge.
6. Record calibration file hash, temperature, serial numbers, and test date with
   each research dataset.

Print the current profile at any time:

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py report
```

Run deterministic mathematical tests without hardware:

```bash
/usr/bin/python3 tools/rig_calibration/rig_calibration.py self-test
```
