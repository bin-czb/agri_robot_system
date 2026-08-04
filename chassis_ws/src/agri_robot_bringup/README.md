# agri_robot_bringup

This package owns the base, sensor, and localization launch files.

Simulation localization starts the robot description with the `bb_robot/` frame prefix, bridges Gazebo `/clock` and odometry, and lets `robot_localization` publish the `bb_robot/odom -> bb_robot/base_link` TF. Do not bridge Gazebo's `/model/bb_robot/tf` at the same time.

For the current simulation stage, keep the `bb_robot/` frame prefix. Real-robot configs stay separate and use standard `odom -> base_link` frames.

Full simulation bringup:

```bash
cd /home/czb/agri_robot_system/chassis_ws
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 launch agri_robot_bringup sim_bringup.launch.py headless:=false
```

The GUI mode matches the previously stable Gazebo command: `hexarotor_forest_orchard.sdf`, OGRE2, and `simple_gui_ogre2.config`.

For headless navigation debugging:

```bash
ros2 launch agri_robot_bringup sim_bringup.launch.py
```

To try the lightweight layout:

```bash
ros2 launch agri_robot_bringup sim_bringup.launch.py \
  headless:=false \
  gui_config:=agri_light_gui.config \
  render_engine_gui:=ogre
```

If the GUI exits with a GLX/EGL/OpenGL context error:

```bash
ros2 launch agri_robot_bringup sim_bringup.launch.py \
  headless:=false \
  gui_software_rendering:=true \
  gui_gl_integration:=xcb_glx
```

Why this bringup exists:

- Keep Gazebo TF out of ROS while `robot_localization` publishes `bb_robot/odom -> bb_robot/base_link`.
- Bridge only the inputs we need: `/clock`, `/model/bb_robot/odometry`, camera topics, and `/cmd_vel`.
- Provide one repeatable command for simulation, EKF, camera bridge, and navigation preparation.

```bash
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/chassis_ws/install/setup.bash
ros2 launch agri_robot_bringup localization.launch.py
```

For standard real-robot frames, use:

```bash
ros2 launch agri_robot_bringup localization.launch.py \
  params_file:=/home/czb/agri_robot_system/chassis_ws/src/agri_robot_bringup/config/ekf_real.yaml \
  frame_prefix:='' \
  bridge_gazebo_odom:=false
```

The real EKF configuration currently uses the Orbbec Gemini built-in synchronized IMU:

```text
imu0: /camera/gyro_accel/sample
```

Only yaw angular velocity is fused for now. The Orbbec IMU message does not provide a trusted absolute orientation estimate, so orientation and linear acceleration are intentionally not fused in `ekf_real.yaml`.

Start the IMU-capable camera entry before the real EKF:

```bash
ros2 launch agri_vslam_bringup orbbec_gemini_imu.launch.py
ros2 topic hz /camera/gyro_accel/sample
```
