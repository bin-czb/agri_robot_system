# agri_robot_description

This package provides the first navigation skeleton milestone: the robot URDF and TF publishers.

The model mirrors the `bb_robot` geometry in `trunk_gazebo_worlds` while keeping ROS navigation frame names such as `base_link`, `camera_link`, `camera_optical_frame`, and `imu_link`.

Run the description only:

```bash
source /opt/ros/humble/setup.bash
source /home/czb/agri_robot_system/chassis_ws/install/setup.bash
ros2 launch agri_robot_description description.launch.py
```

To match Gazebo's current DiffDrive TF names, add:

```bash
ros2 launch agri_robot_description description.launch.py frame_prefix:=bb_robot/
```
