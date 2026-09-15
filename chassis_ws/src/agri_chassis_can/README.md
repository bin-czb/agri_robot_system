# agri_chassis_can

TD48150B-2E real-chassis package for `agri_robot_system`.

## Command chain

```text
AUTO:   Nav2 /cmd_vel ----┐
                          ├-> chassis_mode_teleop -> /chassis/cmd_vel -> agri_chassis_can -> CAN
MANUAL: /joy + A button ---┘
```

A button defaults to joystick button index `0` and toggles `AUTO <-> MANUAL`.
Both modes use the same final `/chassis/cmd_vel` subscription, so manual control
verifies the same ROS 2 -> chassis node -> CAN path used by navigation.

Mode topic:

```bash
ros2 topic echo /chassis/control_mode
```

Service equivalents:

```bash
ros2 service call /chassis_mode_teleop/set_auto std_srvs/srv/Trigger '{}'
ros2 service call /chassis_mode_teleop/set_manual std_srvs/srv/Trigger '{}'
ros2 service call /chassis_mode_teleop/toggle std_srvs/srv/Trigger '{}'
```

## First hardware bring-up

The TD48150B manual specifies 250 kbit/s and extended CAN frames. First run must
remain listen-only until the real heartbeat ID is confirmed.

```bash
source /home/czb/agri_robot_system/scripts/source_all.bash
ros2 run agri_chassis_can setup_can.sh can0 250000
candump can0
ros2 launch agri_chassis_can chassis_bringup.launch.py \
  listen_only:=true start_joy:=true
```

Only after CAN IDs, A/B mapping, motor signs, feedback unit, wheel radius,
track width, gear ratio and driver maximum RPM are verified should active
transmission be enabled.

The CAN node publishes `/wheel/odometry` and intentionally does not publish
`odom -> base_link`; the existing `robot_localization` EKF remains the sole TF
owner.
