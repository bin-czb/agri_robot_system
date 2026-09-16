"""AUTO/MANUAL command selector for the real chassis.

Information flow in agri_robot_system (ROS 2 Humble):

AUTO:
    Nav2 controller_server -> /cmd_vel_nav
    Nav2 velocity_smoother -> /cmd_vel
    this node              -> /chassis/cmd_vel
    agri_chassis_can       -> TD48150B CAN

MANUAL:
    Linux joystick -> joy_node -> /joy
    this node       -> /chassis/cmd_vel
    agri_chassis_can -> TD48150B CAN

The important design rule is that Nav2 and the joystick never publish directly to
the CAN driver at the same time.  This node is the single command selector and
the CAN node has exactly one ROS velocity input: /chassis/cmd_vel.

The user-facing behavior follows the tested Taizhou usage: the A button toggles
AUTO <-> MANUAL.  The current public Taizhou mirror does not contain the
original chassis_bringup.launch.py, so the A/axis numeric indices remain ROS
parameters and must be checked once with `ros2 topic echo /joy` on the actual
controller before active motion.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class ChassisModeTeleop(Node):
    AUTO = "AUTO"
    MANUAL = "MANUAL"

    def __init__(self) -> None:
        super().__init__("chassis_mode_teleop")

        # /cmd_vel is the FINAL Nav2 command in ROS 2 Humble navigation_launch.py:
        # controller_server publishes /cmd_vel_nav and velocity_smoother publishes
        # the smoothed result on /cmd_vel.  Do not subscribe to /cmd_vel_nav here,
        # otherwise the existing Nav2 velocity_smoother would be bypassed.
        self.declare_parameter("auto_cmd_topic", "/cmd_vel")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("output_cmd_topic", "/chassis/cmd_vel")
        self.declare_parameter("mode_topic", "/chassis/control_mode")

        self.declare_parameter("initial_mode", self.AUTO)

        # Taizhou/Xbox-style defaults.  These are deliberately parameters because
        # Linux joystick mappings can differ by controller/kernel.  Verify once on
        # the real gamepad before disabling chassis listen_only mode.
        self.declare_parameter("toggle_button", 0)  # A button on common Xbox mapping
        self.declare_parameter("linear_axis", 1)    # left stick vertical
        self.declare_parameter("angular_axis", 0)   # left stick horizontal
        self.declare_parameter("linear_axis_sign", 1.0)
        self.declare_parameter("angular_axis_sign", 1.0)

        # These are bring-up limits, intentionally aligned with the CAN node's
        # current conservative limits.  The CAN node clamps again downstream.
        self.declare_parameter("manual_max_linear_mps", 0.10)
        self.declare_parameter("manual_max_angular_radps", 0.30)

        # Source watchdogs prevent a stale Nav2 command or stale joystick state
        # from surviving a cable disconnect, node crash, or mode change.
        self.declare_parameter("joy_timeout_s", 0.50)
        self.declare_parameter("auto_timeout_s", 0.50)
        self.declare_parameter("publish_rate_hz", 20.0)

        initial_mode = str(self.get_parameter("initial_mode").value).strip().upper()
        self.mode = self.MANUAL if initial_mode == self.MANUAL else self.AUTO

        self.toggle_button = int(self.get_parameter("toggle_button").value)
        self.linear_axis = int(self.get_parameter("linear_axis").value)
        self.angular_axis = int(self.get_parameter("angular_axis").value)
        self.linear_axis_sign = float(self.get_parameter("linear_axis_sign").value)
        self.angular_axis_sign = float(self.get_parameter("angular_axis_sign").value)
        self.manual_max_linear_mps = abs(
            float(self.get_parameter("manual_max_linear_mps").value)
        )
        self.manual_max_angular_radps = abs(
            float(self.get_parameter("manual_max_angular_radps").value)
        )
        self.joy_timeout_s = max(0.05, float(self.get_parameter("joy_timeout_s").value))
        self.auto_timeout_s = max(
            0.05, float(self.get_parameter("auto_timeout_s").value)
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("output_cmd_topic").value),
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("mode_topic").value),
            10,
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("auto_cmd_topic").value),
            self._auto_cmd_cb,
            10,
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._joy_cb,
            10,
        )

        self.last_auto_cmd = Twist()
        self.last_auto_time = 0.0
        self.last_joy_time = 0.0
        self.manual_linear = 0.0
        self.manual_angular = 0.0

        # Do not treat a button already held when joy_node first appears as a new
        # A-button press.  The first Joy message only establishes button state.
        self.joy_button_state_initialized = False
        self.last_toggle_pressed = False

        # A command received before this timestamp is not allowed to move the
        # robot after a mode switch.  This prevents stale cached commands from
        # becoming active when AUTO/MANUAL changes.
        self.mode_switch_time = time.monotonic()

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self._publish_loop)
        self.create_timer(1.0, self._publish_mode)

        self._publish_mode()
        self.get_logger().info(
            "command selector ready: "
            f"mode={self.mode}, auto={self.get_parameter('auto_cmd_topic').value}, "
            f"joy={self.get_parameter('joy_topic').value}, "
            f"output={self.get_parameter('output_cmd_topic').value}"
        )

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    @staticmethod
    def _button(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    @staticmethod
    def _axis(msg: Joy, index: int) -> float:
        if 0 <= index < len(msg.axes):
            return max(-1.0, min(1.0, float(msg.axes[index])))
        return 0.0

    def _auto_cmd_cb(self, msg: Twist) -> None:
        self.last_auto_cmd = msg
        self.last_auto_time = time.monotonic()

    def _joy_cb(self, msg: Joy) -> None:
        now = time.monotonic()
        pressed = self._button(msg, self.toggle_button)

        if not self.joy_button_state_initialized:
            self.last_toggle_pressed = pressed
            self.joy_button_state_initialized = True
        else:
            # Rising edge only: holding A cannot repeatedly flip the mode.
            if pressed and not self.last_toggle_pressed:
                self._toggle_mode(now)
            self.last_toggle_pressed = pressed

        # Store manual axes even while AUTO is active.  mode_switch_time below
        # still requires a NEW Joy message after switching to MANUAL, so the
        # cached stick position from AUTO can never move the robot immediately.
        self.manual_linear = (
            self._axis(msg, self.linear_axis)
            * self.linear_axis_sign
            * self.manual_max_linear_mps
        )
        self.manual_angular = (
            self._axis(msg, self.angular_axis)
            * self.angular_axis_sign
            * self.manual_max_angular_radps
        )
        self.last_joy_time = now

    def _toggle_mode(self, now: float) -> None:
        self.mode = self.MANUAL if self.mode == self.AUTO else self.AUTO
        self.mode_switch_time = now

        # Stop first, then wait for a fresh message from the newly selected source.
        self.cmd_pub.publish(self._zero_twist())
        self._publish_mode()
        self.get_logger().warn(f"control mode -> {self.mode}; zero command sent")

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.mode))

    def _publish_loop(self) -> None:
        now = time.monotonic()
        out = Twist()

        if self.mode == self.AUTO:
            auto_is_fresh = (
                self.last_auto_time >= self.mode_switch_time
                and now - self.last_auto_time <= self.auto_timeout_s
            )
            if auto_is_fresh:
                out = self.last_auto_cmd
        else:
            joy_is_fresh = (
                self.last_joy_time >= self.mode_switch_time
                and now - self.last_joy_time <= self.joy_timeout_s
            )
            if joy_is_fresh:
                out.linear.x = float(self.manual_linear)
                out.angular.z = float(self.manual_angular)

        # Publish continuously so the downstream chassis node receives a clear
        # zero command when the selected source is stale.  The CAN node has its
        # own independent watchdog as the second safety layer.
        self.cmd_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisModeTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort stop on shutdown.  The CAN node independently times out too.
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
