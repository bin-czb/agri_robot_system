"""AUTO/MANUAL command gate with Taizhou-style A-button mode switching.

AUTO path:
    Nav2 /cmd_vel -> this node -> /chassis/cmd_vel -> CAN chassis node

MANUAL path:
    /joy -> this node -> /chassis/cmd_vel -> CAN chassis node

Both modes exercise the same downstream ROS chassis communication chain.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ChassisModeTeleop(Node):
    AUTO = "AUTO"
    MANUAL = "MANUAL"

    def __init__(self) -> None:
        super().__init__("chassis_mode_teleop")

        self.declare_parameter("auto_cmd_topic", "/cmd_vel")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("output_cmd_topic", "/chassis/cmd_vel")
        self.declare_parameter("mode_topic", "/chassis/control_mode")
        self.declare_parameter("initial_mode", "AUTO")
        self.declare_parameter("toggle_button", 0)
        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 0)
        self.declare_parameter("linear_axis_sign", 1.0)
        self.declare_parameter("angular_axis_sign", 1.0)
        self.declare_parameter("manual_max_linear_mps", 0.10)
        self.declare_parameter("manual_max_angular_radps", 0.30)
        self.declare_parameter("deadman_button", -1)
        self.declare_parameter("joy_timeout_s", 0.50)
        self.declare_parameter("auto_timeout_s", 0.50)
        self.declare_parameter("publish_rate_hz", 20.0)

        initial = str(self.get_parameter("initial_mode").value).strip().upper()
        self.mode = self.MANUAL if initial == self.MANUAL else self.AUTO
        self.toggle_button = int(self.get_parameter("toggle_button").value)
        self.linear_axis = int(self.get_parameter("linear_axis").value)
        self.angular_axis = int(self.get_parameter("angular_axis").value)
        self.linear_axis_sign = float(self.get_parameter("linear_axis_sign").value)
        self.angular_axis_sign = float(self.get_parameter("angular_axis_sign").value)
        self.manual_max_linear_mps = float(self.get_parameter("manual_max_linear_mps").value)
        self.manual_max_angular_radps = float(self.get_parameter("manual_max_angular_radps").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.joy_timeout_s = float(self.get_parameter("joy_timeout_s").value)
        self.auto_timeout_s = float(self.get_parameter("auto_timeout_s").value)

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_topic").value), 10
        )
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter("mode_topic").value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("auto_cmd_topic").value), self._auto_cmd_cb, 10
        )
        self.create_subscription(
            Joy, str(self.get_parameter("joy_topic").value), self._joy_cb, 10
        )

        self.create_service(Trigger, "~/set_auto", self._set_auto_srv)
        self.create_service(Trigger, "~/set_manual", self._set_manual_srv)
        self.create_service(Trigger, "~/toggle", self._toggle_srv)

        self.last_auto_cmd = Twist()
        self.last_auto_time = 0.0
        self.last_joy_time = 0.0
        self.manual_linear = 0.0
        self.manual_angular = 0.0
        self.last_toggle_pressed = False
        self.mode_switch_time = time.monotonic()

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_loop)
        self.create_timer(1.0, self._publish_mode)

        self.get_logger().info(
            f"mode={self.mode}; A button index={self.toggle_button}; "
            f"AUTO {self.get_parameter('auto_cmd_topic').value} -> "
            f"{self.get_parameter('output_cmd_topic').value}"
        )

    @staticmethod
    def _zero_twist() -> Twist:
        return Twist()

    def _switch_mode(self, mode: str, reason: str) -> None:
        new_mode = self.MANUAL if str(mode).upper() == self.MANUAL else self.AUTO
        if new_mode == self.mode:
            return
        self.mode = new_mode
        self.mode_switch_time = time.monotonic()
        self.cmd_pub.publish(self._zero_twist())
        self._publish_mode()
        self.get_logger().warn(f"control mode -> {self.mode} ({reason})")

    def _toggle_mode(self, reason: str) -> None:
        self._switch_mode(self.MANUAL if self.mode == self.AUTO else self.AUTO, reason)

    def _auto_cmd_cb(self, msg: Twist) -> None:
        self.last_auto_cmd = msg
        self.last_auto_time = time.monotonic()

    @staticmethod
    def _button(msg: Joy, index: int) -> bool:
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    @staticmethod
    def _axis(msg: Joy, index: int) -> float:
        return float(msg.axes[index]) if 0 <= index < len(msg.axes) else 0.0

    def _joy_cb(self, msg: Joy) -> None:
        self.last_joy_time = time.monotonic()
        pressed = self._button(msg, self.toggle_button)
        if pressed and not self.last_toggle_pressed:
            self._toggle_mode("A button")
        self.last_toggle_pressed = pressed

        deadman_ok = self.deadman_button < 0 or self._button(msg, self.deadman_button)
        if deadman_ok:
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
        else:
            self.manual_linear = 0.0
            self.manual_angular = 0.0

    def _publish_loop(self) -> None:
        now = time.monotonic()
        out = Twist()
        if self.mode == self.AUTO:
            fresh = (
                self.last_auto_time >= self.mode_switch_time
                and now - self.last_auto_time <= self.auto_timeout_s
            )
            if fresh:
                out = self.last_auto_cmd
        else:
            fresh = (
                self.last_joy_time >= self.mode_switch_time
                and now - self.last_joy_time <= self.joy_timeout_s
            )
            if fresh:
                out.linear.x = float(self.manual_linear)
                out.angular.z = float(self.manual_angular)
        self.cmd_pub.publish(out)

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.mode))

    def _set_auto_srv(self, _request, response):
        self._switch_mode(self.AUTO, "service")
        response.success = True
        response.message = self.mode
        return response

    def _set_manual_srv(self, _request, response):
        self._switch_mode(self.MANUAL, "service")
        response.success = True
        response.message = self.mode
        return response

    def _toggle_srv(self, _request, response):
        self._toggle_mode("service")
        response.success = True
        response.message = self.mode
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisModeTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
