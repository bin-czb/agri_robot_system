"""Reject invalid ROS velocity inputs before they can reach the CAN driver."""

from types import SimpleNamespace

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from agri_chassis_can.chassis_can_node import TD48150BChassisNode
from agri_chassis_can.chassis_mode_teleop import ChassisModeTeleop


class Logger:
    def error(self, _message):
        pass


def test_can_driver_rejects_nonfinite_velocity():
    driver = SimpleNamespace(
        max_linear_mps=0.1,
        max_angular_radps=0.3,
        target_linear=0.05,
        target_angular=0.1,
        last_cmd_time=1.0,
        get_logger=lambda: Logger(),
    )
    command = Twist()
    command.linear.x = float("nan")
    TD48150BChassisNode._cmd_vel_cb(driver, command)
    assert driver.target_linear == 0.0
    assert driver.target_angular == 0.0
    assert driver.last_cmd_time is None


def test_auto_selector_rejects_nonfinite_velocity():
    selector = SimpleNamespace(
        last_auto_cmd=Twist(),
        last_auto_time=1.0,
        get_logger=lambda: Logger(),
    )
    command = Twist()
    command.angular.z = float("inf")
    ChassisModeTeleop._auto_cmd_cb(selector, command)
    assert selector.last_auto_cmd.linear.x == 0.0
    assert selector.last_auto_cmd.angular.z == 0.0
    assert selector.last_auto_time == 0.0


def test_joystick_rejects_nonfinite_axis():
    selector = SimpleNamespace(
        toggle_button=0,
        linear_axis=1,
        angular_axis=0,
        linear_axis_sign=1.0,
        angular_axis_sign=1.0,
        manual_max_linear_mps=0.1,
        manual_max_angular_radps=0.3,
        joy_button_state_initialized=False,
        last_toggle_pressed=False,
        last_joy_time=1.0,
        _button=ChassisModeTeleop._button,
        _axis=ChassisModeTeleop._axis,
        get_logger=lambda: Logger(),
    )
    joystick = Joy()
    joystick.buttons = [0]
    joystick.axes = [float("nan"), 0.5]
    ChassisModeTeleop._joy_cb(selector, joystick)
    assert selector.manual_linear == 0.0
    assert selector.manual_angular == 0.0
    assert selector.last_joy_time == 0.0
