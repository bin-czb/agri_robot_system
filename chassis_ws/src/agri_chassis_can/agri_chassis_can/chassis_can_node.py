"""ROS 2 driver for a TD48150B-2E dual-servo CAN chassis."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger

from .differential_kinematics import DifferentialKinematics, integrate_midpoint
from .socketcan_transport import SocketCanTransport
from .td48150b_protocol import (
    CHANNEL_A,
    CHANNEL_B,
    FaultFeedback,
    SpeedFeedback,
    TD48150BProtocol,
)


class TD48150BChassisNode(Node):
    def __init__(self) -> None:
        super().__init__("agri_chassis_can")

        self.declare_parameter("interface", "can0")
        self.declare_parameter("listen_only", True)
        self.declare_parameter("command_can_id", 0x06000001)
        self.declare_parameter("feedback_can_id", 0x05800001)
        self.declare_parameter("heartbeat_can_id", 0x07000001)
        self.declare_parameter("require_heartbeat_before_enable", True)
        self.declare_parameter("heartbeat_timeout_s", 2.5)
        self.declare_parameter("rx_poll_rate_hz", 100.0)

        self.declare_parameter("cmd_vel_topic", "/chassis/cmd_vel")
        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.50)
        self.declare_parameter("feedback_timeout_s", 0.30)
        self.declare_parameter("speed_query_rate_hz", 20.0)
        self.declare_parameter("diagnostic_query_rate_hz", 1.0)

        self.declare_parameter("wheel_radius_m", 0.20)
        self.declare_parameter("track_width_m", 1.22916)
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("driver_max_rpm", 3000.0)
        self.declare_parameter("max_linear_mps", 0.10)
        self.declare_parameter("max_angular_radps", 0.30)

        self.declare_parameter("left_channel", CHANNEL_A)
        self.declare_parameter("right_channel", CHANNEL_B)
        self.declare_parameter("left_command_sign", 1.0)
        self.declare_parameter("right_command_sign", 1.0)
        self.declare_parameter("left_feedback_sign", 1.0)
        self.declare_parameter("right_feedback_sign", 1.0)
        self.declare_parameter("feedback_speed_mode", "rpm")

        self.interface = str(self.get_parameter("interface").value)
        self.listen_only = bool(self.get_parameter("listen_only").value)
        self.command_can_id = int(self.get_parameter("command_can_id").value)
        self.feedback_can_id = int(self.get_parameter("feedback_can_id").value)
        self.heartbeat_can_id = int(self.get_parameter("heartbeat_can_id").value)
        self.require_heartbeat = bool(self.get_parameter("require_heartbeat_before_enable").value)
        self.heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        self.cmd_vel_timeout_s = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.feedback_timeout_s = float(self.get_parameter("feedback_timeout_s").value)
        self.max_linear_mps = abs(float(self.get_parameter("max_linear_mps").value))
        self.max_angular_radps = abs(float(self.get_parameter("max_angular_radps").value))
        self.left_channel = int(self.get_parameter("left_channel").value)
        self.right_channel = int(self.get_parameter("right_channel").value)
        self.left_command_sign = float(self.get_parameter("left_command_sign").value)
        self.right_command_sign = float(self.get_parameter("right_command_sign").value)
        self.left_feedback_sign = float(self.get_parameter("left_feedback_sign").value)
        self.right_feedback_sign = float(self.get_parameter("right_feedback_sign").value)
        self.feedback_speed_mode = str(self.get_parameter("feedback_speed_mode").value).strip().lower()
        self.driver_max_rpm = float(self.get_parameter("driver_max_rpm").value)

        if self.left_channel not in (CHANNEL_A, CHANNEL_B) or self.right_channel not in (CHANNEL_A, CHANNEL_B):
            raise ValueError("left_channel/right_channel must be 1 or 2")
        if self.left_channel == self.right_channel:
            raise ValueError("left_channel and right_channel must be different")
        if self.feedback_speed_mode not in ("rpm", "normalized"):
            raise ValueError("feedback_speed_mode must be 'rpm' or 'normalized'")

        self.kinematics = DifferentialKinematics(
            wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
            track_width_m=float(self.get_parameter("track_width_m").value),
            gear_ratio=float(self.get_parameter("gear_ratio").value),
            driver_max_rpm=self.driver_max_rpm,
        )

        self.transport = SocketCanTransport(self.interface)
        self.transport.open()

        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._cmd_vel_cb,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            20,
        )
        self.state_pub = self.create_publisher(String, "/chassis/can/state", 10)
        self.raw_rx_pub = self.create_publisher(String, "/chassis/can/raw_rx", 50)
        self.raw_tx_pub = self.create_publisher(String, "/chassis/can/raw_tx", 50)
        self.left_rpm_pub = self.create_publisher(Float32, "/chassis/left_motor_rpm", 20)
        self.right_rpm_pub = self.create_publisher(Float32, "/chassis/right_motor_rpm", 20)
        self.fault_a_pub = self.create_publisher(Int32, "/chassis/fault_a", 10)
        self.fault_b_pub = self.create_publisher(Int32, "/chassis/fault_b", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.create_service(Trigger, "~/enable", self._enable_srv)
        self.create_service(Trigger, "~/disable", self._disable_srv)
        self.create_service(Trigger, "~/estop", self._estop_srv)
        self.create_service(Trigger, "~/clear_estop", self._clear_estop_srv)

        self.enabled = False
        self.estop_latched = False
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time: Optional[float] = None
        self.last_heartbeat_time: Optional[float] = None
        self.last_speed_feedback_time: Optional[float] = None
        self.last_speed_odom_time: Optional[float] = None
        self.last_fault: Optional[FaultFeedback] = None
        self.last_speed_raw: Optional[SpeedFeedback] = None
        self.last_left_motor_rpm: Optional[float] = None
        self.last_right_motor_rpm: Optional[float] = None
        self.bus_voltage_raw: Optional[int] = None
        self.temperature_raw: Optional[tuple[int, int, int]] = None
        self.current_raw: Optional[tuple[int, int]] = None
        self.rx_count = 0
        self.tx_count = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._diag_query_index = 0

        rx_rate = max(10.0, float(self.get_parameter("rx_poll_rate_hz").value))
        control_rate = max(2.0, float(self.get_parameter("control_rate_hz").value))
        query_rate = max(0.1, float(self.get_parameter("speed_query_rate_hz").value))
        diag_query_rate = max(0.1, float(self.get_parameter("diagnostic_query_rate_hz").value))
        self.create_timer(1.0 / rx_rate, self._poll_can)
        self.create_timer(1.0 / control_rate, self._control_loop)
        self.create_timer(1.0 / query_rate, self._speed_query_loop)
        self.create_timer(1.0 / diag_query_rate, self._diagnostic_query_loop)
        self.create_timer(1.0, self._diagnostics_loop)

        mode = "LISTEN_ONLY" if self.listen_only else "ACTIVE_CAPABLE"
        self.get_logger().info(
            f"TD48150B CAN opened {self.interface}; {mode}; "
            f"cmd=0x{self.command_can_id:08X} feedback=0x{self.feedback_can_id:08X} "
            f"heartbeat=0x{self.heartbeat_can_id:08X}"
        )

    @staticmethod
    def _age(now: float, stamp: Optional[float]) -> float:
        return math.inf if stamp is None else max(0.0, now - stamp)

    def _heartbeat_alive(self, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        return self._age(now, self.last_heartbeat_time) <= self.heartbeat_timeout_s

    def _cmd_vel_cb(self, msg: Twist) -> None:
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.last_cmd_time = None
            self.get_logger().error("non-finite cmd_vel rejected; command watchdog set to zero")
            return
        self.target_linear = max(-self.max_linear_mps, min(self.max_linear_mps, linear))
        self.target_angular = max(
            -self.max_angular_radps,
            min(self.max_angular_radps, angular),
        )
        self.last_cmd_time = time.monotonic()

    def _send(self, payload: bytes) -> bool:
        if self.listen_only:
            return False
        try:
            self.transport.send(self.command_can_id, payload, extended=True)
        except OSError as exc:
            self.get_logger().error(f"CAN TX failed: {exc}")
            return False
        self.tx_count += 1
        self.raw_tx_pub.publish(
            String(data=f"id=0x{self.command_can_id:08X} data={payload.hex(' ').upper()}")
        )
        return True

    def _send_zero(self) -> None:
        self._send(TD48150BProtocol.speed_command(self.left_channel, 0))
        self._send(TD48150BProtocol.speed_command(self.right_channel, 0))

    def _enable_driver(self) -> tuple[bool, str]:
        if self.listen_only:
            return False, "listen_only=true; transmission disabled"
        if self.estop_latched:
            return False, "E-stop is latched; clear it first"
        if self.require_heartbeat and not self._heartbeat_alive():
            return False, "no fresh heartbeat; refusing to enable"
        ok_a = self._send(TD48150BProtocol.enable(CHANNEL_A))
        ok_b = self._send(TD48150BProtocol.enable(CHANNEL_B))
        if ok_a and ok_b:
            self.enabled = True
            self._send_zero()
            return True, "A/B enabled; zero speed sent"
        self.enabled = False
        return False, "enable transmission failed"

    def _disable_driver(self) -> tuple[bool, str]:
        if self.listen_only:
            self.enabled = False
            return False, "listen_only=true; no disable frame transmitted"
        self._send_zero()
        ok_a = self._send(TD48150BProtocol.disable(CHANNEL_A))
        ok_b = self._send(TD48150BProtocol.disable(CHANNEL_B))
        self.enabled = False
        return ok_a and ok_b, "A/B disabled"

    def _enable_srv(self, _request, response):
        response.success, response.message = self._enable_driver()
        return response

    def _disable_srv(self, _request, response):
        response.success, response.message = self._disable_driver()
        return response

    def _estop_srv(self, _request, response):
        self.estop_latched = True
        if not self.listen_only:
            self._send_zero()
            self._send(TD48150BProtocol.disable(CHANNEL_A))
            self._send(TD48150BProtocol.disable(CHANNEL_B))
        self.enabled = False
        response.success = True
        response.message = "software E-stop latched"
        return response

    def _clear_estop_srv(self, _request, response):
        self.estop_latched = False
        response.success = True
        response.message = "software E-stop cleared; driver remains disabled"
        return response

    def _control_loop(self) -> None:
        if self.listen_only or not self.enabled or self.estop_latched:
            return
        now = time.monotonic()
        if self.require_heartbeat and not self._heartbeat_alive(now):
            self.get_logger().error("heartbeat timeout while enabled; disabling")
            self._disable_driver()
            return
        if self._age(now, self.last_cmd_time) > self.cmd_vel_timeout_s:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.target_linear
            angular = self.target_angular
        targets = self.kinematics.body_twist_to_targets(
            linear,
            angular,
            left_command_sign=self.left_command_sign,
            right_command_sign=self.right_command_sign,
        )
        self._send(TD48150BProtocol.speed_command(self.left_channel, targets.left_command))
        self._send(TD48150BProtocol.speed_command(self.right_channel, targets.right_command))

    def _speed_query_loop(self) -> None:
        if not self.listen_only:
            self._send(TD48150BProtocol.query_speed())

    def _diagnostic_query_loop(self) -> None:
        if self.listen_only:
            return
        queries = (
            TD48150BProtocol.query_fault,
            TD48150BProtocol.query_current,
            TD48150BProtocol.query_bus_voltage,
            TD48150BProtocol.query_temperature,
        )
        query = queries[self._diag_query_index % len(queries)]
        self._diag_query_index += 1
        self._send(query())

    def _poll_can(self) -> None:
        try:
            frames = self.transport.recv_many()
        except OSError as exc:
            self.get_logger().error(f"CAN RX failed: {exc}")
            return
        now = time.monotonic()
        for frame in frames:
            self.rx_count += 1
            self.raw_rx_pub.publish(
                String(data=f"id=0x{frame.can_id:08X} data={frame.data.hex(' ').upper()}")
            )
            if not frame.is_extended or frame.is_error:
                continue
            if frame.can_id == self.heartbeat_can_id:
                self.last_heartbeat_time = now
                continue
            if frame.can_id != self.feedback_can_id:
                continue

            speed = TD48150BProtocol.parse_speed_feedback(frame.data)
            if speed is not None:
                self._handle_speed_feedback(speed, now)
                continue
            fault = TD48150BProtocol.parse_fault_feedback(frame.data)
            if fault is not None:
                self.last_fault = fault
                self.fault_a_pub.publish(Int32(data=int(fault.channel_a)))
                self.fault_b_pub.publish(Int32(data=int(fault.channel_b)))
                continue
            current = TD48150BProtocol.parse_current_feedback(frame.data)
            if current is not None:
                self.current_raw = current
                continue
            voltage = TD48150BProtocol.parse_bus_voltage_feedback(frame.data)
            if voltage is not None:
                self.bus_voltage_raw = voltage
                continue
            temperature = TD48150BProtocol.parse_temperature_feedback(frame.data)
            if temperature is not None:
                self.temperature_raw = temperature

    def _raw_speed_to_motor_rpm(self, raw: int) -> float:
        if self.feedback_speed_mode == "normalized":
            return float(raw) / 10000.0 * self.driver_max_rpm
        return float(raw)

    @staticmethod
    def _channel_value(feedback: SpeedFeedback, channel: int) -> int:
        return feedback.channel_a if channel == CHANNEL_A else feedback.channel_b

    def _handle_speed_feedback(self, feedback: SpeedFeedback, now: float) -> None:
        self.last_speed_raw = feedback
        left_raw = self._channel_value(feedback, self.left_channel)
        right_raw = self._channel_value(feedback, self.right_channel)
        left_motor_rpm = self._raw_speed_to_motor_rpm(left_raw) * self.left_feedback_sign
        right_motor_rpm = self._raw_speed_to_motor_rpm(right_raw) * self.right_feedback_sign
        self.last_left_motor_rpm = left_motor_rpm
        self.last_right_motor_rpm = right_motor_rpm
        self.last_speed_feedback_time = now
        self.left_rpm_pub.publish(Float32(data=float(left_motor_rpm)))
        self.right_rpm_pub.publish(Float32(data=float(right_motor_rpm)))

        if self.last_speed_odom_time is None:
            self.last_speed_odom_time = now
            return
        dt = now - self.last_speed_odom_time
        self.last_speed_odom_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        _, _, linear, angular = self.kinematics.feedback_motor_rpm_to_body_twist(
            left_motor_rpm, right_motor_rpm
        )
        self.x, self.y, self.yaw = integrate_midpoint(
            self.x, self.y, self.yaw, linear, angular, dt
        )
        self._publish_odom(linear, angular)

    def _publish_odom(self, linear: float, angular: float) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("odom_frame_id").value)
        msg.child_frame_id = str(self.get_parameter("base_frame_id").value)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.twist.twist.linear.x = float(linear)
        msg.twist.twist.angular.z = float(angular)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.03
        msg.twist.covariance[7] = 0.20
        msg.twist.covariance[35] = 0.08
        self.odom_pub.publish(msg)

    def _diagnostics_loop(self) -> None:
        now = time.monotonic()
        heartbeat_age = self._age(now, self.last_heartbeat_time)
        feedback_age = self._age(now, self.last_speed_feedback_time)
        cmd_age = self._age(now, self.last_cmd_time)

        status = DiagnosticStatus()
        status.name = "agri_chassis_can/TD48150B"
        status.hardware_id = self.interface
        faulted = bool(self.last_fault and (self.last_fault.channel_a or self.last_fault.channel_b))
        if faulted or self.estop_latched:
            status.level = DiagnosticStatus.ERROR
            status.message = "fault or E-stop"
        elif self.require_heartbeat and heartbeat_age > self.heartbeat_timeout_s:
            status.level = DiagnosticStatus.ERROR
            status.message = "heartbeat timeout"
        elif self.enabled and feedback_age > self.feedback_timeout_s:
            status.level = DiagnosticStatus.WARN
            status.message = "speed feedback stale"
        elif self.listen_only:
            status.level = DiagnosticStatus.OK
            status.message = "listen-only bring-up"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "enabled" if self.enabled else "disabled"

        def kv(key: str, value) -> KeyValue:
            return KeyValue(key=key, value=str(value))

        status.values = [
            kv("listen_only", self.listen_only),
            kv("enabled", self.enabled),
            kv("estop_latched", self.estop_latched),
            kv("heartbeat_age_s", f"{heartbeat_age:.3f}" if math.isfinite(heartbeat_age) else "inf"),
            kv("cmd_vel_age_s", f"{cmd_age:.3f}" if math.isfinite(cmd_age) else "inf"),
            kv("speed_feedback_age_s", f"{feedback_age:.3f}" if math.isfinite(feedback_age) else "inf"),
            kv("left_motor_rpm", self.last_left_motor_rpm),
            kv("right_motor_rpm", self.last_right_motor_rpm),
            kv("fault_a", None if self.last_fault is None else f"0x{self.last_fault.channel_a:04X}"),
            kv("fault_b", None if self.last_fault is None else f"0x{self.last_fault.channel_b:04X}"),
            kv("bus_voltage_raw", self.bus_voltage_raw),
            kv("temperature_raw", self.temperature_raw),
            kv("current_raw", self.current_raw),
            kv("rx_count", self.rx_count),
            kv("tx_count", self.tx_count),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)
        self.state_pub.publish(String(data=status.message))

    def destroy_node(self) -> bool:
        try:
            if not self.listen_only:
                self._send_zero()
                if self.enabled:
                    self._send(TD48150BProtocol.disable(CHANNEL_A))
                    self._send(TD48150BProtocol.disable(CHANNEL_B))
            self.transport.close()
        except Exception as exc:
            self.get_logger().warning(f"shutdown cleanup failed: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TD48150BChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
