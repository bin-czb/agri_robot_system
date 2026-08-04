#!/usr/bin/env python3
"""
简单串口联调遥控节点（不直接访问串口，发布到 /chassis_control/cmd）
提供服务：/chassis_test/forward、/backward、/turn_left、/turn_right、/stop
用于验证 ROS→底盘 串口链路是否正常（需配合 chassis_controller 运行）。
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_srvs.srv import Trigger
from std_msgs.msg import Header
from trunk_interfaces.msg import ChassisControl


class ChassisSerialTeleop(Node):
    def __init__(self) -> None:
        super().__init__('chassis_serial_teleop')

        # 参数（可通过 ros2 param set 调整）
        self.declare_parameter('control_topic', '/chassis_control/cmd')
        self.declare_parameter('linear_speed', 0.05)   # m/s 低速
        self.declare_parameter('angular_speed', 0.2)   # rad/s 低速转向
        self.declare_parameter('command_duration', 1.0)  # s 维持时间
        self.declare_parameter('publish_rate_hz', 10.0)  # 发布频率

        self.control_topic = self.get_parameter('control_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.command_duration = float(self.get_parameter('command_duration').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(ChassisControl, self.control_topic, qos)

        # 当前指令状态
        self.active_until = 0.0
        self.active_linear = 0.0
        self.active_angular = 0.0

        # 发布循环
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_loop)

        # 服务接口
        self.srv_forward = self.create_service(Trigger, '/chassis_test/forward', self._srv_forward)
        self.srv_backward = self.create_service(Trigger, '/chassis_test/backward', self._srv_backward)
        self.srv_turn_left = self.create_service(Trigger, '/chassis_test/turn_left', self._srv_turn_left)
        self.srv_turn_right = self.create_service(Trigger, '/chassis_test/turn_right', self._srv_turn_right)
        self.srv_stop = self.create_service(Trigger, '/chassis_test/stop', self._srv_stop)

        self.get_logger().info(
            f'Teleop ready. topic={self.control_topic}, v={self.linear_speed} m/s, w={self.angular_speed} rad/s, duration={self.command_duration}s')

    def _activate(self, v: float, w: float, duration: float):
        self.active_linear = v
        self.active_angular = w
        self.active_until = time.time() + max(0.0, duration)

    def _publish_loop(self):
        now = time.time()
        msg = ChassisControl()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if now < self.active_until:
            msg.linear_velocity = float(self.active_linear)
            msg.angular_velocity = float(self.active_angular)
            # 规范化履带速度字段（由 chassis_controller 内部再次换算）
            msg.left_track_speed = 0.0
            msg.right_track_speed = 0.0
            msg.control_mode = 1  # 跟踪/自动模式通道
            msg.emergency_stop = False
        else:
            # 超时后持续发布停止，确保安全
            msg.linear_velocity = 0.0
            msg.angular_velocity = 0.0
            msg.left_track_speed = 0.0
            msg.right_track_speed = 0.0
            msg.control_mode = 0
            msg.emergency_stop = False

        self.pub.publish(msg)

    # 服务回调
    def _srv_forward(self, req, res):
        self._activate(self.linear_speed, 0.0, self.command_duration)
        res.success = True
        res.message = f'forward v={self.linear_speed} for {self.command_duration}s'
        return res

    def _srv_backward(self, req, res):
        self._activate(-self.linear_speed, 0.0, self.command_duration)
        res.success = True
        res.message = f'backward v={-self.linear_speed} for {self.command_duration}s'
        return res

    def _srv_turn_left(self, req, res):
        self._activate(0.0, self.angular_speed, self.command_duration)
        res.success = True
        res.message = f'turn_left w={self.angular_speed} for {self.command_duration}s'
        return res

    def _srv_turn_right(self, req, res):
        self._activate(0.0, -self.angular_speed, self.command_duration)
        res.success = True
        res.message = f'turn_right w={-self.angular_speed} for {self.command_duration}s'
        return res

    def _srv_stop(self, req, res):
        self._activate(0.0, 0.0, 0.0)
        res.success = True
        res.message = 'stop'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ChassisSerialTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


