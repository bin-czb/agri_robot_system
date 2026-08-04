#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class TreeStopDetector(Node):

    def __init__(self):
        super().__init__('tree_stop_detector')

        # ---------- 参数声明 ----------
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('left_enabled', True)
        self.declare_parameter('right_enabled', True)

        self.declare_parameter('x_min', -0.20)
        self.declare_parameter('x_max', 0.40)

        self.declare_parameter('left_y_min', 0.50)
        self.declare_parameter('left_y_max', 1.80)

        self.declare_parameter('right_y_min', -1.80)
        self.declare_parameter('right_y_max', -0.50)

        self.declare_parameter('points_threshold', 5)
        self.declare_parameter('stable_frames', 3)
        self.declare_parameter('empty_frames', 5)

        # ---------- 读取参数 ----------
        self.scan_topic = self.get_parameter('scan_topic').value

        self.left_enabled = self.get_parameter('left_enabled').value
        self.right_enabled = self.get_parameter('right_enabled').value

        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value

        self.left_y_min = self.get_parameter('left_y_min').value
        self.left_y_max = self.get_parameter('left_y_max').value

        self.right_y_min = self.get_parameter('right_y_min').value
        self.right_y_max = self.get_parameter('right_y_max').value

        self.points_threshold = self.get_parameter('points_threshold').value
        self.stable_frames = self.get_parameter('stable_frames').value
        self.empty_frames = self.get_parameter('empty_frames').value

        # ---------- 连续帧状态 ----------
        self.detect_count = 0
        self.empty_count = 0
        self.stop_active = False

        # ---------- 订阅 / 发布 ----------
        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        self.pub_stop = self.create_publisher(
            Bool,
            '/lidar/stop_flag',
            10,
        )

        self.get_logger().info(
            f'TreeStopDetector started  |  topic={self.scan_topic}  |  '
            f'window x=[{self.x_min}, {self.x_max}]  '
            f'left_y=[{self.left_y_min}, {self.left_y_max}]  '
            f'right_y=[{self.right_y_min}, {self.right_y_max}]  |  '
            f'threshold={self.points_threshold}  '
            f'stable={self.stable_frames}  empty={self.empty_frames}'
        )

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        left_points = 0
        right_points = 0

        angle = msg.angle_min

        for r in msg.ranges:
            if not math.isfinite(r):
                angle += msg.angle_increment
                continue

            if r <= msg.range_min or r >= msg.range_max:
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if self.left_enabled:
                if (self.x_min <= x <= self.x_max
                        and self.left_y_min <= y <= self.left_y_max):
                    left_points += 1

            if self.right_enabled:
                if (self.x_min <= x <= self.x_max
                        and self.right_y_min <= y <= self.right_y_max):
                    right_points += 1

            angle += msg.angle_increment

        detected = (left_points >= self.points_threshold
                    or right_points >= self.points_threshold)

        if detected:
            self.detect_count += 1
            self.empty_count = 0
        else:
            self.empty_count += 1
            self.detect_count = 0

        if self.detect_count >= self.stable_frames:
            self.stop_active = True

        if self.empty_count >= self.empty_frames:
            self.stop_active = False

        msg_out = Bool()
        msg_out.data = self.stop_active
        self.pub_stop.publish(msg_out)

        self.get_logger().debug(
            f'left={left_points}  right={right_points}  '
            f'stop={self.stop_active}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TreeStopDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
