"""Adapt and robustly smooth AOA local poses for the navigation stack."""

import json
import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .filter_core import AoaComplementaryFilter, wrap_angle


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class AoaPoseFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('aoa_global_pose_filter')
        self.declare_parameter('input_topic', '/ground_station/absolute_pose_local_cov')
        self.declare_parameter('output_topic', '/global_pose/aoa')
        self.declare_parameter('selected_topic', '/global_pose/selected')
        self.declare_parameter('status_topic', '/global_pose/aoa/status')
        self.declare_parameter('expected_frame', 'local_origin')
        self.declare_parameter('publish_selected', True)
        self.declare_parameter('position_alpha', 0.25)
        self.declare_parameter('yaw_alpha', 0.20)
        self.declare_parameter('max_jump_m', 0.75)
        self.declare_parameter('max_speed_mps', 2.0)
        self.declare_parameter('jump_slack_m', 0.20)
        self.declare_parameter('gate_dt_max_s', 2.0)
        self.declare_parameter('innovation_sigma_limit', 3.5)
        self.declare_parameter('initialization_samples', 5)
        self.declare_parameter('candidate_cluster_radius_m', 0.50)
        self.declare_parameter('min_xy_variance', 0.04)
        self.declare_parameter('min_yaw_variance', math.radians(5.0) ** 2)
        self.declare_parameter('degraded_after_s', 1.0)
        self.declare_parameter('lost_after_s', 3.0)
        self.declare_parameter('use_odometry', False)
        self.declare_parameter('odometry_topic', '/odometry/filtered')

        self.expected_frame = str(self.get_parameter('expected_frame').value)
        self.publish_selected = bool(self.get_parameter('publish_selected').value)
        self.min_xy_variance = float(self.get_parameter('min_xy_variance').value)
        self.min_yaw_variance = float(self.get_parameter('min_yaw_variance').value)
        self.degraded_after_s = float(self.get_parameter('degraded_after_s').value)
        self.lost_after_s = float(self.get_parameter('lost_after_s').value)
        self.use_odometry = bool(self.get_parameter('use_odometry').value)
        self.filter = AoaComplementaryFilter(
            position_alpha=self.get_parameter('position_alpha').value,
            yaw_alpha=self.get_parameter('yaw_alpha').value,
            max_jump_m=self.get_parameter('max_jump_m').value,
            max_speed_mps=self.get_parameter('max_speed_mps').value,
            jump_slack_m=self.get_parameter('jump_slack_m').value,
            gate_dt_max_s=self.get_parameter('gate_dt_max_s').value,
            innovation_sigma_limit=self.get_parameter(
                'innovation_sigma_limit').value,
            min_measurement_variance=self.min_xy_variance,
            initialization_samples=self.get_parameter(
                'initialization_samples').value,
            candidate_cluster_radius_m=self.get_parameter(
                'candidate_cluster_radius_m').value,
        )

        self.accepted = 0
        self.rejected = 0
        self.last_reason = 'waiting_for_aoa'
        self.last_covariance = [0.0] * 36
        self.last_header = None
        self.last_accepted_clock_s: Optional[float] = None
        self.last_innovation_m = 0.0
        self.last_normalized_innovation = 0.0
        self.previous_odom: Optional[Tuple[float, float, float, float]] = None

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('output_topic').value), 10)
        self.selected_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('selected_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('input_topic').value), self._pose_callback, 10)
        if self.use_odometry:
            self.create_subscription(
                Odometry, str(self.get_parameter('odometry_topic').value),
                self._odometry_callback, 30)
        self.create_timer(1.0, self._publish_status)

        mode = 'AOA correction + odometry prediction' if self.use_odometry else 'AOA-only smoothing'
        self.get_logger().info(
            f'{mode}; output keeps frame {self.expected_frame!r}; this node does not publish TF')

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.expected_frame:
            self.rejected += 1
            self.last_reason = f'unexpected_frame:{msg.header.frame_id}'
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp == 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        pose = msg.pose.pose
        decision = self.filter.update_measurement(
            (pose.position.x, pose.position.y, pose.position.z),
            quaternion_to_yaw(pose.orientation), stamp,
            xy_variance=max(msg.pose.covariance[0], msg.pose.covariance[7]))
        self.last_reason = decision.reason
        self.last_innovation_m = decision.innovation_m
        self.last_normalized_innovation = decision.normalized_innovation
        if not decision.accepted:
            if decision.reason != 'initializing':
                self.rejected += 1
            if decision.reason == 'position_jump':
                self.get_logger().warning(
                    f'Rejected AOA jump {decision.innovation_m:.2f} m '
                    f'(limit {decision.allowed_m:.2f} m)',
                    throttle_duration_sec=2.0)
            return

        self.accepted += 1
        self.last_accepted_clock_s = self.get_clock().now().nanoseconds * 1e-9
        self.last_header = msg.header
        self.last_covariance = list(msg.pose.covariance)
        self._publish_pose(msg.header)

    def _odometry_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        current = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z), quaternion_to_yaw(pose.orientation))
        if self.previous_odom is None:
            self.previous_odom = current
            return
        previous = self.previous_odom
        self.previous_odom = current
        if not self.filter.initialized:
            return

        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        cosine = math.cos(previous[3])
        sine = math.sin(previous[3])
        forward = cosine * dx + sine * dy
        left = -sine * dx + cosine * dy
        self.filter.predict_body_delta(
            forward, left, current[2] - previous[2],
            wrap_angle(current[3] - previous[3]))
        if self.last_header is not None and self._health() != 'LOST':
            header = type(self.last_header)()
            header.stamp = msg.header.stamp
            header.frame_id = self.expected_frame
            self._publish_pose(header)

    def _publish_pose(self, header) -> None:
        if self.filter.position is None:
            return
        output = PoseWithCovarianceStamped()
        output.header = header
        output.header.frame_id = self.expected_frame
        output.pose.pose.position.x = self.filter.position[0]
        output.pose.pose.position.y = self.filter.position[1]
        output.pose.pose.position.z = self.filter.position[2]
        output.pose.pose.orientation = yaw_to_quaternion(self.filter.yaw)
        output.pose.covariance = list(self.last_covariance)
        output.pose.covariance[0] = max(output.pose.covariance[0], self.min_xy_variance)
        output.pose.covariance[7] = max(output.pose.covariance[7], self.min_xy_variance)
        output.pose.covariance[35] = max(
            output.pose.covariance[35], self.min_yaw_variance)
        self.pose_pub.publish(output)
        if self.publish_selected:
            self.selected_pub.publish(output)

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps({
            'source': 'aoa',
            'frame': self.expected_frame,
            'initialized': self.filter.initialized,
            'health': self._health(),
            'use_odometry': self.use_odometry,
            'accepted': self.accepted,
            'rejected': self.rejected,
            'last_reason': self.last_reason,
            'innovation_m': round(self.last_innovation_m, 4),
            'normalized_innovation': round(
                self.last_normalized_innovation, 4),
            'consecutive_rejections': self.filter.consecutive_rejections,
            'relocalization_candidate_count': self.filter.candidate_count,
        }, ensure_ascii=True, separators=(',', ':'))
        self.status_pub.publish(status)

    def _health(self) -> str:
        if not self.filter.initialized or self.last_accepted_clock_s is None:
            return 'WAITING'
        age = (
            self.get_clock().now().nanoseconds * 1e-9
            - self.last_accepted_clock_s)
        if age > self.lost_after_s:
            return 'LOST'
        if age > self.degraded_after_s or self.filter.consecutive_rejections >= 3:
            return 'DEGRADED'
        return 'NORMAL'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AoaPoseFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
