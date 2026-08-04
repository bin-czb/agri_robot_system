"""Convert RTK NavSatFix measurements to local navigation and RViz outputs."""

from collections import deque
import json
import math
from typing import Deque, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .filter_core import RtkPositionFilter
from .geodesy import AutoDatumInitializer, LocalCartesian


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


class RtkLocalizer(Node):
    """Publish a gated local RTK pose, odometry, marker and bounded path."""

    def __init__(self, **kwargs) -> None:
        super().__init__('rtk_global_localizer', **kwargs)
        self._declare_parameters()

        self.input_fix_topic = str(self.get_parameter('input_fix_topic').value)
        self.input_quality_topic = str(
            self.get_parameter('input_quality_topic').value)
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.require_rtk_fixed = bool(
            self.get_parameter('require_rtk_fixed').value)
        self.fixed_quality = int(self.get_parameter('fixed_quality').value)
        self.quality_timeout_s = float(
            self.get_parameter('quality_timeout_s').value)
        self.two_d_mode = bool(self.get_parameter('two_d_mode').value)
        self.publish_selected = bool(
            self.get_parameter('publish_selected').value)
        self.course_min_distance_m = float(
            self.get_parameter('course_min_distance_m').value)
        self.path_min_distance_m = float(
            self.get_parameter('path_min_distance_m').value)
        self.path_max_poses = int(
            self.get_parameter('path_max_poses').value)
        self.fix_timeout_s = float(
            self.get_parameter('fix_timeout_s').value)

        self.filter = RtkPositionFilter(
            position_alpha=float(
                self.get_parameter('position_alpha').value),
            max_jump_m=float(self.get_parameter('max_jump_m').value),
            max_speed_mps=float(
                self.get_parameter('max_speed_mps').value),
            jump_slack_m=float(
                self.get_parameter('jump_slack_m').value),
            gate_dt_max_s=float(
                self.get_parameter('gate_dt_max_s').value),
        )
        self.datum_mode = str(self.get_parameter('datum_mode').value).lower()
        self.datum_initializer = AutoDatumInitializer(
            sample_count=int(
                self.get_parameter('auto_datum_samples').value),
            max_spread_m=float(
                self.get_parameter('auto_datum_max_spread_m').value),
        )
        self.projection: Optional[LocalCartesian] = None
        if self.datum_mode == 'manual':
            self.projection = LocalCartesian(
                float(self.get_parameter('datum_latitude').value),
                float(self.get_parameter('datum_longitude').value),
                float(self.get_parameter('datum_altitude').value),
            )
        elif self.datum_mode != 'auto':
            raise ValueError('datum_mode must be auto or manual')

        self.latest_quality: Optional[int] = None
        self.latest_quality_clock_s: Optional[float] = None
        self.last_fix_clock_s: Optional[float] = None
        self.last_reason = 'waiting_for_fix'
        self.accepted = 0
        self.rejected = 0
        self.last_yaw = 0.0
        self.course_valid = False
        self.course_reference: Optional[Tuple[float, float]] = None
        self.path: Deque[PoseStamped] = deque(maxlen=self.path_max_poses)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value), 10)
        self.selected_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter('selected_topic').value), 10)
        self.odometry_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter('odometry_topic').value), 10)
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), latched_qos)
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('marker_topic').value), latched_qos)
        self.datum_pub = self.create_publisher(
            GeoPointStamped,
            str(self.get_parameter('datum_topic').value), latched_qos)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('diagnostics_topic').value), 10)

        self.create_subscription(
            NavSatFix, self.input_fix_topic,
            self._fix_callback, qos_profile_sensor_data)
        self.create_subscription(
            UInt8, self.input_quality_topic, self._quality_callback, 10)
        self.create_service(
            Trigger, '~/reset_datum', self._reset_datum_callback)
        self.create_service(
            Trigger, '~/clear_path', self._clear_path_callback)
        self.create_timer(1.0, self._publish_diagnostics)

        if self.projection is not None:
            self._publish_datum()
        self.get_logger().info(
            f'RTK localizer: {self.input_fix_topic} -> '
            f'{self.get_parameter("pose_topic").value}, '
            f'frame={self.output_frame}, datum={self.datum_mode}; no TF output')

    def _declare_parameters(self) -> None:
        defaults = {
            'input_fix_topic': '/fix',
            'input_quality_topic': '/rtk/fix_quality',
            'pose_topic': '/global_pose/rtk',
            'selected_topic': '/global_pose/selected',
            'odometry_topic': '/odometry/rtk',
            'path_topic': '/rtk/path',
            'marker_topic': '/rtk/marker',
            'datum_topic': '/rtk/datum',
            'diagnostics_topic': '/rtk/diagnostics',
            'output_frame': 'local_origin',
            'child_frame': 'gps_link',
            'publish_selected': True,
            'require_rtk_fixed': True,
            'fixed_quality': 4,
            'quality_timeout_s': 2.0,
            'datum_mode': 'auto',
            'datum_latitude': 0.0,
            'datum_longitude': 0.0,
            'datum_altitude': 0.0,
            'auto_datum_samples': 20,
            'auto_datum_max_spread_m': 0.20,
            'two_d_mode': True,
            'position_alpha': 0.8,
            'max_jump_m': 3.0,
            'max_speed_mps': 2.0,
            'jump_slack_m': 0.30,
            'gate_dt_max_s': 1.0,
            'course_min_distance_m': 0.30,
            'path_min_distance_m': 0.05,
            'path_max_poses': 2000,
            'fix_timeout_s': 2.0,
            'fixed_horizontal_sigma_m': 0.03,
            'fixed_vertical_sigma_m': 0.08,
            'float_horizontal_sigma_m': 0.50,
            'float_vertical_sigma_m': 1.00,
            'other_horizontal_sigma_m': 3.00,
            'other_vertical_sigma_m': 5.00,
            'course_yaw_variance': 0.25,
            'unknown_yaw_variance': 1000000.0,
            'marker_scale_m': 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _quality_callback(self, message: UInt8) -> None:
        self.latest_quality = int(message.data)
        self.latest_quality_clock_s = (
            self.get_clock().now().nanoseconds * 1e-9)

    def _quality_is_accepted(self, now_s: float) -> Tuple[bool, str]:
        if not self.require_rtk_fixed:
            return True, 'quality_check_disabled'
        if (
                self.latest_quality is None
                or self.latest_quality_clock_s is None
                or now_s - self.latest_quality_clock_s
                > self.quality_timeout_s):
            return False, 'quality_unavailable'
        if self.latest_quality != self.fixed_quality:
            return False, f'not_rtk_fixed:{self.latest_quality}'
        return True, 'rtk_fixed'

    def _fix_callback(self, message: NavSatFix) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if message.status.status == NavSatStatus.STATUS_NO_FIX:
            self._reject('navsat_no_fix')
            return
        if not (
                math.isfinite(message.latitude)
                and math.isfinite(message.longitude)
                and -90.0 <= message.latitude <= 90.0
                and -180.0 <= message.longitude <= 180.0):
            self._reject('invalid_geodetic_position')
            return

        quality_ok, reason = self._quality_is_accepted(now_s)
        if not quality_ok:
            self._reject(reason)
            return

        altitude = (
            message.altitude if math.isfinite(message.altitude) else 0.0)
        if self.projection is None:
            self.projection = self.datum_initializer.add(
                message.latitude, message.longitude, altitude)
            if self.projection is None:
                self.last_reason = (
                    f'initializing_datum:{self.datum_initializer.collected}')
                return
            self._publish_datum()
            self.get_logger().info(
                'RTK auto datum initialized: '
                f'{self.projection.latitude_deg:.9f}, '
                f'{self.projection.longitude_deg:.9f}, '
                f'{self.projection.altitude_m:.3f}')

        if not math.isfinite(message.altitude):
            altitude = self.projection.altitude_m
        east, north, up = self.projection.forward(
            message.latitude, message.longitude, altitude)
        measurement = (east, north, 0.0 if self.two_d_mode else up)

        measurement_stamp_s = stamp_seconds(message.header.stamp)
        if measurement_stamp_s <= 0.0:
            message.header.stamp = self.get_clock().now().to_msg()
            measurement_stamp_s = stamp_seconds(message.header.stamp)
        decision = self.filter.update(measurement, measurement_stamp_s)
        if not decision.accepted or decision.position is None:
            self._reject(
                f'{decision.reason}:'
                f'{decision.innovation_m:.3f}/{decision.allowed_m:.3f}')
            return

        self.accepted += 1
        self.last_reason = decision.reason
        self.last_fix_clock_s = now_s
        yaw = self._update_course(decision.position)
        pose = self._build_pose(message, decision.position, yaw)
        self.pose_pub.publish(pose)
        if self.publish_selected:
            self.selected_pub.publish(pose)
        self.odometry_pub.publish(self._build_odometry(pose))
        self._publish_path(pose)
        self._publish_marker(pose)

    def _update_course(self, position: Tuple[float, float, float]) -> float:
        xy = (position[0], position[1])
        if self.course_reference is None:
            self.course_reference = xy
            return self.last_yaw
        dx = xy[0] - self.course_reference[0]
        dy = xy[1] - self.course_reference[1]
        if math.hypot(dx, dy) >= self.course_min_distance_m:
            self.last_yaw = math.atan2(dy, dx)
            self.course_valid = True
            self.course_reference = xy
        return self.last_yaw

    def _build_pose(
            self,
            fix: NavSatFix,
            position: Tuple[float, float, float],
            yaw: float) -> PoseWithCovarianceStamped:
        output = PoseWithCovarianceStamped()
        output.header.stamp = fix.header.stamp
        output.header.frame_id = self.output_frame
        output.pose.pose.position = Point(
            x=position[0], y=position[1], z=position[2])
        output.pose.pose.orientation = yaw_to_quaternion(yaw)

        covariance = [0.0] * 36
        xy_variance, z_variance = self._position_variances(fix)
        covariance[0] = xy_variance
        covariance[7] = xy_variance
        covariance[14] = z_variance
        covariance[21] = 1000000.0
        covariance[28] = 1000000.0
        covariance[35] = float(self.get_parameter(
            'course_yaw_variance' if self.course_valid
            else 'unknown_yaw_variance').value)
        output.pose.covariance = covariance
        return output

    def _position_variances(self, fix: NavSatFix) -> Tuple[float, float]:
        if (
                fix.position_covariance_type
                != NavSatFix.COVARIANCE_TYPE_UNKNOWN
                and fix.position_covariance[0] > 0.0
                and fix.position_covariance[4] > 0.0):
            xy_variance = max(
                float(fix.position_covariance[0]),
                float(fix.position_covariance[4]))
            z_variance = max(float(fix.position_covariance[8]), xy_variance)
            return xy_variance, z_variance

        quality = self.latest_quality if self.latest_quality is not None else 0
        if quality == 4:
            xy_sigma = float(
                self.get_parameter('fixed_horizontal_sigma_m').value)
            z_sigma = float(
                self.get_parameter('fixed_vertical_sigma_m').value)
        elif quality == 5:
            xy_sigma = float(
                self.get_parameter('float_horizontal_sigma_m').value)
            z_sigma = float(
                self.get_parameter('float_vertical_sigma_m').value)
        else:
            xy_sigma = float(
                self.get_parameter('other_horizontal_sigma_m').value)
            z_sigma = float(
                self.get_parameter('other_vertical_sigma_m').value)
        return xy_sigma * xy_sigma, z_sigma * z_sigma

    def _build_odometry(
            self, pose: PoseWithCovarianceStamped) -> Odometry:
        odometry = Odometry()
        odometry.header = pose.header
        odometry.child_frame_id = self.child_frame
        odometry.pose = pose.pose
        odometry.twist.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            odometry.twist.covariance[index] = 1000000.0
        return odometry

    def _publish_path(self, pose: PoseWithCovarianceStamped) -> None:
        stamped_pose = PoseStamped()
        stamped_pose.header = pose.header
        stamped_pose.pose = pose.pose.pose
        if self.path:
            previous = self.path[-1].pose.position
            distance = math.hypot(
                stamped_pose.pose.position.x - previous.x,
                stamped_pose.pose.position.y - previous.y)
            if distance < self.path_min_distance_m:
                return
        self.path.append(stamped_pose)
        path = Path()
        path.header = pose.header
        path.poses = list(self.path)
        self.path_pub.publish(path)

    def _publish_marker(self, pose: PoseWithCovarianceStamped) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'rtk'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        scale = float(self.get_parameter('marker_scale_m').value)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.10
        marker.color.g = 0.90
        marker.color.b = 0.20
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _publish_datum(self) -> None:
        if self.projection is None:
            return
        datum = GeoPointStamped()
        datum.header.stamp = self.get_clock().now().to_msg()
        datum.header.frame_id = 'wgs84'
        datum.position.latitude = self.projection.latitude_deg
        datum.position.longitude = self.projection.longitude_deg
        datum.position.altitude = self.projection.altitude_m
        self.datum_pub.publish(datum)

    def _reject(self, reason: str) -> None:
        self.rejected += 1
        self.last_reason = reason
        self.get_logger().warning(
            f'RTK measurement rejected: {reason}',
            throttle_duration_sec=2.0)

    def _publish_diagnostics(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        age = (
            math.inf if self.last_fix_clock_s is None
            else max(0.0, now_s - self.last_fix_clock_s))
        status = DiagnosticStatus()
        status.name = 'agri_rtk_localization'
        status.hardware_id = self.child_frame
        if self.projection is None:
            status.level = DiagnosticStatus.STALE
            status.message = 'WAITING_DATUM'
        elif age > self.fix_timeout_s:
            status.level = DiagnosticStatus.ERROR
            status.message = 'LOST'
        elif self.last_reason.startswith(('not_rtk_fixed', 'position_jump')):
            status.level = DiagnosticStatus.WARN
            status.message = 'DEGRADED'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'RTK_FIXED'
        values = {
            'frame': self.output_frame,
            'datum_mode': self.datum_mode,
            'quality': self.latest_quality,
            'accepted': self.accepted,
            'rejected': self.rejected,
            'last_reason': self.last_reason,
            'last_accepted_age_s': None if math.isinf(age) else round(age, 3),
            'path_poses': len(self.path),
            'course_valid': self.course_valid,
        }
        status.values = [
            KeyValue(key=key, value=json.dumps(value, ensure_ascii=True))
            for key, value in values.items()
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)

    def _reset_datum_callback(self, request, response):
        del request
        self.filter.reset()
        self.datum_initializer.reset()
        self.path.clear()
        self.course_reference = None
        self.course_valid = False
        self.last_fix_clock_s = None
        if self.datum_mode == 'auto':
            self.projection = None
            response.message = '已清除自动 datum、滤波状态和轨迹'
        else:
            response.message = '已重置滤波状态和轨迹；保留人工 datum'
            self._publish_datum()
        response.success = True
        return response

    def _clear_path_callback(self, request, response):
        del request
        self.path.clear()
        response.success = True
        response.message = '已清除 RTK 移动轨迹'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtkLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
