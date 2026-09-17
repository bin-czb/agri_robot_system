"""ROS adapter: match pose/evidence timestamps before exclusive selection."""

from collections import OrderedDict
from copy import deepcopy
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtk_interfaces.msg import RtkFix
from trunk_interfaces.msg import AoaObservation
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .core import Config, Sample, Selector, SOURCES
from .evidence import aoa_reason, rtk_reason, uav_reason


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1000000000 + stamp.nanosec


def bounded_put(cache, key, value):
    cache[key] = value
    while len(cache) > 100:
        cache.popitem(last=False)


class LocalizationSelector(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_selector', **kwargs)
        defaults = {
            **vars(Config()),
            'alignment_verified': False,
            'alignment_id': '',
            'reference_frame': 'sensor_rig',
            'rtk_reference_frame': 'gps_link',
            'aoa_reference_frame': 'sensor_rig',
            'rtk_topic': '/global_pose/rtk',
            # No complementary-filter predictions or stale covariance reuse.
            'aoa_topic': '/global_pose/aoa_raw_map',
            'rtk_evidence_topic': '/rtk/fix',
            'aoa_evidence_topic': '/aoa/observation',
            'uav_health_topic': '/uav/localization_health',
            'selected_topic': '/global_pose/selected',
            'status_topic': '/global_pose/selection_status',
            'usable_topic': '/global_pose/usable',
            'max_correction_age': 5.0,
            'max_hdop': 2.5,
            'max_uav_age': 0.5,
            'max_aoa_yaw_sigma': 0.175,
            'aoa_extra_sigma': 0.5,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.core = Selector(Config(**{k: self.p[k] for k in vars(Config())}))
        for k in ('max_correction_age', 'max_hdop', 'max_uav_age', 'max_aoa_yaw_sigma'):
            if not math.isfinite(self.p[k]) or self.p[k] <= 0:
                raise ValueError(f'{k} must be finite and positive')
        if not math.isfinite(self.p['aoa_extra_sigma']) or self.p['aoa_extra_sigma'] < 0:
            raise ValueError('aoa_extra_sigma must be nonnegative and finite')
        topics = [self.p[k] for k in ('rtk_topic', 'aoa_topic', 'selected_topic')]
        if len(set(topics)) != 3:
            raise ValueError('input and output pose topics must be distinct')
        self.poses = {s: OrderedDict() for s in SOURCES}
        self.evidence = {s: OrderedDict() for s in SOURCES}
        self.latest_evidence = {}
        self.messages = {}
        self.uav_health = None
        self.last_ros = None
        self.last_state = None
        self.last_owner_check = -math.inf
        self.owner_conflict = False
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.p['selected_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.usable_pub = self.create_publisher(Bool, self.p['usable_topic'], 10)
        for source in SOURCES:
            self.create_subscription(
                PoseWithCovarianceStamped, self.p[f'{source}_topic'],
                lambda m, s=source: self._pose(s, m), 10)
        self.create_subscription(RtkFix, self.p['rtk_evidence_topic'],
                                 lambda m: self._evidence('rtk', m), 20)
        self.create_subscription(AoaObservation, self.p['aoa_evidence_topic'],
                                 lambda m: self._evidence('aoa', m), 20)
        self.create_subscription(String, self.p['uav_health_topic'], self._uav, 20)
        self.create_service(Trigger, '~/reset', self._reset)
        # Wall watchdog continues even when bag /clock stops.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self.steady_clock)

    def _pose(self, source, message):
        bounded_put(self.poses[source], stamp_ns(message), (deepcopy(message), time.monotonic()))

    def _evidence(self, source, message):
        ns = stamp_ns(message)
        now = time.monotonic()
        previous = self.latest_evidence.get(source)
        if previous and ns <= stamp_ns(previous[0]):
            self.core.invalidate(source, 'evidence_out_of_order', now)
            return
        bounded_put(self.evidence[source], ns, message)
        self.latest_evidence[source] = (message, now)

    def _uav(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('health must be an object')
            self.uav_health = (data, time.monotonic())
        except (ValueError, TypeError):
            self.uav_health = None

    def _quality(self, source, message):
        return (rtk_reason(message, self.p['max_correction_age'], self.p['max_hdop'])
                if source == 'rtk' else aoa_reason(message))

    def _alignment_reason(self):
        if not self.p['alignment_verified'] or not self.p['alignment_id'].strip():
            return 'alignment_not_verified'
        if any(self.p[f'{s}_reference_frame'] != self.p['reference_frame'] for s in SOURCES):
            return 'reference_point_mismatch'
        return ''

    def _consume(self, source, now, ros_now):
        latest = self.latest_evidence.get(source)
        if latest is None or now - latest[1] > self.core.cfg.max_age:
            return 'quality_evidence_timeout'
        if not -self.core.cfg.future_tolerance <= ros_now - stamp_ns(latest[0]) * 1e-9 <= self.core.cfg.max_age:
            return 'quality_evidence_stamp_invalid'
        if reason := self._quality(source, latest[0]):
            return reason
        if source == 'aoa':
            if self.uav_health is None or now - self.uav_health[1] > self.p['max_uav_age']:
                return 'uav_health_timeout'
            if reason := uav_reason(self.uav_health[0], ros_now, self.p['max_uav_age']):
                return reason
        for ns, (message, received) in list(self.poses[source].items()):
            if now - received > self.core.cfg.max_age:
                del self.poses[source][ns]
                continue
            evidence = self.evidence[source].get(ns)
            if evidence is None:
                continue  # bounded wait for callback ordering, never nearest-neighbour quality.
            del self.poses[source][ns]
            if reason := self._quality(source, evidence):
                self.core.invalidate(source, reason, now)
                continue
            pose, cov = message.pose.pose, message.pose.covariance
            q = pose.orientation
            values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w, *cov)
            if not all(math.isfinite(v) for v in values) or abs(
                    q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.01:
                self.core.invalidate(source, 'invalid_pose_or_quaternion', now)
                continue
            if abs(cov[1] - cov[6]) > 1e-8 or any(cov[i] < 0 for i in (0, 7, 14, 21, 28, 35)):
                self.core.invalidate(source, 'invalid_covariance', now)
                continue
            extra = 0.0
            if source == 'aoa':
                if cov[35] <= 0 or cov[35] > self.p['max_aoa_yaw_sigma']**2:
                    self.core.invalidate(source, 'aoa_heading_uncertain', now)
                    continue
                extra = self.p['aoa_extra_sigma']**2
            sample = Sample(ns * 1e-9, message.header.frame_id, pose.position.x,
                            pose.position.y, (cov[0] + extra, cov[1], cov[7] + extra))
            if self.core.offer(source, sample, received, ros_now):
                # A single-antenna RTK course is not body heading. Both modes
                # expose only x/y observations, never switch the consumer's yaw.
                message.pose.pose.position.z = 0.0
                q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
                output_cov = [0.0] * 36
                output_cov[0], output_cov[1], output_cov[6], output_cov[7] = (
                    sample.covariance[0], sample.covariance[1],
                    sample.covariance[1], sample.covariance[2])
                for index in (14, 21, 28, 35):
                    output_cov[index] = 1e6
                message.pose.covariance = output_cov
                self.messages[source] = message
        return ''

    def _tick(self):
        now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds * 1e-9
        if self.last_ros is not None and ros_now < self.last_ros - 1e-6:
            self._clear()
        self.last_ros = ros_now
        if now - self.last_owner_check >= 0.5:
            self.owner_conflict = len(self.get_publishers_info_by_topic(self.p['selected_topic'])) > 1
            self.last_owner_check = now
        blocked = 'multiple_selected_publishers' if self.owner_conflict else self._alignment_reason()
        for source in SOURCES:
            reason = blocked or self._consume(source, now, ros_now)
            if reason:
                self.core.invalidate(source, reason, now)
        output = self.core.tick(now, ros_now)
        if output:
            self.pose_pub.publish(self.messages[output[0]])
        active = self.core.active
        usable = bool(active and self.core.tracks[active].bad_since is None and not blocked)
        status = self.core.status()
        status.update({'stamp_s': ros_now, 'usable': usable,
                       'alignment_id': self.p['alignment_id'],
                       'reference_frame': self.p['reference_frame'],
                       'output_kind': 'position_measurement_not_navigation_pose'})
        if blocked:
            status['reason'] = blocked
        status['measurement_stamp_s'] = (self.core.anchor.stamp if self.core.anchor else None)
        state = (status['state'], status['reason'])
        if state != self.last_state:
            self.get_logger().info(f'Selection state: {state}')
            self.last_state = state
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.usable_pub.publish(Bool(data=usable))

    def _clear(self):
        self.core.reset()
        for caches in (self.poses, self.evidence):
            for cache in caches.values():
                cache.clear()
        self.messages.clear()
        self.latest_evidence.clear()
        self.uav_health = None

    def _reset(self, request, response):
        self._clear()
        response.success = True
        response.message = 'Cleared evidence and map anchor; waiting for fresh qualification.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.usable_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
