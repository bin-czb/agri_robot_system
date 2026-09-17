"""ROS adapter tests: actual messages and node, deterministic callback clock."""
from types import SimpleNamespace
import json

import pytest
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtk_interfaces.msg import RtkFix
from trunk_interfaces.msg import AoaObservation
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agri_localization_selector.node import LocalizationSelector


@pytest.fixture
def harness(monkeypatch):
    rclpy.init()
    node = LocalizationSelector(parameter_overrides=[
        Parameter('alignment_verified', value=True),
        Parameter('alignment_id', value='synthetic-test'),
        Parameter('rtk_reference_frame', value='sensor_rig'),
        Parameter('qualify_time', value=.1),
        Parameter('qualify_samples', value=2),
    ])
    clock = [100.]
    monkeypatch.setattr('agri_localization_selector.node.time.monotonic', lambda: clock[0])
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))))
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: [object()])
    outputs, statuses = [], []
    monkeypatch.setattr(node.pose_pub, 'publish', outputs.append)
    monkeypatch.setattr(node.status_pub, 'publish', statuses.append)
    yield node, clock, outputs, statuses
    node.destroy_node()
    rclpy.shutdown()


def packet(stamp, source='rtk'):
    p = PoseWithCovarianceStamped()
    ns = round(stamp * 1e9)
    p.header.stamp.sec, p.header.stamp.nanosec = divmod(ns, 1000000000)
    p.header.frame_id = 'map'
    p.pose.pose.orientation.w = 1.
    p.pose.covariance[0] = p.pose.covariance[7] = .01
    p.pose.covariance[35] = .01
    if source == 'rtk':
        e = RtkFix()
        e.position_valid = e.rtk_fixed = True
        e.fix_quality, e.satellites = 4, 12
        e.hdop, e.correction_age = 1., 1.
    else:
        e = AoaObservation()
        e.range_calibrated_m = 10.
        e.link_status = 2
    e.header.stamp = p.header.stamp
    return p, e


def feed(h, t, source='rtk'):
    node, clock, _, _ = h
    clock[0] = t
    p, e = packet(t, source)
    node._pose(source, p)
    node._evidence(source, e)
    node._uav(String(data=json.dumps(dict(stamp_s=t, gps_age_s=.05,
                                        attitude_age_s=.05, fix_type=6))))
    node._tick()


def test_exact_matching_and_callback_order(harness):
    node, clock, out, _ = harness
    p, e = packet(100)
    node._pose('rtk', p)
    node._tick()
    assert not out
    node._evidence('rtk', e)
    node._tick()
    feed(harness, 100.15)
    assert len(out) == 1
    assert out[0].header.stamp == packet(100.15)[0].header.stamp
    assert out[0].pose.covariance[35] == 1e6


def test_unmatched_quality_cannot_validate_pose(harness):
    node, clock, out, _ = harness
    for t in (100., 100.2, 100.4):
        clock[0] = t
        node._pose('rtk', packet(t)[0])
        node._evidence('rtk', packet(t+.001)[1])
        node._tick()
    assert not out


def test_calibration_gate(harness):
    node, _, out, statuses = harness
    node.p['alignment_verified'] = False
    feed(harness, 100)
    feed(harness, 100.2)
    assert not out
    assert json.loads(statuses[-1].data)['reason'] == 'alignment_not_verified'


def test_reference_frame_gate(harness):
    node, _, out, _ = harness
    node.p['rtk_reference_frame'] = 'gps_link'
    feed(harness, 100)
    feed(harness, 100.2)
    assert not out


def test_aoa_requires_fresh_uav_quality(harness):
    node, clock, out, _ = harness
    feed(harness, 100, 'aoa')
    feed(harness, 100.2, 'aoa')
    assert node.core.active == 'aoa'
    assert out[-1].pose.covariance[0] == pytest.approx(.26)
    node._uav(String(data=json.dumps(dict(stamp_s=100.2, gps_age_s=.05,
                                        attitude_age_s=.05, fix_type=5))))
    node._tick()
    assert node.core.active is None


def test_clock_rewind_discards_all_pending_evidence(harness):
    node, clock, out, _ = harness
    feed(harness, 100)
    feed(harness, 100.2)
    assert out
    clock[0] = 90
    node._tick()
    assert node.core.active is None
    assert not node.latest_evidence
    assert not node.messages


def test_duplicate_publishers_revoke_selection(harness, monkeypatch):
    node, clock, out, statuses = harness
    feed(harness, 100)
    feed(harness, 100.2)
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: [1, 2])
    clock[0] = 100.6
    node._tick()
    assert node.core.active is None
    assert json.loads(statuses[-1].data)['reason'] == 'multiple_selected_publishers'


def test_operator_reset_clears_anchor_and_waits(harness):
    node, _, out, _ = harness
    feed(harness, 100)
    feed(harness, 100.2)
    response = node._reset(Trigger.Request(), Trigger.Response())
    assert response.success
    assert node.core.anchor is None
    assert not node.messages
