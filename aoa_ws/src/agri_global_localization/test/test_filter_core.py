import math

from agri_global_localization.filter_core import AoaComplementaryFilter


def test_initialization_and_smoothing():
    filter_ = AoaComplementaryFilter(
        position_alpha=0.25, yaw_alpha=0.5, initialization_samples=1,
        max_jump_m=2.0, min_measurement_variance=1.0)
    assert filter_.update_measurement((0.0, 0.0, 0.0), 0.0, 1.0).accepted
    assert filter_.update_measurement((1.0, 0.0, 0.0), 0.2, 2.0).accepted
    assert filter_.position == (0.25, 0.0, 0.0)
    assert math.isclose(filter_.yaw, 0.1)


def test_large_coordinate_jump_is_rejected_without_poisoning_state():
    filter_ = AoaComplementaryFilter(
        max_jump_m=2.0, max_speed_mps=1.0, jump_slack_m=0.2,
        initialization_samples=1)
    filter_.update_measurement((0.0, 0.0, 0.0), 0.0, 1.0)
    decision = filter_.update_measurement((20.0, 0.0, 0.0), 0.0, 1.1)
    assert not decision.accepted
    assert decision.reason == 'position_jump'
    assert filter_.position == (0.0, 0.0, 0.0)


def test_odometry_prediction_uses_filtered_global_heading():
    filter_ = AoaComplementaryFilter(initialization_samples=1)
    filter_.update_measurement((10.0, 20.0, 0.0), math.pi / 2.0, 1.0)
    filter_.predict_body_delta(2.0, 0.0, 0.0, 0.1)
    assert math.isclose(filter_.position[0], 10.0, abs_tol=1e-9)
    assert math.isclose(filter_.position[1], 22.0, abs_tol=1e-9)
    assert math.isclose(filter_.yaw, math.pi / 2.0 + 0.1)


def test_out_of_order_measurement_is_rejected():
    filter_ = AoaComplementaryFilter(initialization_samples=1)
    filter_.update_measurement((0.0, 0.0, 0.0), 0.0, 2.0)
    decision = filter_.update_measurement((0.1, 0.0, 0.0), 0.0, 1.0)
    assert not decision.accepted
    assert decision.reason == 'non_monotonic_stamp'


def test_initialization_requires_consistent_cluster():
    filter_ = AoaComplementaryFilter(
        initialization_samples=3, candidate_cluster_radius_m=0.3)
    first = filter_.update_measurement((0.0, 0.0, 0.0), 0.0, 1.0)
    second = filter_.update_measurement((0.1, 0.0, 0.0), 0.0, 1.1)
    third = filter_.update_measurement((0.05, 0.0, 0.0), 0.0, 1.2)
    assert first.reason == 'initializing'
    assert second.reason == 'initializing'
    assert third.accepted
    assert filter_.initialized


def test_rejected_cluster_never_auto_relocalizes():
    filter_ = AoaComplementaryFilter(
        initialization_samples=1, max_jump_m=0.5,
        candidate_cluster_radius_m=0.3)
    filter_.update_measurement((0.0, 0.0, 0.0), 0.0, 1.0)
    for index in range(20):
        decision = filter_.update_measurement(
            (10.0, 0.0, 0.0), 0.0, 1.1 + index * 0.1)
        assert not decision.accepted
    assert filter_.position == (0.0, 0.0, 0.0)
    assert filter_.candidate_count == 20
