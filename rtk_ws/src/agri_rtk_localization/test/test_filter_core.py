import math

from agri_rtk_localization.filter_core import RtkPositionFilter


def test_filter_initializes_and_smooths():
    filter_ = RtkPositionFilter(position_alpha=0.5)
    first = filter_.update((0.0, 0.0, 0.0), 1.0)
    second = filter_.update((0.2, 0.0, 0.0), 1.1)
    assert first.accepted
    assert second.accepted
    assert math.isclose(second.position[0], 0.1)


def test_large_jump_is_rejected_without_poisoning_state():
    filter_ = RtkPositionFilter(
        position_alpha=1.0,
        max_jump_m=3.0,
        max_speed_mps=2.0,
        jump_slack_m=0.3)
    filter_.update((0.0, 0.0, 0.0), 1.0)
    rejected = filter_.update((10.0, 0.0, 0.0), 1.1)
    accepted = filter_.update((0.2, 0.0, 0.0), 1.2)
    assert not rejected.accepted
    assert rejected.reason == 'position_jump'
    assert accepted.accepted
    assert accepted.position == (0.2, 0.0, 0.0)


def test_out_of_order_fix_is_rejected():
    filter_ = RtkPositionFilter()
    filter_.update((0.0, 0.0, 0.0), 2.0)
    decision = filter_.update((0.1, 0.0, 0.0), 1.0)
    assert not decision.accepted
    assert decision.reason == 'non_monotonic_stamp'
