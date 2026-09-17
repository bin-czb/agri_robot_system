import math
from dataclasses import replace
from types import SimpleNamespace

import pytest

from agri_localization_selector.core import Config, Sample, Selector
from agri_localization_selector.evidence import aoa_reason, rtk_reason, uav_reason


CFG = Config(qualify_time=0.3, qualify_samples=3, recovery_time=0.8,
             min_dwell=0.5, max_age=0.25, soft_failure_time=0.15,
             max_speed=1.0, gate_distance=0.05, max_handover_jump=0.4,
             anchor_timeout=1.0)


def sample(t, x=0, variance=0.01):
    return Sample(100 + t, 'map', x, 0, (variance, 0, variance))


def step(core, t, sources=('rtk', 'aoa'), offsets=None):
    for src in sources:
        core.offer(src, sample(t, t * 0.1 + (offsets or {}).get(src, 0)), t, 100+t)
    return core.tick(t, 100+t)


def ready():
    core = Selector(CFG)
    for i in range(6):
        step(core, i * 0.1)
    assert core.active == 'rtk'
    return core


def test_boot_requires_distinct_samples_and_duration():
    core = Selector(CFG)
    assert step(core, 0) is None
    assert step(core, 0.1) is None
    assert core.tick(0.15, 100.15) is None
    assert step(core, 0.2) is None
    assert step(core, 0.31)[0] == 'rtk'


def test_hard_loss_uses_already_qualified_standby_immediately():
    core = ready()
    core.invalidate('rtk', 'rtk_not_fixed', 0.6)
    out = step(core, 0.6, ('aoa',))
    assert out[0] == 'aoa'
    assert out[1] is core.tracks['aoa'].sample


def test_cold_standby_requires_qualification():
    core = Selector(CFG)
    for i in range(6):
        step(core, i * .1, ('rtk',))
    core.invalidate('rtk', 'lost', .6)
    assert step(core, .6, ('aoa',)) is None
    for t in (.7, .8, .91):
        out = step(core, t, ('aoa',))
    assert out[0] == 'aoa'


def test_recovery_hysteresis_and_flapping():
    core = ready()
    core.invalidate('rtk', 'not_fixed', .6)
    step(core, .6, ('aoa',))
    for t in (.7, .8, .9):
        step(core, t)
        assert core.active == 'aoa'
    core.invalidate('rtk', 'not_fixed', 1.0)
    step(core, 1.0, ('aoa',))
    for i in range(11, 19):
        step(core, i * .1)
        assert core.active == 'aoa'
    step(core, 1.91)
    assert core.active == 'rtk'


def test_hard_failure_overrides_minimum_dwell():
    core = ready()
    core.cfg = replace(CFG, min_dwell=100)
    core.invalidate('rtk', 'not_fixed', .6)
    assert step(core, .6, ('aoa',))[0] == 'aoa'


def test_no_stale_republication_and_wall_timeout_when_clock_paused():
    core = ready()
    assert core.tick(.51, 100.5) is None
    assert core.tick(.8, 100.5) is None
    assert core.active is None


def test_loss_cannot_be_hidden_by_continuous_bad_packets():
    core = ready()
    for i in range(6, 12):
        t = i * .1
        core.offer('rtk', sample(t, 100), t, 100+t)
        core.invalidate('aoa', 'lost', t)
        assert core.tick(t, 100+t) is None
    assert core.active is None


def test_disagreement_is_not_blended_or_forced():
    core = Selector(CFG)
    for i in range(6):
        step(core, i * .1, offsets={'aoa': 10})
    core.invalidate('rtk', 'lost', .6)
    assert step(core, .6, ('aoa',), {'aoa': 10}) is None
    assert core.tracks['aoa'].reason == 'handover_disagreement'
    assert core.active is None


def test_anisotropic_covariance_rejects_error_in_precise_direction():
    core = Selector(CFG)
    a = Sample(100, 'map', 0, 0, (.0001, 0, .5))
    b = Sample(100, 'map', .3, 0, (.0001, 0, .5))
    assert core._comparison(a, b)[0] > CFG.gate_score


@pytest.mark.parametrize('bad,reason', [
    (replace(sample(.6), frame='utm'), 'frame_mismatch'),
    (replace(sample(.6), x=math.nan), 'nonfinite_pose'),
    (replace(sample(.6), covariance=(0, 0, 0)), 'nonpositive_covariance'),
    (replace(sample(.6), covariance=(.1, .2, .1)), 'nonpositive_covariance'),
    (replace(sample(.6), covariance=(math.nan, 0, .1)), 'nonfinite_covariance'),
    (replace(sample(.6), covariance=(2, 0, 2)), 'uncertainty_exceeds_budget'),
    (replace(sample(.6), stamp=99), 'duplicate_or_out_of_order'),
    (replace(sample(.6), stamp=110), 'measurement_time_invalid'),
])
def test_invalid_sample_revokes_qualification(bad, reason):
    core = ready()
    assert not core.offer('rtk', bad, .6, 100.6)
    assert core.tracks['rtk'].reason == reason
    assert core.tracks['rtk'].count == 0


def test_duplicate_does_not_count_as_new_evidence():
    core = Selector(CFG)
    core.offer('rtk', sample(0), 0, 100)
    assert not core.offer('rtk', sample(0), .1, 100.1)
    assert core.tick(.1, 100.1) is None


def test_recovery_after_long_blackout_requires_explicit_reset():
    core = ready()
    core.tick(3, 103)
    for i in range(31, 36):
        assert step(core, i * .1, ('aoa',)) is None
    assert core.tracks['aoa'].reason == 'relocalization_required'
    core.reset()
    for i in range(36, 42):
        out = step(core, i * .1, ('aoa',))
    assert out[0] == 'aoa'


def test_gap_restarts_warmup_even_without_intermediate_ticks():
    core = Selector(CFG)
    step(core, 0, ('aoa',))
    assert step(core, 10, ('aoa',)) is None
    assert core.tracks['aoa'].count == 1


def test_exclusive_monotonic_output_under_repeated_failures():
    core = Selector(CFG)
    stamps = []
    for i in range(200):
        t = i * .05
        sources = ('rtk', 'aoa') if i % 40 < 20 else ('aoa',)
        if sources == ('aoa',):
            core.invalidate('rtk', 'not_fixed', t)
        out = step(core, t, sources)
        if out:
            assert out[1] is core.tracks[out[0]].sample
            stamps.append(out[1].stamp)
    assert len(stamps) > 100
    assert all(a < b for a, b in zip(stamps, stamps[1:]))


def test_quality_fields_are_not_interchangeable_with_pose_freshness():
    fix = SimpleNamespace(position_valid=True, rtk_fixed=True, fix_quality=4,
                          correction_age=1., hdop=1., satellites=12)
    assert not rtk_reason(fix)
    fix.correction_age = math.nan
    assert rtk_reason(fix) == 'rtk_correction_age_invalid'
    obs = SimpleNamespace(gate_code=0, link_status=2, range_calibrated_m=10.,
                          azimuth_body_rad=0., base_yaw_enu_rad=0.)
    assert not aoa_reason(obs)
    obs.gate_code = 7
    assert aoa_reason(obs) == 'aoa_driver_rejected'
    h = {'stamp_s': 100., 'gps_age_s': .1, 'attitude_age_s': .1, 'fix_type': 6}
    assert not uav_reason(h, 100.1)
    assert uav_reason(h, 101) == 'uav_position_stale'
    h['fix_type'] = 5
    assert uav_reason(h, 100.1) == 'uav_not_rtk_fixed'


@pytest.mark.parametrize('change', [dict(max_age=0), dict(max_speed=math.nan),
                                  dict(qualify_samples=1), dict(recovery_time=.1)])
def test_invalid_configuration_fails_at_startup(change):
    with pytest.raises(ValueError):
        replace(CFG, **change)
