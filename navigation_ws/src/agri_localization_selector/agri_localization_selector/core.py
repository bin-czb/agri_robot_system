"""Deterministic selection state machine, independent of ROS and hardware.

Receive/watchdog durations use a monotonic clock supplied by the caller.
Measurement stamps use the ROS time domain. Outputs are unmodified samples.
The motion test uses a reachable disk, not a statistically calibrated NIS:
relative heading and independent odometry are not assumed available.
"""

from dataclasses import dataclass
import math
from typing import Optional


SOURCES = ('rtk', 'aoa')


@dataclass(frozen=True)
class Config:
    frame: str = 'map'
    max_age: float = 0.6
    future_tolerance: float = 0.05
    qualify_time: float = 1.0
    qualify_samples: int = 5
    recovery_time: float = 3.0
    min_dwell: float = 2.0
    soft_failure_time: float = 0.3
    max_speed: float = 2.0
    gate_distance: float = 0.25
    gate_score: float = 9.21
    max_rtk_sigma: float = 0.30
    max_aoa_sigma: float = 1.0
    max_handover_jump: float = 0.75
    anchor_timeout: float = 5.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if name == 'frame':
                if not value:
                    raise ValueError('frame must not be empty')
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if type(self.qualify_samples) is not int or self.qualify_samples < 2:
            raise ValueError('qualify_samples must be an integer >= 2')
        if self.recovery_time < self.qualify_time:
            raise ValueError('recovery_time must be >= qualify_time')
        if self.anchor_timeout < self.max_age:
            raise ValueError('anchor_timeout must be >= max_age')


@dataclass(frozen=True)
class Sample:
    stamp: float
    frame: str
    x: float
    y: float
    # Full symmetric 2D position covariance (xx, xy, yy).
    covariance: tuple

    def variance(self):
        xx, xy, yy = self.covariance
        if not all(math.isfinite(v) for v in (xx, xy, yy)):
            raise ValueError('nonfinite_covariance')
        if xx <= 0 or yy <= 0 or xx * yy <= xy * xy:
            raise ValueError('nonpositive_covariance')
        return 0.5 * (xx + yy + math.hypot(xx - yy, 2 * xy))


@dataclass
class Track:
    sample: Optional[Sample] = None
    received: float = -math.inf
    seen_stamp: float = -math.inf
    good_since: Optional[float] = None
    count: int = 0
    bad_since: Optional[float] = None
    hard_bad: bool = True
    reason: str = 'waiting_for_measurement'


class Selector:
    def __init__(self, config=Config()):
        self.cfg = config
        self.tracks = {s: Track() for s in SOURCES}
        self.active = None
        self.anchor = None
        self.anchor_received = -math.inf
        self.switched_at = -math.inf
        self.last_output_stamp = -math.inf
        self.switches = 0
        self.reason = 'initializing'

    def invalidate(self, source, reason, now, hard=True):
        t = self.tracks[source]
        t.good_since, t.count = None, 0
        t.bad_since = now if t.bad_since is None else t.bad_since
        t.hard_bad = t.hard_bad or hard
        t.reason = reason

    def _comparison(self, a, b):
        """Conservative covariance ellipsoid around a motion-reachable disk.

        2*(Ca+Cb) upper-bounds difference covariance for unknown correlation.
        This avoids claiming independent evidence from shared GNSS/heading.
        """
        dx, dy = a.x - b.x, a.y - b.y
        distance = math.hypot(dx, dy)
        allowance = self.cfg.max_speed * abs(a.stamp - b.stamp)
        residual = max(0.0, distance - allowance - self.cfg.gate_distance)
        if not residual:
            return 0.0, max(0.0, distance - allowance)
        dx, dy = dx * residual / distance, dy * residual / distance
        xx, xy, yy = (2 * (u + v) for u, v in zip(a.covariance, b.covariance))
        score = (yy * dx * dx - 2 * xy * dx * dy + xx * dy * dy) / (xx * yy - xy * xy)
        return score, max(0.0, distance - allowance)

    def offer(self, source, sample, now, ros_now):
        t = self.tracks[source]
        # A late callback cannot revive an expired active source before tick()
        # notices the outage. It must qualify and pass the old-anchor gate again.
        if self.active == source and (t.hard_bad or now - t.received > self.cfg.max_age
                or (t.bad_since is not None
                    and now - t.bad_since >= self.cfg.soft_failure_time)):
            self.active = None
        try:
            if sample.frame != self.cfg.frame:
                raise ValueError('frame_mismatch')
            if not all(math.isfinite(v) for v in (sample.stamp, sample.x, sample.y)):
                raise ValueError('nonfinite_pose')
            if sample.stamp <= 0 or sample.stamp <= t.seen_stamp:
                raise ValueError('duplicate_or_out_of_order')
            if not -self.cfg.future_tolerance <= ros_now - sample.stamp <= self.cfg.max_age:
                raise ValueError('measurement_time_invalid')
            variance = sample.variance()
            limit = self.cfg.max_rtk_sigma if source == 'rtk' else self.cfg.max_aoa_sigma
            if variance > limit * limit:
                raise ValueError('uncertainty_exceeds_budget')
        except ValueError as error:
            self.invalidate(source, str(error), now)
            return False
        t.seen_stamp = sample.stamp
        if t.sample is not None and now - t.received <= self.cfg.max_age:
            if self._comparison(sample, t.sample)[0] > self.cfg.gate_score:
                self.invalidate(source, 'motion_inconsistent', now, hard=False)
                return False
        if t.good_since is None or now - t.received > self.cfg.max_age:
            t.good_since, t.count = now, 0
        t.sample, t.received = sample, now
        t.count += 1
        t.bad_since, t.hard_bad, t.reason = None, False, 'healthy'
        return True

    def _usable(self, source, now, ros_now):
        t = self.tracks[source]
        if t.sample is None or t.hard_bad:
            return False
        if now - t.received > self.cfg.max_age or not (
                -self.cfg.future_tolerance <= ros_now - t.sample.stamp <= self.cfg.max_age):
            self.invalidate(source, 'measurement_timeout', now)
            return False
        return t.bad_since is None or now - t.bad_since < self.cfg.soft_failure_time

    def _qualified(self, source, now, ros_now, recovering=False):
        t = self.tracks[source]
        duration = self.cfg.recovery_time if recovering else self.cfg.qualify_time
        return (self._usable(source, now, ros_now) and t.bad_since is None
                and t.good_since is not None and now - t.good_since >= duration
                and t.count >= self.cfg.qualify_samples)

    def _compatible(self, source, now):
        if self.tracks[source].sample.stamp <= self.last_output_stamp:
            self.tracks[source].reason = 'candidate_older_than_last_output'
            return False
        if self.anchor is None:
            return True
        t = self.tracks[source]
        if now - self.anchor_received > self.cfg.anchor_timeout:
            t.reason = 'relocalization_required'
            return False
        score, jump = self._comparison(t.sample, self.anchor)
        if jump > self.cfg.max_handover_jump or score > self.cfg.gate_score:
            t.reason = 'handover_disagreement'
            return False
        return True

    def tick(self, now, ros_now):
        """Return (source, sample) once for each new selected measurement."""
        # Expire BOTH tracks so gaps cannot count toward qualification.
        for source in SOURCES:
            self._usable(source, now, ros_now)
        if self.active and not self._usable(self.active, now, ros_now):
            self.reason = self.tracks[self.active].reason
            self.active = None
        candidate = None
        if self.active is None:
            for source in SOURCES:
                if self._qualified(source, now, ros_now) and self._compatible(source, now):
                    candidate = source
                    break
        elif (self.active == 'aoa' and now - self.switched_at >= self.cfg.min_dwell
              and self._qualified('rtk', now, ros_now, recovering=True)
              and self._compatible('rtk', now)):
            candidate = 'rtk'
        if candidate:
            self.active = candidate
            self.switched_at = now
            self.switches += 1
            self.reason = 'qualified_takeover'
        if self.active is None:
            return None
        t = self.tracks[self.active]
        # No old sample replay while a source is soft-rejected.
        if t.bad_since is not None or t.sample.stamp <= self.last_output_stamp:
            return None
        self.anchor, self.anchor_received = t.sample, t.received
        self.last_output_stamp = t.sample.stamp
        return self.active, t.sample

    def reset(self):
        """Explicit reinitialization; does not preserve an obsolete map anchor."""
        self.__init__(self.cfg)

    def status(self):
        return {
            'state': f'{self.active.upper()}_ACTIVE' if self.active else 'GLOBAL_UNAVAILABLE',
            'source': self.active or 'none', 'reason': self.reason,
            'transitions': self.switches,
            'sources': {s: {'reason': t.reason, 'qualified_samples': t.count}
                        for s, t in self.tracks.items()},
        }
