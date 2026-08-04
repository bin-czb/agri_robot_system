"""Navigation-side RTK position gate and light smoothing."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class FilterDecision:
    accepted: bool
    reason: str
    position: Optional[Tuple[float, float, float]]
    innovation_m: float
    allowed_m: float


class RtkPositionFilter:
    """Reject non-physical jumps without hiding sustained sensor failure."""

    def __init__(
            self,
            position_alpha: float = 0.8,
            max_jump_m: float = 3.0,
            max_speed_mps: float = 2.0,
            jump_slack_m: float = 0.30,
            gate_dt_max_s: float = 1.0) -> None:
        if not 0.0 < position_alpha <= 1.0:
            raise ValueError('position_alpha must be within (0, 1]')
        if min(max_jump_m, max_speed_mps, jump_slack_m, gate_dt_max_s) < 0.0:
            raise ValueError('gate parameters must be non-negative')
        self.position_alpha = position_alpha
        self.max_jump_m = max_jump_m
        self.max_speed_mps = max_speed_mps
        self.jump_slack_m = jump_slack_m
        self.gate_dt_max_s = gate_dt_max_s
        self.position: Optional[Tuple[float, float, float]] = None
        self.stamp_s: Optional[float] = None

    def reset(self) -> None:
        self.position = None
        self.stamp_s = None

    def update(
            self,
            measurement: Tuple[float, float, float],
            stamp_s: float) -> FilterDecision:
        if not math.isfinite(stamp_s) or not all(
                math.isfinite(value) for value in measurement):
            return FilterDecision(
                False, 'non_finite', self.position, math.nan, 0.0)

        if self.position is None or self.stamp_s is None:
            self.position = measurement
            self.stamp_s = stamp_s
            return FilterDecision(True, 'initialized', self.position, 0.0, 0.0)

        dt = stamp_s - self.stamp_s
        if dt <= 0.0:
            return FilterDecision(
                False, 'non_monotonic_stamp', self.position, math.nan, 0.0)

        innovation = math.sqrt(sum(
            (measurement[index] - self.position[index]) ** 2
            for index in range(3)
        ))
        allowed = min(
            self.max_jump_m,
            self.jump_slack_m
            + self.max_speed_mps * min(dt, self.gate_dt_max_s),
        )
        if innovation > allowed:
            return FilterDecision(
                False, 'position_jump', self.position, innovation, allowed)

        alpha = self.position_alpha
        self.position = tuple(
            alpha * measurement[index]
            + (1.0 - alpha) * self.position[index]
            for index in range(3)
        )
        self.stamp_s = stamp_s
        return FilterDecision(
            True, 'accepted', self.position, innovation, allowed)
