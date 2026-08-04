"""ROS-independent complementary filter used by the AOA pose adapter."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class FilterDecision:
    accepted: bool
    reason: str
    innovation_m: float = 0.0
    allowed_m: float = 0.0
    normalized_innovation: float = 0.0
    candidate_count: int = 0


class AoaComplementaryFilter:
    """Low-pass AOA position while allowing optional odometry prediction."""

    def __init__(
        self,
        position_alpha: float = 0.25,
        yaw_alpha: float = 0.20,
        max_jump_m: float = 0.75,
        max_speed_mps: float = 2.0,
        jump_slack_m: float = 0.20,
        gate_dt_max_s: float = 2.0,
        innovation_sigma_limit: float = 3.5,
        min_measurement_variance: float = 0.04,
        initialization_samples: int = 5,
        candidate_cluster_radius_m: float = 0.50,
    ) -> None:
        self.position_alpha = self._check_alpha(position_alpha, "position_alpha")
        self.yaw_alpha = self._check_alpha(yaw_alpha, "yaw_alpha")
        self.max_jump_m = max(0.0, float(max_jump_m))
        self.max_speed_mps = max(0.0, float(max_speed_mps))
        self.jump_slack_m = max(0.0, float(jump_slack_m))
        self.gate_dt_max_s = max(0.0, float(gate_dt_max_s))
        self.innovation_sigma_limit = max(0.1, float(innovation_sigma_limit))
        self.min_measurement_variance = max(1e-9, float(min_measurement_variance))
        self.initialization_samples = max(1, int(initialization_samples))
        self.candidate_cluster_radius_m = max(
            0.0, float(candidate_cluster_radius_m))
        self.position: Optional[Tuple[float, float, float]] = None
        self.yaw = 0.0
        self.last_measurement_stamp: Optional[float] = None
        self.candidate_position: Optional[Tuple[float, float, float]] = None
        self.candidate_yaw = 0.0
        self.candidate_count = 0
        self.consecutive_rejections = 0

    @staticmethod
    def _check_alpha(value: float, name: str) -> float:
        value = float(value)
        if not 0.0 < value <= 1.0:
            raise ValueError(f"{name} must be in (0, 1]")
        return value

    @property
    def initialized(self) -> bool:
        return self.position is not None

    def reset(self) -> None:
        self.position = None
        self.yaw = 0.0
        self.last_measurement_stamp = None
        self.candidate_position = None
        self.candidate_yaw = 0.0
        self.candidate_count = 0
        self.consecutive_rejections = 0

    def _update_candidate(self, values, yaw: float) -> None:
        if self.candidate_position is None:
            self.candidate_position = values
            self.candidate_yaw = yaw
            self.candidate_count = 1
            return
        distance = math.hypot(
            values[0] - self.candidate_position[0],
            values[1] - self.candidate_position[1])
        if distance > self.candidate_cluster_radius_m:
            self.candidate_position = values
            self.candidate_yaw = yaw
            self.candidate_count = 1
            return
        count = self.candidate_count + 1
        alpha = 1.0 / count
        self.candidate_position = tuple(
            previous + alpha * (measurement - previous)
            for previous, measurement in zip(self.candidate_position, values)
        )
        self.candidate_yaw = wrap_angle(
            self.candidate_yaw
            + alpha * wrap_angle(yaw - self.candidate_yaw))
        self.candidate_count = count

    def _clear_candidate(self) -> None:
        self.candidate_position = None
        self.candidate_yaw = 0.0
        self.candidate_count = 0

    def update_measurement(
        self,
        position: Sequence[float],
        yaw: float,
        stamp: float,
        xy_variance: Optional[float] = None,
    ) -> FilterDecision:
        values = tuple(float(v) for v in position)
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            return FilterDecision(False, "non_finite_position")
        if not math.isfinite(yaw) or not math.isfinite(stamp):
            return FilterDecision(False, "non_finite_orientation_or_stamp")

        yaw = wrap_angle(yaw)
        if self.position is None:
            self._update_candidate(values, yaw)
            if self.candidate_count < self.initialization_samples:
                return FilterDecision(
                    False, "initializing", candidate_count=self.candidate_count)
            self.position = self.candidate_position
            self.yaw = self.candidate_yaw
            self.last_measurement_stamp = stamp
            self._clear_candidate()
            return FilterDecision(True, "initialized")

        if self.last_measurement_stamp is not None and stamp <= self.last_measurement_stamp:
            return FilterDecision(False, "non_monotonic_stamp")

        dx = values[0] - self.position[0]
        dy = values[1] - self.position[1]
        innovation = math.hypot(dx, dy)
        dt = stamp - self.last_measurement_stamp
        allowed = self.max_jump_m
        if dt <= self.gate_dt_max_s:
            allowed = min(allowed, self.jump_slack_m + self.max_speed_mps * dt)
        variance = self.min_measurement_variance
        if xy_variance is not None and math.isfinite(float(xy_variance)):
            variance = max(variance, float(xy_variance))
        normalized = innovation / math.sqrt(variance)
        if innovation > allowed or normalized > self.innovation_sigma_limit:
            self.consecutive_rejections += 1
            self._update_candidate(values, yaw)
            return FilterDecision(
                False, "position_jump", innovation, allowed, normalized,
                self.candidate_count)

        a = self.position_alpha
        self.position = tuple(
            previous + a * (measurement - previous)
            for previous, measurement in zip(self.position, values)
        )
        self.yaw = wrap_angle(self.yaw + self.yaw_alpha * wrap_angle(yaw - self.yaw))
        self.last_measurement_stamp = stamp
        self.consecutive_rejections = 0
        self._clear_candidate()
        return FilterDecision(True, "updated", innovation, allowed, normalized)

    def predict_body_delta(self, forward: float, left: float, up: float, dyaw: float) -> None:
        """Apply an odometry increment expressed in the previous body frame."""
        if self.position is None:
            return
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        east = cosine * forward - sine * left
        north = sine * forward + cosine * left
        self.position = (
            self.position[0] + east,
            self.position[1] + north,
            self.position[2] + up,
        )
        self.yaw = wrap_angle(self.yaw + dyaw)
