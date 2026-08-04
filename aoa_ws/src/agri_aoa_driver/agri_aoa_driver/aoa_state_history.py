#!/usr/bin/env python3
"""Time-aligned UAV state buffer for AOA localization."""

from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class UavState:
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    pose_altitude_m: float


class UavStateHistory:
    def __init__(self, maxlen: int = 400) -> None:
        self._samples = deque(maxlen=max(2, int(maxlen)))

    def append(self, stamp: float, position, quaternion_xyzw, pose_altitude_m: float) -> None:
        position = np.asarray(position, dtype=float).reshape(3)
        quaternion = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
        norm = float(np.linalg.norm(quaternion))
        if not np.all(np.isfinite(position)) or not np.isfinite(pose_altitude_m):
            return
        if not np.all(np.isfinite(quaternion)) or norm < 1e-9:
            return
        self._samples.append((
            float(stamp), position.copy(), quaternion / norm,
            float(pose_altitude_m)))

    @staticmethod
    def _interpolate(first, second, alpha: float) -> UavState:
        _, position_a, quaternion_a, altitude_a = first
        _, position_b, quaternion_b, altitude_b = second
        quaternion_b = quaternion_b.copy()
        if float(np.dot(quaternion_a, quaternion_b)) < 0.0:
            quaternion_b *= -1.0
        quaternion = (1.0 - alpha) * quaternion_a + alpha * quaternion_b
        quaternion /= np.linalg.norm(quaternion)
        return UavState(
            position=(1.0 - alpha) * position_a + alpha * position_b,
            quaternion_xyzw=quaternion,
            pose_altitude_m=(1.0 - alpha) * altitude_a + alpha * altitude_b,
        )

    def sample(self, stamp: float, max_extrapolation_s: float) -> Optional[UavState]:
        if not self._samples:
            return None
        target = float(stamp)
        maximum = max(0.0, float(max_extrapolation_s))
        samples = list(self._samples)
        if target <= samples[0][0]:
            if samples[0][0] - target > maximum:
                return None
            return self._interpolate(samples[0], samples[0], 0.0)
        if target >= samples[-1][0]:
            if target - samples[-1][0] > maximum:
                return None
            return self._interpolate(samples[-1], samples[-1], 0.0)

        for first, second in zip(samples, samples[1:]):
            if first[0] <= target <= second[0]:
                duration = second[0] - first[0]
                alpha = 1.0 if duration < 1e-9 else (target - first[0]) / duration
                return self._interpolate(first, second, alpha)
        return None


def _run_selftest() -> None:
    history = UavStateHistory()
    history.append(1.0, [0, 0, 0], [0, 0, 0, 1], 0.0)
    history.append(2.0, [2, 4, 0], [0, 0, 0, 1], 2.0)
    state = history.sample(1.5, 0.2)
    assert state is not None
    assert np.allclose(state.position, [1, 2, 0])
    assert abs(state.pose_altitude_m - 1.0) < 1e-9
    assert history.sample(3.0, 0.2) is None
    print("aoa_state_history.py self-test: OK")


if __name__ == '__main__':
    _run_selftest()
