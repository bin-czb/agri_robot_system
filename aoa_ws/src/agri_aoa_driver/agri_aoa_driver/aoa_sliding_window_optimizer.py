#!/usr/bin/env python3
"""
Sliding-window robust least-squares backend for the AOA path.

This module is ROS-independent.  It estimates a static / low-speed
ground-base XY position in local_origin coordinates plus a short-term
AOA azimuth bias using scipy.optimize.least_squares.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np
from scipy.optimize import least_squares


_VALID_LOSSES = {"linear", "soft_l1", "huber", "cauchy", "arctan"}


def wrap_angle_rad(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AoaSlidingWindowOptimizer:
    """
    Static XY sliding-window optimizer.

    Each accepted frame contributes:
      - tag_xy_local: UAV AOA tag XY position in local_origin frame.
      - rho_m: horizontal range from base to tag.
      - bearing_rad: calibrated/disambiguated bearing from base to tag.

    State:
      [base_x_local, base_y_local, azimuth_bias_rad]
    """

    def __init__(
        self,
        window_size: int = 30,
        min_frames: int = 8,
        sigma_rho_m: float = 0.5,
        sigma_theta_rad: float = math.radians(5.0),
        sigma_beta_prior_rad: float = math.radians(15.0),
        beta_bound_rad: float = math.radians(30.0),
        loss: str = "soft_l1",
        f_scale: float = 1.0,
        max_nfev: int = 30,
    ) -> None:
        self.window_size = max(1, int(window_size))
        self.min_frames = max(1, int(min_frames))
        self.sigma_rho_m = max(float(sigma_rho_m), 1e-6)
        self.sigma_theta_rad = max(float(sigma_theta_rad), 1e-6)
        self.sigma_beta_prior_rad = max(float(sigma_beta_prior_rad), 1e-6)
        self.beta_bound_rad = max(float(beta_bound_rad), 1e-6)
        self.loss = loss if loss in _VALID_LOSSES else "soft_l1"
        self.f_scale = max(float(f_scale), 1e-6)
        self.max_nfev = max(1, int(max_nfev))

        self.frames: deque[dict] = deque(maxlen=self.window_size)
        self.last_state: Optional[np.ndarray] = None
        self.last_cost: Optional[float] = None
        self.last_success: bool = False

    def clear(self) -> None:
        self.frames.clear()
        self.last_state = None
        self.last_cost = None
        self.last_success = False

    def add_frame(self,
                  tag_xy_local: np.ndarray,
                  rho_m: float,
                  bearing_rad: float) -> None:
        self.frames.append({
            "tag_xy": np.asarray(tag_xy_local, dtype=float).reshape(2),
            "rho": float(rho_m),
            "bearing": float(bearing_rad),
        })

    def _residual(self, state: np.ndarray) -> np.ndarray:
        x_g = float(state[0])
        y_g = float(state[1])
        beta = float(state[2])
        res: list[float] = []

        for frame in self.frames:
            tag_x, tag_y = frame["tag_xy"]
            rho_meas = float(frame["rho"])
            theta_meas = float(frame["bearing"])

            dx = float(tag_x) - x_g
            dy = float(tag_y) - y_g
            rho_pred = math.hypot(dx, dy)
            theta_pred = math.atan2(dy, dx)

            res.append((rho_pred - rho_meas) / self.sigma_rho_m)
            res.append(
                wrap_angle_rad(theta_pred - (theta_meas + beta))
                / self.sigma_theta_rad
            )

        res.append(beta / self.sigma_beta_prior_rad)
        return np.asarray(res, dtype=float)

    def _initial_state(self,
                       init_xy: Optional[np.ndarray],
                       init_beta_rad: Optional[float]) -> np.ndarray:
        if init_xy is not None:
            xy0 = np.asarray(init_xy, dtype=float).reshape(2)
        elif self.last_state is not None:
            xy0 = self.last_state[:2].copy()
        else:
            frame = self.frames[-1]
            theta = float(frame["bearing"])
            rho = float(frame["rho"])
            tag_xy = frame["tag_xy"]
            xy0 = np.array([
                tag_xy[0] - rho * math.cos(theta),
                tag_xy[1] - rho * math.sin(theta),
            ], dtype=float)

        if init_beta_rad is not None:
            beta0 = float(init_beta_rad)
        elif self.last_state is not None:
            beta0 = float(self.last_state[2])
        else:
            beta0 = 0.0

        beta0 = min(max(beta0, -self.beta_bound_rad), self.beta_bound_rad)
        return np.array([xy0[0], xy0[1], beta0], dtype=float)

    def optimize(self,
                 init_xy: Optional[np.ndarray] = None,
                 init_beta_rad: Optional[float] = None) -> Optional[dict]:
        if len(self.frames) < self.min_frames:
            return None

        x0 = self._initial_state(init_xy, init_beta_rad)
        bounds = (
            np.array([-np.inf, -np.inf, -self.beta_bound_rad], dtype=float),
            np.array([np.inf, np.inf, self.beta_bound_rad], dtype=float),
        )

        try:
            result = least_squares(
                self._residual,
                x0=x0,
                bounds=bounds,
                method="trf",
                loss=self.loss,
                f_scale=self.f_scale,
                max_nfev=self.max_nfev,
            )
        except Exception:
            self.last_success = False
            return None

        if not result.success:
            self.last_success = False
            return None

        self.last_state = result.x.copy()
        self.last_cost = float(result.cost)
        self.last_success = True
        return {
            "xy": result.x[:2].copy(),
            "azimuth_bias_rad": float(result.x[2]),
            "cost": float(result.cost),
            "success": bool(result.success),
            "n_frames": len(self.frames),
        }


def _run_selftest() -> None:
    base_xy = np.array([-8.0, 0.5], dtype=float)
    beta = math.radians(4.0)
    opt = AoaSlidingWindowOptimizer(
        window_size=40,
        min_frames=8,
        sigma_rho_m=0.05,
        sigma_theta_rad=math.radians(0.5),
        sigma_beta_prior_rad=math.radians(20.0),
        beta_bound_rad=math.radians(30.0),
        loss="linear",
        max_nfev=100,
    )

    for ang in np.linspace(-0.7, 0.7, 20):
        tag_xy = np.array([
            base_xy[0] + 8.0 * math.cos(ang),
            base_xy[1] + 8.0 * math.sin(ang),
        ])
        dx = tag_xy[0] - base_xy[0]
        dy = tag_xy[1] - base_xy[1]
        rho = math.hypot(dx, dy)
        bearing_meas = wrap_angle_rad(math.atan2(dy, dx) - beta)
        opt.add_frame(tag_xy, rho, bearing_meas)

    result = opt.optimize()
    assert result is not None
    assert np.linalg.norm(result["xy"] - base_xy) < 1e-3, result
    assert abs(result["azimuth_bias_rad"] - beta) < math.radians(0.05), result
    print("aoa_sliding_window_optimizer.py self-test: OK")


if __name__ == "__main__":
    _run_selftest()
