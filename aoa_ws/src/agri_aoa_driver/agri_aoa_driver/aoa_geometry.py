#!/usr/bin/env python3
"""Pure geometry helpers for the ground AOA / UAV tag arrangement.

Coordinate conventions:
  * body: ROS FLU (x forward, y left, z up)
  * world: local ENU (x east, y north, z up)
  * yaw: counter-clockwise from East

This module deliberately has no ROS dependency so the coordinate transforms can
be regression-tested without a serial device, camera, or running ROS graph.
"""

import math
from typing import Iterable

import numpy as np


def wrap_angle_rad(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def body_bearing_to_enu(body_bearing_rad: float,
                        base_yaw_enu_rad: float) -> float:
    """Rotate an AOA bearing from the base body frame into ENU."""
    return wrap_angle_rad(body_bearing_rad + base_yaw_enu_rad)


def rotate_body_xy_to_enu(vector_body: Iterable[float],
                          base_yaw_enu_rad: float) -> np.ndarray:
    """Yaw-rotate a ROS FLU vector into ENU.

    The ground sensor rig is assumed level for the AOA horizontal bearing.  The
    z component is retained unchanged.  Roll/pitch remain an upstream state
    estimation concern and are not observable from the current AOA packet.
    """
    vector = np.asarray(vector_body, dtype=float).reshape(3)
    c = math.cos(base_yaw_enu_rad)
    s = math.sin(base_yaw_enu_rad)
    return np.array([
        c * vector[0] - s * vector[1],
        s * vector[0] + c * vector[1],
        vector[2],
    ], dtype=float)


def solve_ground_base_position(
        uav_rtk_abs_enu: Iterable[float],
        tag_offset_enu: Iterable[float],
        horizontal_range_m: float,
        vertical_tag_minus_aoa_m: float,
        bearing_enu_rad: float,
        base_to_aoa_body_m: Iterable[float],
        base_yaw_enu_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """Solve the ground ``base_link`` position from one AOA observation.

    Returns ``(base_abs_enu, uav_rtk_rel_base_enu)``.  The relative vector is
    expressed in ENU and points from the ground ``base_link`` reference point
    to the UAV RTK antenna.
    """
    uav_rtk_abs = np.asarray(uav_rtk_abs_enu, dtype=float).reshape(3)
    tag_offset = np.asarray(tag_offset_enu, dtype=float).reshape(3)
    base_to_aoa_enu = rotate_body_xy_to_enu(
        base_to_aoa_body_m, base_yaw_enu_rad)

    aoa_to_tag_enu = np.array([
        float(horizontal_range_m) * math.cos(bearing_enu_rad),
        float(horizontal_range_m) * math.sin(bearing_enu_rad),
        float(vertical_tag_minus_aoa_m),
    ], dtype=float)

    tag_abs = uav_rtk_abs + tag_offset
    aoa_abs = tag_abs - aoa_to_tag_enu
    base_abs = aoa_abs - base_to_aoa_enu
    uav_rtk_rel_base = uav_rtk_abs - base_abs
    return base_abs, uav_rtk_rel_base


def _assert_close(actual: np.ndarray, expected: Iterable[float]) -> None:
    if not np.allclose(actual, np.asarray(expected), atol=1e-9):
        raise AssertionError(f"actual={actual}, expected={expected}")


def _self_test() -> None:
    # The same body-forward AOA reading follows the cart as it turns.
    assert math.isclose(
        body_bearing_to_enu(0.0, math.pi / 2.0), math.pi / 2.0)
    assert math.isclose(
        body_bearing_to_enu(-math.pi / 2.0, math.pi / 2.0), 0.0)

    base_abs, uav_rel = solve_ground_base_position(
        uav_rtk_abs_enu=[10.0, 0.0, 2.0],
        tag_offset_enu=[0.0, 0.0, 0.0],
        horizontal_range_m=5.0,
        vertical_tag_minus_aoa_m=2.0,
        bearing_enu_rad=0.0,
        base_to_aoa_body_m=[0.0, 0.0, 0.0],
        base_yaw_enu_rad=0.0,
    )
    _assert_close(base_abs, [5.0, 0.0, 0.0])
    _assert_close(uav_rel, [5.0, 0.0, 2.0])

    # Cart points North; AOA is 0.2 m ahead of base_link.  A forward
    # reading therefore lies on +North and the lever arm is also +North.
    base_abs, uav_rel = solve_ground_base_position(
        uav_rtk_abs_enu=[0.0, 5.2, 2.0],
        tag_offset_enu=[0.0, 0.0, 0.0],
        horizontal_range_m=5.0,
        vertical_tag_minus_aoa_m=2.0,
        bearing_enu_rad=math.pi / 2.0,
        base_to_aoa_body_m=[0.2, 0.0, 0.0],
        base_yaw_enu_rad=math.pi / 2.0,
    )
    _assert_close(base_abs, [0.0, 0.0, 0.0])
    _assert_close(uav_rel, [0.0, 5.2, 2.0])

    # Rotating the rig changes both body bearing and lever-arm direction,
    # but the recovered base_link position must remain invariant.
    truth_base = np.array([100.0, 50.0, 0.0])
    uav_abs = np.array([110.0, 50.0, 2.0])
    lever_body = np.array([0.2, 0.0, 0.19])
    for base_yaw in (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0):
        aoa_abs = truth_base + rotate_body_xy_to_enu(
            lever_body, base_yaw)
        aoa_to_tag = uav_abs - aoa_abs
        rho = float(np.hypot(aoa_to_tag[0], aoa_to_tag[1]))
        theta_enu = math.atan2(aoa_to_tag[1], aoa_to_tag[0])
        theta_body = wrap_angle_rad(theta_enu - base_yaw)
        solved, _ = solve_ground_base_position(
            uav_rtk_abs_enu=uav_abs,
            tag_offset_enu=[0.0, 0.0, 0.0],
            horizontal_range_m=rho,
            vertical_tag_minus_aoa_m=aoa_to_tag[2],
            bearing_enu_rad=body_bearing_to_enu(theta_body, base_yaw),
            base_to_aoa_body_m=lever_body,
            base_yaw_enu_rad=base_yaw,
        )
        _assert_close(solved, truth_base)

    print("aoa_geometry.py self-test: OK")


if __name__ == '__main__':
    _self_test()
