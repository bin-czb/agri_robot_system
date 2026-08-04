"""Planar transform helpers used by the VSLAM-to-map registration node."""

import math
from typing import Tuple


PlanarPose = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw component of a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Build a planar quaternion as x, y, z, w."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def compute_map_to_vslam(
        map_to_base: PlanarPose,
        vslam_to_base: PlanarPose) -> PlanarPose:
    """Compute map->vslam from a desired map->base and live vslam->base."""
    map_base_x, map_base_y, map_base_yaw = map_to_base
    vslam_base_x, vslam_base_y, vslam_base_yaw = vslam_to_base

    map_vslam_yaw = normalize_angle(map_base_yaw - vslam_base_yaw)
    cos_yaw = math.cos(map_vslam_yaw)
    sin_yaw = math.sin(map_vslam_yaw)
    rotated_x = cos_yaw * vslam_base_x - sin_yaw * vslam_base_y
    rotated_y = sin_yaw * vslam_base_x + cos_yaw * vslam_base_y

    return (
        map_base_x - rotated_x,
        map_base_y - rotated_y,
        map_vslam_yaw,
    )
