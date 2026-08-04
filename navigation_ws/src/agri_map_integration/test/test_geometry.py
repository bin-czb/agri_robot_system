import math

import pytest

from agri_map_integration.geometry import compute_map_to_vslam


def test_identity_vslam_pose_returns_desired_map_pose():
    result = compute_map_to_vslam((4.0, 7.0, 0.5), (0.0, 0.0, 0.0))
    assert result == pytest.approx((4.0, 7.0, 0.5))


def test_alignment_reconstructs_map_base_pose():
    map_to_base = (12.0, 4.0, math.pi / 2.0)
    vslam_to_base = (2.0, 1.0, math.pi / 4.0)
    map_to_vslam = compute_map_to_vslam(map_to_base, vslam_to_base)

    yaw = map_to_vslam[2]
    reconstructed_x = (
        map_to_vslam[0]
        + math.cos(yaw) * vslam_to_base[0]
        - math.sin(yaw) * vslam_to_base[1]
    )
    reconstructed_y = (
        map_to_vslam[1]
        + math.sin(yaw) * vslam_to_base[0]
        + math.cos(yaw) * vslam_to_base[1]
    )
    reconstructed_yaw = yaw + vslam_to_base[2]

    assert reconstructed_x == pytest.approx(map_to_base[0])
    assert reconstructed_y == pytest.approx(map_to_base[1])
    assert reconstructed_yaw == pytest.approx(map_to_base[2])
