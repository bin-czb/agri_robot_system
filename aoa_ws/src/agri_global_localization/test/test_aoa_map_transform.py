import math

import pytest

from agri_global_localization.aoa_map_transform import (
    quaternion_to_yaw,
    rotate_pose_covariance,
    transform_utm_position,
    yaw_to_quaternion,
)


MAP_ORIGIN = (244721.8491, 3356945.1346, 0.0)


def test_map_origin_becomes_zero_in_two_d_mode():
    position = transform_utm_position(
        (MAP_ORIGIN[0], MAP_ORIGIN[1], 14.5),
        MAP_ORIGIN,
        map_yaw_rad=0.0,
        two_d_mode=True,
    )
    assert position == pytest.approx((0.0, 0.0, 0.0))


def test_known_tree_utm_matches_map_csv_coordinates():
    position = transform_utm_position(
        (244733.3603, 3356969.1984, 0.0),
        MAP_ORIGIN,
        map_yaw_rad=0.0,
        two_d_mode=True,
    )
    assert position == pytest.approx((11.5112, 24.0638, 0.0), abs=1.0e-6)


def test_map_yaw_rotates_position_and_heading():
    position = transform_utm_position(
        (11.0, 20.0, 0.0),
        (10.0, 20.0, 0.0),
        map_yaw_rad=math.pi / 2.0,
        two_d_mode=False,
    )
    assert position == pytest.approx((0.0, -1.0, 0.0), abs=1.0e-9)

    input_quaternion = yaw_to_quaternion(math.pi)
    input_yaw = quaternion_to_yaw((
        input_quaternion.x,
        input_quaternion.y,
        input_quaternion.z,
        input_quaternion.w,
    ))
    assert input_yaw - math.pi / 2.0 == pytest.approx(math.pi / 2.0)


def test_covariance_is_rotated_into_map_axes():
    covariance = [0.0] * 36
    covariance[0] = 1.0
    covariance[7] = 4.0
    covariance[35] = 0.25
    transformed = rotate_pose_covariance(covariance, math.pi / 2.0)

    assert transformed[0] == pytest.approx(4.0)
    assert transformed[7] == pytest.approx(1.0)
    assert transformed[35] == pytest.approx(0.25)
