import math

import pytest

from agri_map_integration.occupancy_grid_overlay import (
    occupied_cell_centers,
)


def test_unknown_and_free_cells_are_not_visualized():
    points = occupied_cell_centers(
        data=[-1, 0, 49, 50, 100, -1],
        width=3,
        height=2,
        resolution=0.5,
        origin_x=1.0,
        origin_y=2.0,
        origin_z=0.0,
        origin_yaw=0.0,
        occupied_threshold=50,
        z_offset=0.1,
    )

    assert len(points) == 2
    assert (points[0].x, points[0].y, points[0].z) == pytest.approx(
        (1.25, 2.75, 0.1))
    assert (points[1].x, points[1].y, points[1].z) == pytest.approx(
        (1.75, 2.75, 0.1))


def test_grid_origin_rotation_is_applied():
    points = occupied_cell_centers(
        data=[100],
        width=1,
        height=1,
        resolution=2.0,
        origin_x=10.0,
        origin_y=20.0,
        origin_z=0.0,
        origin_yaw=math.pi / 2.0,
        occupied_threshold=50,
    )

    assert len(points) == 1
    assert (points[0].x, points[0].y) == pytest.approx((9.0, 21.0))


def test_invalid_grid_size_is_rejected():
    with pytest.raises(ValueError):
        occupied_cell_centers(
            data=[100],
            width=2,
            height=1,
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            origin_z=0.0,
            origin_yaw=0.0,
            occupied_threshold=50,
        )

