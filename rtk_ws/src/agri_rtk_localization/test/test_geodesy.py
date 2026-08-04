import math

from agri_rtk_localization.geodesy import (
    AutoDatumInitializer,
    LocalCartesian,
)


def test_origin_is_zero():
    projection = LocalCartesian(30.0, 114.0, 50.0)
    east, north, up = projection.forward(30.0, 114.0, 50.0)
    assert math.isclose(east, 0.0, abs_tol=1e-9)
    assert math.isclose(north, 0.0, abs_tol=1e-9)
    assert math.isclose(up, 0.0, abs_tol=1e-9)


def test_small_offsets_have_enu_signs_and_scale():
    projection = LocalCartesian(30.0, 114.0, 0.0)
    east, north, _ = projection.forward(30.00001, 114.00001, 0.0)
    assert 0.9 < east < 1.1
    assert 1.0 < north < 1.2


def test_auto_datum_requires_compact_window():
    initializer = AutoDatumInitializer(sample_count=3, max_spread_m=0.2)
    assert initializer.add(30.0, 114.0, 10.0) is None
    assert initializer.add(30.0, 114.0, 10.0) is None
    assert initializer.add(30.0, 114.0, 10.0) is not None
