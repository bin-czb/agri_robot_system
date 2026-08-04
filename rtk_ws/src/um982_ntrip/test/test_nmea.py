import math

import pytest

from um982_ntrip.nmea import parse_gga


def test_parse_fixed_solution_and_heights():
    fix = parse_gga(
        '$GNGGA,073512.00,3114.123456,N,12128.123456,E,4,25,0.7,'
        '15.300,M,8.200,M,1.0,0000*00')
    assert fix is not None
    assert fix.latitude == pytest.approx(31.2353909333)
    assert fix.longitude == pytest.approx(121.4687242667)
    assert fix.quality == 4
    assert fix.status_name == 'RTK_FIXED'
    assert fix.altitude_msl == pytest.approx(15.3)
    assert fix.altitude_ellipsoid == pytest.approx(23.5)


def test_parse_empty_no_fix_sentence():
    fix = parse_gga('$GNGGA,,,,,,0,,,,,,,,*78')
    assert fix is not None
    assert fix.quality == 0
    assert math.isnan(fix.latitude)
    assert math.isnan(fix.altitude_ellipsoid)


def test_unrelated_sentence_is_ignored():
    assert parse_gga('$GNRMC,073512.00,A,0,N,0,E,0,0,010101,,,A*00') is None
