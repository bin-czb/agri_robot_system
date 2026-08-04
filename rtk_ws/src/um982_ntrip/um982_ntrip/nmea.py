"""Small NMEA helpers used by the UM982 ROS 2 node."""

from dataclasses import dataclass
import math
from typing import Optional


QUALITY_NAMES = {
    0: 'NO_FIX',
    1: 'SINGLE',
    2: 'DGNSS',
    3: 'PPS',
    4: 'RTK_FIXED',
    5: 'RTK_FLOAT',
    6: 'DEAD_RECKONING',
}


@dataclass(frozen=True)
class GgaFix:
    """Values extracted from one GGA sentence."""

    sentence: str
    utc: str
    latitude: float
    longitude: float
    quality: int
    satellites: int
    hdop: float
    altitude_msl: float
    geoid_separation: float
    correction_age: float

    @property
    def altitude_ellipsoid(self) -> float:
        """Return WGS-84 ellipsoid height required by NavSatFix."""
        if math.isnan(self.altitude_msl) or math.isnan(self.geoid_separation):
            return math.nan
        return self.altitude_msl + self.geoid_separation

    @property
    def status_name(self) -> str:
        """Return a readable name for the NMEA quality value."""
        return QUALITY_NAMES.get(self.quality, f'QUALITY_{self.quality}')


def _float_or_nan(value: str) -> float:
    return float(value) if value else math.nan


def _int_or_zero(value: str) -> int:
    return int(value) if value else 0


def _degrees(value: str, hemisphere: str, is_longitude: bool) -> float:
    if not value:
        return math.nan
    degree_digits = 3 if is_longitude else 2
    if len(value) < degree_digits + 2:
        raise ValueError('invalid NMEA coordinate')
    degrees = int(value[:degree_digits])
    minutes = float(value[degree_digits:])
    if minutes >= 60.0:
        raise ValueError('invalid NMEA minutes')
    result = degrees + minutes / 60.0
    if hemisphere in ('S', 'W'):
        result = -result
    elif hemisphere not in ('N', 'E'):
        raise ValueError('invalid NMEA hemisphere')
    return result


def parse_gga(sentence: str) -> Optional[GgaFix]:
    """Parse a GPGGA/GNGGA sentence, returning None for unrelated input."""
    sentence = sentence.strip()
    if not sentence.startswith(('$GPGGA,', '$GNGGA,')):
        return None

    payload = sentence.split('*', 1)[0]
    fields = payload.split(',')
    if len(fields) < 15:
        raise ValueError('incomplete GGA sentence')

    latitude = _degrees(fields[2], fields[3], False)
    longitude = _degrees(fields[4], fields[5], True)
    return GgaFix(
        sentence=sentence,
        utc=fields[1],
        latitude=latitude,
        longitude=longitude,
        quality=_int_or_zero(fields[6]),
        satellites=_int_or_zero(fields[7]),
        hdop=_float_or_nan(fields[8]),
        altitude_msl=_float_or_nan(fields[9]),
        geoid_separation=_float_or_nan(fields[11]),
        correction_age=_float_or_nan(fields[13]),
    )
