"""Small WGS-84 local Cartesian conversion helpers."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple


WGS84_A_M = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_E2 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)


def _validate_geodetic(latitude_deg: float, longitude_deg: float) -> None:
    if not math.isfinite(latitude_deg) or not -90.0 <= latitude_deg <= 90.0:
        raise ValueError('latitude must be finite and within [-90, 90]')
    if not math.isfinite(longitude_deg) or not -180.0 <= longitude_deg <= 180.0:
        raise ValueError('longitude must be finite and within [-180, 180]')


def geodetic_to_ecef(
        latitude_deg: float,
        longitude_deg: float,
        altitude_m: float) -> Tuple[float, float, float]:
    """Convert WGS-84 geodetic coordinates to ECEF metres."""
    _validate_geodetic(latitude_deg, longitude_deg)
    if not math.isfinite(altitude_m):
        raise ValueError('altitude must be finite')

    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    prime_vertical = WGS84_A_M / math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude)

    x = (prime_vertical + altitude_m) * cos_latitude * cos_longitude
    y = (prime_vertical + altitude_m) * cos_latitude * sin_longitude
    z = (
        prime_vertical * (1.0 - WGS84_E2) + altitude_m
    ) * sin_latitude
    return x, y, z


@dataclass(frozen=True)
class LocalCartesian:
    """WGS-84 local east-north-up frame anchored at one geodetic datum."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def __post_init__(self) -> None:
        _validate_geodetic(self.latitude_deg, self.longitude_deg)
        if not math.isfinite(self.altitude_m):
            raise ValueError('datum altitude must be finite')

    def forward(
            self,
            latitude_deg: float,
            longitude_deg: float,
            altitude_m: float) -> Tuple[float, float, float]:
        """Return east, north and up coordinates in metres."""
        origin_x, origin_y, origin_z = geodetic_to_ecef(
            self.latitude_deg, self.longitude_deg, self.altitude_m)
        x, y, z = geodetic_to_ecef(
            latitude_deg, longitude_deg, altitude_m)
        dx = x - origin_x
        dy = y - origin_y
        dz = z - origin_z

        latitude = math.radians(self.latitude_deg)
        longitude = math.radians(self.longitude_deg)
        sin_latitude = math.sin(latitude)
        cos_latitude = math.cos(latitude)
        sin_longitude = math.sin(longitude)
        cos_longitude = math.cos(longitude)

        east = -sin_longitude * dx + cos_longitude * dy
        north = (
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz
        )
        up = (
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz
        )
        return east, north, up


class AutoDatumInitializer:
    """Require a compact cluster of fixed solutions before choosing a datum."""

    def __init__(self, sample_count: int, max_spread_m: float) -> None:
        if sample_count < 1:
            raise ValueError('sample_count must be positive')
        if max_spread_m <= 0.0:
            raise ValueError('max_spread_m must be positive')
        self._samples: Deque[Tuple[float, float, float]] = deque(
            maxlen=sample_count)
        self._sample_count = sample_count
        self._max_spread_m = max_spread_m

    @property
    def collected(self) -> int:
        return len(self._samples)

    def reset(self) -> None:
        self._samples.clear()

    def add(
            self,
            latitude_deg: float,
            longitude_deg: float,
            altitude_m: float) -> Optional[LocalCartesian]:
        _validate_geodetic(latitude_deg, longitude_deg)
        if not math.isfinite(altitude_m):
            altitude_m = 0.0
        self._samples.append((latitude_deg, longitude_deg, altitude_m))
        if len(self._samples) < self._sample_count:
            return None

        count = float(len(self._samples))
        mean_latitude = sum(item[0] for item in self._samples) / count
        mean_longitude = sum(item[1] for item in self._samples) / count
        mean_altitude = sum(item[2] for item in self._samples) / count
        candidate = LocalCartesian(
            mean_latitude, mean_longitude, mean_altitude)

        spread = max(
            math.hypot(*candidate.forward(*item)[:2])
            for item in self._samples
        )
        if spread > self._max_spread_m:
            return None
        return candidate
