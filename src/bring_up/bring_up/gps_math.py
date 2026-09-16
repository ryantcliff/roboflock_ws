import math
from typing import Tuple


EARTH_RADIUS_M = 6378137.0


def offset_to_fix(
    origin_latitude: float,
    origin_longitude: float,
    east_m: float,
    north_m: float,
) -> Tuple[float, float]:
    latitude = origin_latitude + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = origin_longitude + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_latitude)))
    )
    return latitude, longitude
