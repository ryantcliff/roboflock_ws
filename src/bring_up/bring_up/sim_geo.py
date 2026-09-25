"""Small geodesy helpers shared by the hardware-free simulation nodes."""
import math

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


class LocalTangent:
    """Converts east/north metres around a datum to lat/lon (and back)."""

    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon
        s2 = math.sin(math.radians(lat)) ** 2
        self.prime_vertical = WGS84_A / math.sqrt(1 - WGS84_E2 * s2)
        self.meridian = WGS84_A * (1 - WGS84_E2) / (1 - WGS84_E2 * s2) ** 1.5

    def to_latlon(self, east, north):
        lat = self.lat + math.degrees(north / self.meridian)
        lon = self.lon + math.degrees(
            east / (self.prime_vertical * math.cos(math.radians(self.lat))))
        return lat, lon

    def to_east_north(self, lat, lon):
        north = math.radians(lat - self.lat) * self.meridian
        east = math.radians(lon - self.lon) * self.prime_vertical * math.cos(
            math.radians(self.lat))
        return east, north


def utm_convergence(lat, lon):
    """Grid convergence (rad) of the UTM zone containing lat/lon."""
    zone = int((lon + 180.0) // 6.0) + 1
    central = -183.0 + 6.0 * zone
    return math.atan(math.tan(math.radians(lon - central)) * math.sin(math.radians(lat)))
