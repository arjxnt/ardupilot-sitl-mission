"""Flat-earth geo math: meters <-> lat/lon offsets and distances.

Equirectangular approximation. Accurate to well under 1% error at the
search-area scale this project operates at (hundreds of meters), and is
what ArduPilot itself uses internally for short-range navigation.
"""

import math

EARTH_RADIUS_M = 6378137.0


def offset_latlon(lat, lon, north_m, east_m):
    """Return (lat, lon) shifted by north_m/east_m meters from (lat, lon)."""
    d_lat = (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))) * (180.0 / math.pi)
    return lat + d_lat, lon + d_lon


def distance_m(lat1, lon1, lat2, lon2):
    """Equirectangular approximation distance in meters between two lat/lon points."""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    x = (math.radians(lon2) - math.radians(lon1)) * math.cos((lat1_r + lat2_r) / 2)
    y = lat2_r - lat1_r
    return math.sqrt(x * x + y * y) * EARTH_RADIUS_M


def bearing_point(center_lat, center_lon, radius_m, angle_deg):
    """Point at radius_m from center, angle_deg measured clockwise from north."""
    angle_rad = math.radians(angle_deg)
    north_m = radius_m * math.cos(angle_rad)
    east_m = radius_m * math.sin(angle_rad)
    return offset_latlon(center_lat, center_lon, north_m, east_m)
