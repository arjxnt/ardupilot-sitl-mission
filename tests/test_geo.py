import math

from missionlib.geo import bearing_point, distance_m, offset_latlon

HOME_LAT, HOME_LON = -35.363261, 149.165230


def haversine(lat1, lon1, lat2, lon2):
    r = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def test_offset_then_distance_round_trips():
    lat2, lon2 = offset_latlon(HOME_LAT, HOME_LON, north_m=1000, east_m=0)
    d = distance_m(HOME_LAT, HOME_LON, lat2, lon2)
    assert abs(d - 1000) < 0.01


def test_distance_matches_haversine_within_tolerance():
    lat2, lon2 = offset_latlon(HOME_LAT, HOME_LON, north_m=500, east_m=300)
    eq = distance_m(HOME_LAT, HOME_LON, lat2, lon2)
    hv = haversine(HOME_LAT, HOME_LON, lat2, lon2)
    assert abs(eq - hv) / hv < 0.01  # within 1% at this scale


def test_bearing_point_is_exactly_radius_away():
    for angle in range(0, 360, 15):
        lat, lon = bearing_point(HOME_LAT, HOME_LON, 15.0, angle)
        d = distance_m(lat, lon, HOME_LAT, HOME_LON)
        assert abs(d - 15.0) < 1e-4


def test_bearing_point_north_increases_latitude():
    lat, lon = bearing_point(HOME_LAT, HOME_LON, 100.0, angle_deg=0)
    assert lat > HOME_LAT
    assert abs(lon - HOME_LON) < 1e-9


def test_bearing_point_east_increases_longitude():
    lat, lon = bearing_point(HOME_LAT, HOME_LON, 100.0, angle_deg=90)
    assert lon > HOME_LON
    assert abs(lat - HOME_LAT) < 1e-9
