from missionlib.geo import distance_m
from missionlib.patterns import expanding_square, lawnmower

HOME_LAT, HOME_LON = -35.363261, 149.165230


def test_expanding_square_leg_lengths_grow_in_pairs():
    waypoints = expanding_square(HOME_LAT, HOME_LON, leg_unit_m=40.0, num_legs=8)
    prev = (HOME_LAT, HOME_LON)
    lengths = []
    for lat, lon in waypoints:
        lengths.append(distance_m(prev[0], prev[1], lat, lon))
        prev = (lat, lon)
    expected = [40, 40, 80, 80, 120, 120, 160, 160]
    for actual, exp in zip(lengths, expected):
        assert abs(actual - exp) < 0.5


def test_expanding_square_returns_requested_leg_count():
    waypoints = expanding_square(HOME_LAT, HOME_LON, leg_unit_m=10.0, num_legs=20)
    assert len(waypoints) == 20


def test_lawnmower_covers_requested_width():
    waypoints = lawnmower(HOME_LAT, HOME_LON, width_m=200, height_m=100, lane_spacing_m=25)
    lons = [lon for _, lon in waypoints]
    lats = [lat for lat, _ in waypoints]
    west_lon, east_lon = min(lons), max(lons)
    south_lat, north_lat = min(lats), max(lats)
    width = distance_m(south_lat, west_lon, south_lat, east_lon)
    height = distance_m(south_lat, west_lon, north_lat, west_lon)
    assert abs(width - 200) < 1.0
    assert abs(height - 100) < 26  # within one lane spacing of requested height
