"""
End-to-end test of the full mission state machine (search -> detect ->
orbit -> RTL) against the in-memory simulator backend. No SITL required.
This is the test that proves the mission logic itself is correct, since the
MAVLink backend implements the exact same Vehicle interface.
"""

from missionlib.backends.sim_backend import SimVehicle
from missionlib.geo import distance_m
from missionlib.mission import MissionConfig, run_mission
from missionlib.patterns import expanding_square

HOME_LAT, HOME_LON = -35.363261, 149.165230
# On leg 6 of expanding_square(HOME, leg_unit_m=40, num_legs=12) - see test below.
TARGET_LAT, TARGET_LON = -35.362542, 149.166111


def make_vehicle():
    return SimVehicle(HOME_LAT, HOME_LON, realtime=False)


def test_target_on_path_is_detected_and_orbited():
    waypoints = expanding_square(HOME_LAT, HOME_LON, leg_unit_m=40.0, num_legs=12)
    cfg = MissionConfig(
        target_lat=TARGET_LAT, target_lon=TARGET_LON, target_radius_m=25.0,
        orbit_duration_s=5.0, orbit_step_s=1.0,
    )
    result = run_mission(make_vehicle(), waypoints, cfg)

    assert result.detected is True
    det_lat, det_lon = result.detection_fix
    assert distance_m(det_lat, det_lon, TARGET_LAT, TARGET_LON) <= cfg.target_radius_m

    orbit_points = [p for p in result.flight_log if p[3] == "orbit"]
    assert len(orbit_points) > 0
    for lat, lon, alt, _ in orbit_points:
        d = distance_m(lat, lon, TARGET_LAT, TARGET_LON)
        assert abs(d - cfg.orbit_radius_m) < 1.0


def test_target_far_from_path_is_never_detected():
    waypoints = expanding_square(HOME_LAT, HOME_LON, leg_unit_m=40.0, num_legs=12)
    cfg = MissionConfig(target_lat=-35.0, target_lon=149.0, target_radius_m=25.0)
    result = run_mission(make_vehicle(), waypoints, cfg)

    assert result.detected is False
    assert result.detection_fix is None
    assert all(p[3] == "search" for p in result.flight_log)


def test_vehicle_lands_back_near_home():
    waypoints = expanding_square(HOME_LAT, HOME_LON, leg_unit_m=40.0, num_legs=12)
    cfg = MissionConfig(
        target_lat=TARGET_LAT, target_lon=TARGET_LON,
        orbit_duration_s=2.0, orbit_step_s=1.0,
    )
    vehicle = make_vehicle()
    run_mission(vehicle, waypoints, cfg)

    assert distance_m(vehicle.lat, vehicle.lon, HOME_LAT, HOME_LON) < 1.0
    assert vehicle.alt == 0.0
    assert vehicle.armed is False
