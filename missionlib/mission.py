"""
Mission state machine: SEARCH -> (target found) -> ORBIT -> RTL.

This module is backend-agnostic: it only calls methods on a Vehicle (see
vehicle.py), so it runs identically whether `vehicle` is the real MAVLink
backend flying SITL or the in-memory simulator backend used by the tests.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime, timezone

from .geo import bearing_point, distance_m


@dataclass
class MissionConfig:
    takeoff_alt_m: float = 30.0
    waypoint_accept_radius_m: float = 3.0
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_radius_m: float = 25.0
    orbit_radius_m: float = 15.0
    orbit_duration_s: float = 30.0
    orbit_angular_rate_deg_s: float = 10.0
    orbit_step_s: float = 1.0


@dataclass
class MissionResult:
    detected: bool
    detection_fix: tuple | None
    detection_time: str | None
    flight_log: list = field(default_factory=list)  # list of (lat, lon, alt, label)
    events: list = field(default_factory=list)  # list of (timestamp, message)


def log_event(msg, events=None):
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
    print(f"[{ts}] {msg}")
    if events is not None:
        events.append((ts, msg))
    return ts


def target_in_range(lat, lon, cfg: MissionConfig):
    return distance_m(lat, lon, cfg.target_lat, cfg.target_lon) <= cfg.target_radius_m


def fly_search_pattern(vehicle, waypoints, cfg: MissionConfig, flight_log, events):
    """
    Walk each leg of the given waypoint list. On every position sample we
    both check arrival at the current leg's waypoint and check the target
    geofence, so detection can interrupt a leg mid-flight rather than only
    at waypoint corners. Returns the detection (lat, lon) or None.
    """
    for idx, (wp_lat, wp_lon) in enumerate(waypoints):
        log_event(f"Leg {idx + 1}/{len(waypoints)}: heading to ({wp_lat:.6f}, {wp_lon:.6f})", events)
        vehicle.goto(wp_lat, wp_lon, cfg.takeoff_alt_m)

        while True:
            lat, lon, alt = vehicle.position()
            flight_log.append((lat, lon, alt, "search"))

            if target_in_range(lat, lon, cfg):
                return lat, lon

            if distance_m(lat, lon, wp_lat, wp_lon) <= cfg.waypoint_accept_radius_m:
                break


def orbit_target(vehicle, cfg: MissionConfig, flight_log, events):
    log_event(
        f"Orbiting target at ({cfg.target_lat:.6f}, {cfg.target_lon:.6f}), "
        f"radius {cfg.orbit_radius_m} m, for {cfg.orbit_duration_s:.0f} s.",
        events,
    )
    angle_deg = 0.0
    steps = int(cfg.orbit_duration_s / cfg.orbit_step_s)
    for _ in range(steps):
        orbit_lat, orbit_lon = bearing_point(
            cfg.target_lat, cfg.target_lon, cfg.orbit_radius_m, angle_deg
        )
        vehicle.goto(orbit_lat, orbit_lon, cfg.takeoff_alt_m)
        flight_log.append((orbit_lat, orbit_lon, cfg.takeoff_alt_m, "orbit"))
        angle_deg = (angle_deg + cfg.orbit_angular_rate_deg_s) % 360.0
        # Skip the artificial pacing delay for backends that don't need
        # real-time pacing (e.g. the sim backend running in fast/test mode).
        if getattr(vehicle, "realtime", True):
            time.sleep(cfg.orbit_step_s)


def run_mission(vehicle, waypoints, cfg: MissionConfig) -> MissionResult:
    flight_log = []
    events = []

    vehicle.connect()
    vehicle.arm_and_takeoff(cfg.takeoff_alt_m)

    detection = fly_search_pattern(vehicle, waypoints, cfg, flight_log, events)

    detection_time = None
    if detection is not None:
        det_lat, det_lon = detection
        detection_time = log_event(
            "TARGET DETECTED - breaking search pattern. "
            f"GPS fix: lat={det_lat:.7f}, lon={det_lon:.7f}, alt={cfg.takeoff_alt_m:.1f} m",
            events,
        )
        orbit_target(vehicle, cfg, flight_log, events)
    else:
        log_event("Search pattern complete - target not detected.", events)

    vehicle.return_and_land()
    log_event("Mission complete.", events)

    return MissionResult(
        detected=detection is not None,
        detection_fix=detection,
        detection_time=detection_time,
        flight_log=flight_log,
        events=events,
    )
