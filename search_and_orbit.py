#!/usr/bin/env python3
"""
ArduCopter SITL mission: expanding-square search -> target detection -> orbit -> RTL.

Flow:
  1. Connect over UDP to the SITL MAVLink stream.
  2. Wait for a HEARTBEAT (proves a vehicle is on the other end).
  3. Arm + switch to GUIDED + takeoff to TAKEOFF_ALT via MAV_CMD_NAV_TAKEOFF.
  4. Walk an expanding-square search pattern, issuing one SET_POSITION_TARGET_GLOBAL_INT
     per leg and polling GLOBAL_POSITION_INT to know when each waypoint is reached.
  5. On every position update, check distance to the hardcoded TARGET. If inside
     TARGET_RADIUS_M, abandon the search pattern and orbit the target at
     ORBIT_RADIUS_M by streaming a slowly-rotating SET_POSITION_TARGET_GLOBAL_INT
     once per second for ORBIT_DURATION_S.
  6. Command RTL and wait for the vehicle to disarm (i.e. landed) before exiting.

Run this against `sim_vehicle.py` SITL, which by default streams MAVLink on
udp:127.0.0.1:14550 (the address MAVProxy forwards telemetry to for ground
control stations / scripts).
"""

import math
import time
from datetime import datetime, timezone

from pymavlink import mavutil

# --------------------------------------------------------------------------
# Mission parameters - edit these for your scenario
# --------------------------------------------------------------------------
CONNECTION_STRING = "udp:127.0.0.1:14550"

TAKEOFF_ALT_M = 30.0          # altitude to climb to before starting the search
WAYPOINT_ACCEPT_RADIUS_M = 3.0  # how close counts as "arrived" at a leg

# Expanding square: center + initial leg length + growth per leg + number of legs.
# Classic SAR pattern: each leg is 90 degrees from the last and grows by one
# "leg unit" every two legs (1,1,2,2,3,3,...).
SEARCH_CENTER_LAT = -35.363261   # default SITL home (CMAC) latitude
SEARCH_CENTER_LON = 149.165230   # default SITL home (CMAC) longitude
LEG_UNIT_M = 40.0                # base leg length, grows each pass
NUM_LEGS = 12                     # how many legs of the square to fly

# Hardcoded "target" geofence: a small circle the vehicle treats as a find.
TARGET_LAT = -35.361500
TARGET_LON = 149.167000
TARGET_RADIUS_M = 25.0           # entering this circle = "target detected"

ORBIT_RADIUS_M = 15.0
ORBIT_DURATION_S = 30.0          # how long to circle once the target is found
ORBIT_ANGULAR_RATE_DEG_S = 10.0  # orbit speed (360/this = seconds per full circle)

# --------------------------------------------------------------------------
# Geo helpers (flat-earth approximation, fine for short SAR-scale distances)
# --------------------------------------------------------------------------
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


def build_expanding_square(center_lat, center_lon, leg_unit_m, num_legs):
    """
    Generate waypoints for an expanding square search starting at the center.
    Directions cycle N, E, S, W; leg length grows every 2 legs (1,1,2,2,3,3...).
    """
    waypoints = []
    lat, lon = center_lat, center_lon
    headings = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # (north_mult, east_mult)
    leg_count = 1
    for i in range(num_legs):
        north_mult, east_mult = headings[i % 4]
        length = leg_unit_m * leg_count
        lat, lon = offset_latlon(lat, lon, north_mult * length, east_mult * length)
        waypoints.append((lat, lon))
        if i % 2 == 1:
            leg_count += 1
    return waypoints


def log_event(msg):
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
    print(f"[{ts}] {msg}")


# --------------------------------------------------------------------------
# MAVLink helpers
# --------------------------------------------------------------------------
def wait_heartbeat(master):
    log_event("Waiting for heartbeat...")
    master.wait_heartbeat()
    log_event(f"Heartbeat received (sys {master.target_system} comp {master.target_component})")


def set_mode(master, mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    # Confirm the mode actually changed before moving on.
    while True:
        ack = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if ack is not None and mavutil.mode_string_v10(ack) == mode_name:
            log_event(f"Mode set to {mode_name}")
            return


def wait_for_gps_fix(master, timeout=30):
    """
    SITL's EKF/GPS take a few seconds to settle after boot; arming before
    that fails pre-arm checks and previously caused the script to hang
    forever in motors_armed_wait(). Block here until we see a usable fix
    (fix_type >= 3 = 3D fix) or give up after `timeout` seconds.
    """
    log_event("Waiting for GPS fix...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if msg is not None and msg.fix_type >= 3:
            log_event("GPS fix acquired.")
            return
    raise TimeoutError("No GPS fix after waiting; aborting before arm.")


def arm(master, retries=5):
    wait_for_gps_fix(master)
    for attempt in range(1, retries + 1):
        log_event(f"Arming (attempt {attempt}/{retries})...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,  # param1=1 -> arm
        )
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            master.motors_armed_wait()
            log_event("Armed.")
            return
        log_event(f"Arm rejected (pre-arm check failed?) - retrying. ack={ack}")
        time.sleep(2)
    raise RuntimeError("Failed to arm after retries - check SITL pre-arm checks.")


def takeoff(master, altitude_m):
    log_event(f"Taking off to {altitude_m} m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude_m,  # param7 = takeoff altitude (relative)
    )
    # Poll altitude until we're close to the target before starting the pattern.
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            continue
        alt_m = msg.relative_alt / 1000.0
        if alt_m >= altitude_m * 0.95:
            log_event(f"Reached takeoff altitude ({alt_m:.1f} m).")
            return


def goto_position_target(master, lat, lon, alt_m):
    """
    Send a single guided-mode position target. type_mask bits below disable
    velocity/acceleration/yaw fields so only lat/lon/alt are used (position-only
    guided move), per MAVLink SET_POSITION_TARGET_GLOBAL_INT semantics.
    """
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_global_int_send(
        0,  # time_boot_ms (unused, 0 = "now")
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        alt_m,
        0, 0, 0,  # vx, vy, vz (ignored)
        0, 0, 0,  # afx, afy, afz (ignored)
        0, 0,     # yaw, yaw_rate (ignored)
    )


def get_position(master, timeout=2.0):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0


def target_in_range(lat, lon):
    return distance_m(lat, lon, TARGET_LAT, TARGET_LON) <= TARGET_RADIUS_M


def fly_search_pattern(master, waypoints, altitude_m):
    """
    Step through each leg of the expanding square. After issuing the position
    target we keep polling GLOBAL_POSITION_INT both to detect arrival at the
    waypoint and, on every sample, to check the target geofence. Returns the
    detection fix (lat, lon) if the target is found mid-pattern, else None.
    """
    for idx, (wp_lat, wp_lon) in enumerate(waypoints):
        log_event(f"Leg {idx + 1}/{len(waypoints)}: heading to ({wp_lat:.6f}, {wp_lon:.6f})")
        goto_position_target(master, wp_lat, wp_lon, altitude_m)

        while True:
            pos = get_position(master)
            if pos is None:
                continue
            cur_lat, cur_lon, cur_alt = pos

            if target_in_range(cur_lat, cur_lon):
                return cur_lat, cur_lon

            if distance_m(cur_lat, cur_lon, wp_lat, wp_lon) <= WAYPOINT_ACCEPT_RADIUS_M:
                break  # arrived at this leg's waypoint, advance to the next leg


def orbit_target(master, center_lat, center_lon, altitude_m):
    """
    Circle the target at ORBIT_RADIUS_M by issuing a new position target once
    per second, walking the bearing angle forward by ORBIT_ANGULAR_RATE_DEG_S.
    This is a simple guided-mode loiter substitute (vs. MAV_CMD_NAV_LOITER_TURNS,
    which requires an AUTO-mode mission item rather than ad-hoc GUIDED commands).
    """
    log_event(
        f"Orbiting target at ({center_lat:.6f}, {center_lon:.6f}), "
        f"radius {ORBIT_RADIUS_M} m, for {ORBIT_DURATION_S:.0f} s."
    )
    angle_deg = 0.0
    steps = int(ORBIT_DURATION_S)
    for _ in range(steps):
        angle_rad = math.radians(angle_deg)
        north_m = ORBIT_RADIUS_M * math.cos(angle_rad)
        east_m = ORBIT_RADIUS_M * math.sin(angle_rad)
        orbit_lat, orbit_lon = offset_latlon(center_lat, center_lon, north_m, east_m)
        goto_position_target(master, orbit_lat, orbit_lon, altitude_m)
        angle_deg = (angle_deg + ORBIT_ANGULAR_RATE_DEG_S) % 360.0
        time.sleep(1.0)


def return_and_land(master):
    log_event("Commanding RTL (return to launch + auto land).")
    set_mode(master, "RTL")
    log_event("Waiting for disarm (landed)...")
    while True:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        if msg is None:
            continue
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if not armed:
            log_event("Disarmed - vehicle has landed.")
            return


# --------------------------------------------------------------------------
# Main mission
# --------------------------------------------------------------------------
def main():
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    wait_heartbeat(master)

    set_mode(master, "GUIDED")
    arm(master)
    takeoff(master, TAKEOFF_ALT_M)

    waypoints = build_expanding_square(
        SEARCH_CENTER_LAT, SEARCH_CENTER_LON, LEG_UNIT_M, NUM_LEGS
    )
    log_event(f"Generated {len(waypoints)}-leg expanding square search pattern.")

    detection = fly_search_pattern(master, waypoints, TAKEOFF_ALT_M)

    if detection is not None:
        det_lat, det_lon = detection
        log_event(
            "TARGET DETECTED - breaking search pattern. "
            f"GPS fix: lat={det_lat:.7f}, lon={det_lon:.7f}, alt={TAKEOFF_ALT_M:.1f} m"
        )
        orbit_target(master, TARGET_LAT, TARGET_LON, TAKEOFF_ALT_M)
    else:
        log_event("Search pattern complete - target not detected.")

    return_and_land(master)
    log_event("Mission complete.")


if __name__ == "__main__":
    main()
