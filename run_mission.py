#!/usr/bin/env python3
"""
CLI entrypoint. Same mission logic, two backends:

  python run_mission.py --backend sim                 # no SITL needed, runs instantly
  python run_mission.py --backend mavlink              # flies real SITL over MAVLink

The mission state machine (missionlib/mission.py) is identical either way;
only the Vehicle implementation underneath it changes. See missionlib/vehicle.py.
"""

import argparse

from missionlib.mission import MissionConfig, run_mission
from missionlib.patterns import expanding_square

SEARCH_CENTER_LAT = -35.363261   # default SITL home (CMAC)
SEARCH_CENTER_LON = 149.165230
LEG_UNIT_M = 40.0
NUM_LEGS = 12

# Sits directly on leg 6 of the default expanding-square pattern above, so a
# default run actually triggers a detection instead of completing the whole
# search with nothing found.
TARGET_LAT = -35.362542
TARGET_LON = 149.166111


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["sim", "mavlink"], default="sim")
    parser.add_argument("--connection", default="udp:127.0.0.1:14550",
                         help="MAVLink connection string (--backend mavlink only)")
    parser.add_argument("--plot", default=None,
                         help="Path to save a PNG of the flight path (requires matplotlib)")
    parser.add_argument("--export-json", default=None,
                         help="Path to save mission data (flight log + events) as JSON "
                              "for the web dashboard in dashboard/")
    args = parser.parse_args()

    cfg = MissionConfig(
        target_lat=TARGET_LAT,
        target_lon=TARGET_LON,
    )
    waypoints = expanding_square(SEARCH_CENTER_LAT, SEARCH_CENTER_LON, LEG_UNIT_M, NUM_LEGS)

    if args.backend == "sim":
        from missionlib.backends.sim_backend import SimVehicle
        vehicle = SimVehicle(SEARCH_CENTER_LAT, SEARCH_CENTER_LON, realtime=False)
    else:
        from missionlib.backends.mavlink_backend import MavlinkVehicle
        vehicle = MavlinkVehicle(connection_string=args.connection)

    result = run_mission(vehicle, waypoints, cfg)

    print()
    print(f"Detected: {result.detected}")
    if result.detected:
        print(f"Detection fix: {result.detection_fix}")
        print(f"Detection time: {result.detection_time}")

    if args.plot:
        from missionlib.plotting import plot_flight_log
        plot_flight_log(result.flight_log, TARGET_LAT, TARGET_LON,
                         cfg.target_radius_m, args.plot)
        print(f"Flight path plotted to {args.plot}")

    if args.export_json:
        from missionlib.export import export_mission_json
        export_mission_json(
            result, cfg,
            home_lat=SEARCH_CENTER_LAT, home_lon=SEARCH_CENTER_LON,
            target_lat=TARGET_LAT, target_lon=TARGET_LON,
            out_path=args.export_json,
        )
        print(f"Mission data exported to {args.export_json}")


if __name__ == "__main__":
    main()
