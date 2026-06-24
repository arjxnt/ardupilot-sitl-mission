"""Serialize a MissionResult to JSON for the web dashboard in dashboard/."""

import json


def export_mission_json(result, cfg, home_lat, home_lon, target_lat, target_lon, out_path):
    data = {
        "home": {"lat": home_lat, "lon": home_lon},
        "target": {"lat": target_lat, "lon": target_lon, "radius_m": cfg.target_radius_m},
        "orbit_radius_m": cfg.orbit_radius_m,
        "takeoff_alt_m": cfg.takeoff_alt_m,
        "detected": result.detected,
        "detection_fix": result.detection_fix,
        "detection_time": result.detection_time,
        "flight_log": [
            {"lat": lat, "lon": lon, "alt": alt, "phase": phase}
            for lat, lon, alt, phase in result.flight_log
        ],
        "events": [{"time": t, "message": m} for t, m in result.events],
    }
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
