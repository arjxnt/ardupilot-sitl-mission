"""
A small kinematic flight simulator that implements the Vehicle interface
with no external dependencies (no SITL, no network, no pymavlink).

It models the drone as a point that moves toward its current goto() target
in a straight line at a fixed ground speed, climbing/descending at a fixed
vertical speed, and reports its position via position(). This is enough to
exercise the full mission state machine (search -> detect -> orbit -> RTL)
end-to-end, which is what the test suite in tests/ does.

It is intentionally simple: it is a test/demo harness, not a flight dynamics
model. Real flight characteristics (wind, inertia, EKF noise) only show up
once you run against actual SITL via the MAVLink backend.
"""

import math
import time

from ..geo import distance_m, offset_latlon
from ..vehicle import Vehicle


class SimVehicle(Vehicle):
    def __init__(self, home_lat, home_lon, ground_speed_mps=8.0, climb_rate_mps=3.0,
                 step_s=0.2, realtime=False):
        self.lat = home_lat
        self.lon = home_lon
        self.alt = 0.0
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.ground_speed_mps = ground_speed_mps
        self.climb_rate_mps = climb_rate_mps
        self.step_s = step_s
        self.realtime = realtime  # if False, time.sleep is skipped (fast tests)
        self.armed = False
        self._goal = None  # (lat, lon, alt)

    def _sleep(self):
        if self.realtime:
            time.sleep(self.step_s)

    def connect(self):
        pass  # nothing to do; the sim is always "connected"

    def arm_and_takeoff(self, altitude_m):
        self.armed = True
        while self.alt < altitude_m:
            self.alt = min(altitude_m, self.alt + self.climb_rate_mps * self.step_s)
            self._sleep()

    def goto(self, lat, lon, alt_m):
        self._goal = (lat, lon, alt_m)

    def _step_toward_goal(self):
        if self._goal is None:
            return
        glat, glon, galt = self._goal
        remaining = distance_m(self.lat, self.lon, glat, glon)
        step_dist = self.ground_speed_mps * self.step_s
        if remaining <= step_dist:
            self.lat, self.lon = glat, glon
        else:
            bearing = math.atan2(
                math.radians(glon - self.lon) * math.cos(math.radians(self.lat)),
                math.radians(glat - self.lat),
            )
            north = step_dist * math.cos(bearing)
            east = step_dist * math.sin(bearing)
            self.lat, self.lon = offset_latlon(self.lat, self.lon, north, east)

        if abs(galt - self.alt) <= self.climb_rate_mps * self.step_s:
            self.alt = galt
        else:
            self.alt += self.climb_rate_mps * self.step_s * (1 if galt > self.alt else -1)

    def position(self):
        self._step_toward_goal()
        self._sleep()
        return self.lat, self.lon, self.alt

    def return_and_land(self):
        self.goto(self.home_lat, self.home_lon, self.alt)
        while distance_m(self.lat, self.lon, self.home_lat, self.home_lon) > 1.0:
            self.position()
        while self.alt > 0:
            self.alt = max(0.0, self.alt - self.climb_rate_mps * self.step_s)
            self._sleep()
        self.armed = False
