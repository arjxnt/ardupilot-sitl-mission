"""
Vehicle interface: the mission logic in mission.py only ever talks to this.

Two backends implement it: backends/mavlink_backend.py (talks to a real
ArduPilot SITL/flight controller over MAVLink) and backends/sim_backend.py
(a lightweight kinematic simulator with no external dependencies). Because
mission.py is written against this interface and not against pymavlink
directly, the exact same search/detect/orbit/RTL logic can be exercised by
an automated test suite (against the sim backend) without ever needing a
running SITL instance, and that same logic is what flies the real simulator
when you swap in the MAVLink backend.
"""

from abc import ABC, abstractmethod


class Vehicle(ABC):
    @abstractmethod
    def connect(self):
        """Establish the connection / wait for the vehicle to be ready."""

    @abstractmethod
    def arm_and_takeoff(self, altitude_m):
        """Arm, switch to a script-controlled mode, and climb to altitude_m."""

    @abstractmethod
    def goto(self, lat, lon, alt_m):
        """Command a move to (lat, lon, alt_m). Non-blocking."""

    @abstractmethod
    def position(self):
        """Return current (lat, lon, alt_m)."""

    @abstractmethod
    def return_and_land(self):
        """Command RTL and block until the vehicle has landed/disarmed."""
