"""
Real Vehicle implementation: talks to ArduPilot SITL (or real hardware) over
MAVLink via pymavlink. This is what mission.run_mission() drives when you
actually have sim_vehicle.py running. See vehicle.py for the interface this
fulfills, and missionlib/mission.py for the mission logic that calls it.
"""

import time

from pymavlink import mavutil

from ..vehicle import Vehicle


class MavlinkVehicle(Vehicle):
    def __init__(self, connection_string="udp:127.0.0.1:14550", gps_timeout_s=30,
                 arm_retries=5):
        self.connection_string = connection_string
        self.gps_timeout_s = gps_timeout_s
        self.arm_retries = arm_retries
        self.master = None

    def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()  # proves a vehicle is on the other end
        self._set_mode("GUIDED")

    def _set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        while True:
            ack = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if ack is not None and mavutil.mode_string_v10(ack) == mode_name:
                return

    def _wait_for_gps_fix(self):
        # SITL's EKF/GPS take a few seconds to settle after boot; arming
        # before that fails ArduPilot's pre-arm checks. fix_type >= 3 = 3D fix.
        deadline = time.time() + self.gps_timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
            if msg is not None and msg.fix_type >= 3:
                return
        raise TimeoutError("No GPS fix after waiting; aborting before arm.")

    def _arm(self):
        self._wait_for_gps_fix()
        for attempt in range(1, self.arm_retries + 1):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0,  # param1=1 -> arm
            )
            ack = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                    and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.master.motors_armed_wait()
                return
            time.sleep(2)
        raise RuntimeError("Failed to arm after retries - check SITL pre-arm checks.")

    def arm_and_takeoff(self, altitude_m):
        self._arm()
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude_m,  # param7 = takeoff altitude (relative)
        )
        while True:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                continue
            if msg.relative_alt / 1000.0 >= altitude_m * 0.95:
                return

    def goto(self, lat, lon, alt_m):
        # type_mask below disables velocity/accel/yaw fields so only
        # lat/lon/alt are used (position-only guided move).
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
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            alt_m,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )

    def position(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if msg is None:
            return self.position()  # retry; mission loop expects a value
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0

    def return_and_land(self):
        self._set_mode("RTL")
        while True:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            if msg is None:
                continue
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                return
