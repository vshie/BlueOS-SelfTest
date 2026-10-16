"""
Simulated autopilot for UI development and tests.

Enable with ``SELFTEST_SIM=bluerov2`` (or ``heavy`` / ``blueboat``). It
implements the subset of :class:`mavlink.MavlinkClient` the extension
uses, with parameter sets taken from the BlueOS Parameter Repository.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

BASE_SUB = {
    "FRAME_CONFIG": 1, "SYSID_THISMAV": 1,
    "BATT_MONITOR": 4, "BATT_VOLT_MULT": 11.0, "BATT_AMP_PERVLT": 37.8788, "BATT_AMP_OFFSET": 0.33,
    "MNT1_TYPE": 1, "MNT1_PITCH_MIN": -45, "MNT1_PITCH_MAX": 45, "MNT1_DEFLT_MODE": 3,
    **{f"SERVO{n}_FUNCTION": 0 for n in range(1, 17)},
    **{f"SERVO{n}_MIN": 1100 for n in range(1, 17)},
    **{f"SERVO{n}_MAX": 1900 for n in range(1, 17)},
    **{f"SERVO{n}_TRIM": 1500 for n in range(1, 17)},
    **{f"MOT_{n}_DIRECTION": 1 for n in range(1, 9)},
}
BASE_SUB.update({f"SERVO{n}_FUNCTION": 32 + n for n in range(1, 7)})
BASE_SUB.update({"SERVO9_FUNCTION": 184, "SERVO13_FUNCTION": 181, "SERVO14_FUNCTION": 182,
                 "SERVO16_FUNCTION": 7, "MOT_2_DIRECTION": -1, "MOT_5_DIRECTION": -1})

PROFILES = {
    "bluerov2": ("MAV_TYPE_SUBMARINE", BASE_SUB),
    "heavy": ("MAV_TYPE_SUBMARINE", {**BASE_SUB, "FRAME_CONFIG": 2,
                                     "SERVO7_FUNCTION": 39, "SERVO8_FUNCTION": 40}),
    "blueboat": ("MAV_TYPE_SURFACE_BOAT", {
        "FRAME_CLASS": 2, "SYSID_THISMAV": 1,
        "BATT_MONITOR": 4, "BATT_VOLT_MULT": 11.3, "BATT_AMP_PERVLT": 100.0, "BATT_AMP_OFFSET": 1.638,
        **{f"SERVO{n}_FUNCTION": 0 for n in range(1, 17)},
        **{f"SERVO{n}_MIN": 1100 for n in range(1, 17)},
        **{f"SERVO{n}_MAX": 1900 for n in range(1, 17)},
        **{f"SERVO{n}_TRIM": 1500 for n in range(1, 17)},
        "SERVO1_FUNCTION": 74, "SERVO3_FUNCTION": 73,
    }),
}


class SimClient:
    def __init__(self, profile: str = "bluerov2") -> None:
        mavtype, params = PROFILES.get(profile, PROFILES["bluerov2"])
        self.mavtype = mavtype
        self.params = dict(params)
        self.target_system = 1
        self.target_component = 1
        self.base_url = "sim://"
        self.connected = True
        self.armed = False
        self.mode = 0 if mavtype == "MAV_TYPE_SUBMARINE" else 4
        self.servo = [0] * 16
        self.offline_until = 0.0
        self._hb_counter = 0
        self.command_log: list = []
        self._lock = threading.Lock()
        threading.Thread(target=self._tick, daemon=True).start()

    def _tick(self):
        while True:
            if time.monotonic() > self.offline_until:
                self._hb_counter += 1
            time.sleep(1.0)

    def connect(self) -> bool:
        return True

    def _online(self) -> bool:
        return time.monotonic() > self.offline_until

    def message(self, name: str):
        stamp = str(self._hb_counter)
        if name == "HEARTBEAT":
            return ({"mavtype": {"type": self.mavtype},
                     "autopilot": {"type": "MAV_AUTOPILOT_ARDUPILOTMEGA"},
                     "base_mode": {"bits": 128 if self.armed else 0},
                     "custom_mode": self.mode}, stamp)
        if name == "AUTOPILOT_VERSION":
            ver = (4 << 24) | (5 << 16) | (7 << 8) if self.mavtype == "MAV_TYPE_SUBMARINE" \
                else (4 << 24) | (5 << 16) | (7 << 8)
            return {"flight_sw_version": ver}, stamp
        if name == "SYS_STATUS":
            mult = self.params.get("BATT_VOLT_MULT", 11) / 11.0
            apv = self.params.get("BATT_AMP_PERVLT", 37.8788) / 37.8788
            return {"voltage_battery": int(15800 * mult), "current_battery": int(120 * apv)}, stamp
        if name == "SERVO_OUTPUT_RAW":
            return {f"servo{i + 1}_raw": v for i, v in enumerate(self.servo)}, stamp
        return None, None

    def heartbeat(self):
        return self.message("HEARTBEAT")[0] if self._online() else None

    def is_armed(self) -> bool:
        return self.armed

    def custom_mode(self):
        return self.mode

    def command_long(self, command: str, params: Optional[dict] = None) -> bool:
        self.command_log.append((command, dict(params or {})))
        p = params or {}
        if command == "MAV_CMD_DO_SET_SERVO":
            ch = int(p[1])
            if self.params.get(f"SERVO{ch}_FUNCTION", 0) == 0:
                self.servo[ch - 1] = int(p[2])
        elif command == "MAV_CMD_DO_MOTOR_TEST":
            if self.mavtype != "MAV_TYPE_SUBMARINE":
                self.armed = True
        return True

    def command_ack(self, command, timeout_s=3.0, params=None):
        self.command_long(command, params)
        p = params or {}
        if command == "MAV_CMD_COMPONENT_ARM_DISARM":
            self.armed = bool(p.get(1))
        elif command == "MAV_CMD_DO_SET_MODE":
            self.mode = int(p.get(2, 0))
        elif command == "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN":
            self.armed = False
            self.offline_until = time.monotonic() + 4
        return "MAV_RESULT_ACCEPTED"

    def set_servo(self, ch, pwm):
        return self.command_long("MAV_CMD_DO_SET_SERVO", {1: ch, 2: pwm})

    def set_mode(self, mode):
        return self.command_ack("MAV_CMD_DO_SET_MODE", params={1: 1, 2: mode})

    def arm(self, force=True):
        return self.command_ack("MAV_CMD_COMPONENT_ARM_DISARM", params={1: 1})

    def disarm(self, force=True):
        return self.command_ack("MAV_CMD_COMPONENT_ARM_DISARM", params={1: 0})

    def motor_test(self, motor, pwm, timeout_s=1.0):
        return self.command_long("MAV_CMD_DO_MOTOR_TEST", {1: motor, 2: 1, 3: pwm})

    def reboot_autopilot(self):
        return self.command_ack("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", params={1: 1})

    def mount_pitch(self, pitch):
        self.command_long("MAV_CMD_DO_MOUNT_CONTROL", {1: pitch, 7: 2})

    def mount_mode(self, mode):
        self.command_long("MAV_CMD_DO_MOUNT_CONFIGURE", {1: mode})

    def request_message(self, msg_id):
        pass

    def param_get(self, name, timeout_s=2.5):
        if not self._online():
            time.sleep(min(timeout_s, 0.2))
            return None
        time.sleep(0.01)
        v = self.params.get(name)
        return None if v is None else float(v)

    def param_set(self, name, value, timeout_s=5.0):
        if not self._online():
            from mavlink import MavlinkError
            raise MavlinkError("offline")
        self.params[name] = float(value)
        return float(value)

    def blueos_get(self, path, timeout_s=None):
        if "/board" in path:
            return {"name": "Navigator"}
        if "available_firmwares" in path:
            return [{"name": f"STABLE-{v}"} for v in ("4.7.2", "4.7.1", "4.5.7")] + [{"name": "BETA"}]
        if "vehicle_name" in path:
            return "Simulated vehicle"
        if "version" in path:
            return {"tag": "1.4.6"}
        return None
