"""
Vehicle and hardware detection.

Works out which Blue Robotics vehicle the autopilot is (BlueROV2,
BlueROV2 Heavy, BlueBoat) and which outputs exist, using the same
SERVOn_FUNCTION mapping that the BlueOS Vehicle Setup > PWM Outputs page
displays (ArduPilot libraries/SRV_Channel/SRV_Channel.h).
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from mavlink import MavlinkClient, enum_value

logger = logging.getLogger(__name__)

NUM_SERVO_OUTPUTS = 16

# SERVOn_FUNCTION values (SRV_Channel.h)
FN_DISABLED = 0
FN_MOUNT_TILT = 7
FN_GROUND_STEERING = 26
FN_GRIPPER = 28
FN_MOTOR1, FN_MOTOR8 = 33, 40
FN_RCIN9, FN_RCIN10 = 59, 60
FN_THROTTLE = 70
FN_THROTTLE_LEFT, FN_THROTTLE_RIGHT = 73, 74
FN_CAMERA_FOCUS = 92
FN_CAMERA_ZOOM = 180
FN_LIGHTS1, FN_LIGHTS2 = 181, 182
FN_ACTUATOR1, FN_ACTUATOR6 = 184, 189

# ArduSub <= 4.5.5 drove lights through RCIN9/RCIN10 passthrough; BlueOS
# labels those outputs "Lights 1/2" up to this version
# (store/autopilot_manager.ts LEGACY_SUB_LIGHTS_MAX_VERSION).
LEGACY_SUB_LIGHTS_MAX_VERSION = (4, 5, 5)

# ArduRover MOTOR_TEST_ORDER for each output function (Rover/motor_test.cpp,
# same mapping as BlueOS PwmSetup.vue).
ROVER_MOTOR_TEST_ID = {FN_THROTTLE: 1, FN_GROUND_STEERING: 2,
                       FN_THROTTLE_LEFT: 3, FN_THROTTLE_RIGHT: 4}

# Default battery-monitor parameters shipped by the BlueOS Parameter
# Repository (docs.bluerobotics.com/Blueos-Parameter-Repository).
VEHICLE_DEFAULTS = {
    "bluerov2": {
        "BATT_VOLT_MULT": 11.0, "BATT_AMP_PERVLT": 37.8788, "BATT_AMP_OFFSET": 0.33,
        "voltage_range": (12.0, 16.8),   # 4S Li-ion pack, empty..full
    },
    "blueboat": {
        "BATT_VOLT_MULT": 11.3, "BATT_AMP_PERVLT": 100.0, "BATT_AMP_OFFSET": 1.638,
        "voltage_range": (12.0, 16.8),
    },
}
CALIBRATION_TOLERANCE = 0.10   # warn when a new value is >10% from default

TEST_ORDER = ("motor", "lights", "actuator", "focus", "zoom", "tilt")


def parse_flight_sw_version(value: int) -> tuple:
    return ((value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF)


def classify_output(fn: int, firmware: str, version: Optional[tuple]) -> Optional[dict]:
    """Map one SERVOn_FUNCTION to a test category, or None if untested."""
    if firmware == "ArduSub":
        if FN_MOTOR1 <= fn <= FN_MOTOR8:
            return {"category": "motor", "name": f"Motor {fn - FN_MOTOR1 + 1}",
                    "motor_number": fn - FN_MOTOR1 + 1}
        if fn in (FN_LIGHTS1, FN_LIGHTS2):
            return {"category": "lights", "name": f"Lights {fn - FN_LIGHTS1 + 1}"}
        if fn in (FN_RCIN9, FN_RCIN10) and version and version <= LEGACY_SUB_LIGHTS_MAX_VERSION:
            return {"category": "lights", "name": f"Lights {fn - FN_RCIN9 + 1}"}
    if firmware == "ArduRover" and fn in ROVER_MOTOR_TEST_ID:
        names = {FN_THROTTLE_LEFT: "Left thruster", FN_THROTTLE_RIGHT: "Right thruster",
                 FN_THROTTLE: "Throttle", FN_GROUND_STEERING: "Steering"}
        return {"category": "motor", "name": names[fn], "rover_motor_id": ROVER_MOTOR_TEST_ID[fn]}
    if fn == FN_GRIPPER:
        return {"category": "actuator", "name": "Gripper"}
    if FN_ACTUATOR1 <= fn <= FN_ACTUATOR6:
        n = fn - FN_ACTUATOR1 + 1
        return {"category": "actuator", "name": f"Actuator {n}" + (" (Gripper)" if n == 1 else "")}
    if fn == FN_CAMERA_FOCUS:
        return {"category": "focus", "name": "Camera Focus"}
    if fn == FN_CAMERA_ZOOM:
        return {"category": "zoom", "name": "Camera Zoom"}
    if fn == FN_MOUNT_TILT:
        return {"category": "tilt", "name": "Camera Tilt"}
    return None


EXPECTATIONS = {
    "motor": "the thruster spinning",
    "motor_boat": "the thruster spinning in the direction shown",
    "lights": "the light ramping from dim to full brightness and back",
    "actuator": "the actuator moving through its full range (gripper opening and closing)",
    "focus": "the camera focus changing (watch the video feed)",
    "zoom": "the camera zoom changing (watch the video feed)",
    "tilt": "the camera tilting smoothly up and down through its full range",
}


class Detector:
    def __init__(self, client: MavlinkClient,
                 progress: Optional[Callable[[str, float], None]] = None):
        self.client = client
        self.progress = progress or (lambda msg, frac: None)
        self.params: dict = {}

    def _p(self, name: str, default=None):
        value = self.client.param_get(name)
        if value is None:
            value = default
        self.params[name] = value
        return value

    def firmware_version(self) -> Optional[tuple]:
        msg, _ = self.client.message("AUTOPILOT_VERSION")
        if msg is None:
            self.client.request_message(148)   # AUTOPILOT_VERSION
            for _ in range(15):
                time.sleep(0.2)
                msg, _ = self.client.message("AUTOPILOT_VERSION")
                if msg:
                    break
        if not msg or not msg.get("flight_sw_version"):
            return None
        return parse_flight_sw_version(int(msg["flight_sw_version"]))

    def run(self) -> dict:
        c = self.client
        self.progress("Waiting for autopilot heartbeat", 0.02)
        hb = c.heartbeat()
        if hb is None:
            raise RuntimeError("No autopilot heartbeat from mavlink2rest")
        mavtype = enum_value(hb, "mavtype")
        if mavtype == "MAV_TYPE_SUBMARINE":
            firmware = "ArduSub"
        elif mavtype in ("MAV_TYPE_SURFACE_BOAT", "MAV_TYPE_GROUND_ROVER"):
            firmware = "ArduRover"
        else:
            firmware = "Unknown"

        self.progress("Reading firmware version", 0.05)
        version = self.firmware_version()

        self.progress("Reading frame parameters", 0.08)
        if firmware == "ArduSub":
            frame = self._p("FRAME_CONFIG")
            model_key = "bluerov2"
            model = {1: "BlueROV2", 2: "BlueROV2 Heavy"}.get(
                int(frame) if frame is not None else -1,
                f"ArduSub vehicle (FRAME_CONFIG={frame})")
        elif firmware == "ArduRover":
            frame = self._p("FRAME_CLASS")
            model_key = "blueboat"
            model = "BlueBoat" if frame is not None and int(frame) == 2 else \
                f"ArduRover vehicle (FRAME_CLASS={frame})"
        else:
            frame, model_key, model = None, None, f"Unsupported vehicle ({mavtype})"

        outputs = []
        for n in range(1, NUM_SERVO_OUTPUTS + 1):
            self.progress(f"Reading output {n} function", 0.1 + 0.5 * n / NUM_SERVO_OUTPUTS)
            fn = self._p(f"SERVO{n}_FUNCTION")
            if fn is None:
                continue
            fn = int(fn)
            info = classify_output(fn, firmware, version)
            outputs.append({"channel": n, "function": fn,
                            "name": info["name"] if info else None})
            if info:
                info.update({"channel": n, "function": fn})
                outputs[-1]["test"] = info

        tests = []
        found = [o["test"] for o in outputs if o.get("test")]
        for i, t in enumerate(found):
            self.progress(f"Reading {t['name']} limits", 0.6 + 0.3 * i / max(1, len(found)))
            ch = t["channel"]
            t["pwm_min"] = int(self._p(f"SERVO{ch}_MIN", 1100))
            t["pwm_max"] = int(self._p(f"SERVO{ch}_MAX", 1900))
            t["pwm_trim"] = int(self._p(f"SERVO{ch}_TRIM", 1500))
            if t["category"] == "motor" and firmware == "ArduSub":
                t["direction"] = int(self._p(f"MOT_{t['motor_number']}_DIRECTION", 1) or 1)
                t["motor_index"] = t["motor_number"] - 1
            if t["category"] == "motor" and firmware == "ArduRover":
                t["motor_index"] = t["rover_motor_id"]
            t["kind"] = t["category"]
            t["id"] = f"{t['category']}_ch{ch}"
            key = "motor_boat" if t["category"] == "motor" and model_key == "blueboat" else t["category"]
            t["expect"] = EXPECTATIONS[key]
            tests.append(t)

        mount = None
        if any(t["category"] == "tilt" for t in tests):
            self.progress("Reading mount parameters", 0.92)
            mount = {
                "type": self._p("MNT1_TYPE"),
                "pitch_min": self._p("MNT1_PITCH_MIN", -45),
                "pitch_max": self._p("MNT1_PITCH_MAX", 45),
                "default_mode": self._p("MNT1_DEFLT_MODE", 3),
            }
            for t in tests:
                if t["category"] == "tilt":
                    # MNT1_TYPE 1 = Servo (stabilised), 7 = BrushlessPWM
                    # (same servo backend, no stabilisation). Either way the
                    # mount owns the output and is driven by mount commands;
                    # with no mount configured the raw servo is swept.
                    t["mount"] = mount
                    t["method"] = "mount" if mount["type"] in (1, 7) else "servo"

        tests.sort(key=lambda t: (TEST_ORDER.index(t["category"]), t["channel"]))

        self.progress("Reading battery monitor", 0.96)
        battery = {name: self._p(name) for name in
                   ("BATT_MONITOR", "BATT_VOLT_MULT", "BATT_AMP_PERVLT", "BATT_AMP_OFFSET")}

        self.progress("Done", 1.0)
        return {
            "firmware": firmware,
            "firmware_version": ".".join(map(str, version)) if version else None,
            "mavtype": mavtype,
            "frame": frame,
            "model": model,
            "model_key": model_key,
            "system_id": c.target_system,
            "outputs": outputs,
            "tests": tests,
            "mount": mount,
            "battery": battery,
            "defaults": VEHICLE_DEFAULTS.get(model_key),
        }


def calibration(measured_v: Optional[float], reported_v: Optional[float],
                measured_a: Optional[float], reported_a: Optional[float],
                params: dict, model_key: Optional[str]) -> dict:
    """Compute new BATT_VOLT_MULT / BATT_AMP_PERVLT like BlueOS BatteryCard.vue.

    newMult = measured_V * BATT_VOLT_MULT / reported_V
    newApv  = measured_A * BATT_AMP_PERVLT / reported_A
    Warnings are raised when a new value is more than CALIBRATION_TOLERANCE
    away from the vehicle's default, or the pack voltage is out of range.
    """
    defaults = VEHICLE_DEFAULTS.get(model_key or "") or {}
    out = {"warnings": [], "changes": {}}

    if measured_v and reported_v and reported_v > 0.5 and params.get("BATT_VOLT_MULT"):
        new = measured_v * params["BATT_VOLT_MULT"] / reported_v
        out["changes"]["BATT_VOLT_MULT"] = {"old": params["BATT_VOLT_MULT"], "new": round(new, 4)}
    elif measured_v:
        out["warnings"].append("Reported voltage is ~0 V; check BATT_MONITOR and the power sense wiring before calibrating voltage.")

    if measured_a and reported_a and reported_a > 0.3 and params.get("BATT_AMP_PERVLT"):
        new = measured_a * params["BATT_AMP_PERVLT"] / reported_a
        out["changes"]["BATT_AMP_PERVLT"] = {"old": params["BATT_AMP_PERVLT"], "new": round(new, 4)}
    elif measured_a:
        out["warnings"].append("Reported current is below 0.3 A; draw more current (e.g. turn on lights) to calibrate current accurately.")

    for name, change in out["changes"].items():
        default = defaults.get(name)
        if default:
            dev = abs(change["new"] - default) / default
            change["default"] = default
            change["deviation_pct"] = round(dev * 100, 1)
            if dev > CALIBRATION_TOLERANCE:
                out["warnings"].append(
                    f"{name} would become {change['new']:.4f}, {dev * 100:.0f}% away from the "
                    f"{model_key} default {default}. Check the meter reading and power sense module.")

    vr = defaults.get("voltage_range")
    if vr and measured_v and not (vr[0] <= measured_v <= vr[1]):
        out["warnings"].append(
            f"Measured voltage {measured_v:.2f} V is outside the expected {vr[0]}–{vr[1]} V range for this vehicle's battery.")
    return out
