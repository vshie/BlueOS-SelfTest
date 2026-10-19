"""
MAVLink access through BlueOS's mavlink2rest service.

Everything the self-test does to the autopilot goes through here:
COMMAND_LONG (servo, motor test, arm, mode, mount, reboot), parameter
read/write, and reading cached telemetry messages.

The parameter helpers are ported from vshie/SubReels_TowFish
(app/mavlink_params.py). Protocol notes worth repeating:

* mavlink2rest answers HTTP 200 with a body starting "Failed" when it
  cannot parse a message, so the body is checked as well as the status.
* Message bodies are built from the server's own template
  (``/helper/mavlink?name=<TYPE>``) so field names match its dialect.
* mavlink2rest caches exactly one PARAM_VALUE per component, so reads are
  "request, then poll the mailbox until the name matches", and writes
  also require the mailbox ``last_update`` stamp to change.
"""

from __future__ import annotations

import copy
import json
import logging
import threading
import time
from typing import Optional

import requests

logger = logging.getLogger(__name__)

# The container runs with NetworkMode=host, so BlueOS's nginx is on
# localhost. host.docker.internal is kept as a fallback for setups that
# run the extension in bridge mode.
CANDIDATE_BASE_URLS = (
    "http://127.0.0.1/mavlink2rest",
    "http://host.docker.internal/mavlink2rest",
    "http://127.0.0.1:6040",
)

PARAM_ID_LEN = 16
DEFAULT_PARAM_TYPE = "MAV_PARAM_TYPE_REAL32"

_REAL_AUTOPILOTS = ("MAV_AUTOPILOT_ARDUPILOTMEGA", "MAV_AUTOPILOT_PX4")


class MavlinkError(Exception):
    """Raised when the autopilot does not answer in time."""


def chars_to_str(param_id_chars) -> str:
    if not isinstance(param_id_chars, (list, tuple)):
        return str(param_id_chars or "").rstrip("\x00")
    return "".join(c for c in param_id_chars if c and c != "\x00")


def str_to_chars(param_id: str, pad_len: int = PARAM_ID_LEN) -> list:
    chars = list(param_id)[:pad_len]
    chars.extend("\x00" for _ in range(pad_len - len(chars)))
    return chars


def enum_value(message: dict, field: str) -> str:
    """Pull a mavlink2rest enum ``type`` string out of a message field."""
    value = (message or {}).get(field)
    if isinstance(value, dict):
        return str(value.get("type") or "")
    return str(value or "")


# HEARTBEAT mavtypes for each BlueOS firmware type
FIRMWARE_MAVTYPES = {
    "ArduSub": ("MAV_TYPE_SUBMARINE",),
    "ArduRover": ("MAV_TYPE_SURFACE_BOAT", "MAV_TYPE_GROUND_ROVER"),
}


def pick_autopilot(vehicles, prefer_mavtypes=()) -> Optional[tuple]:
    """Return ``(system_id, component_id)`` of a real autopilot, or None.

    A boat towing a BlueOS towfish can see both autopilots on the shared
    MAVLink network; ``prefer_mavtypes`` picks the one matching this
    vehicle's own firmware."""
    if not isinstance(vehicles, dict):
        return None
    found = []
    for vid, vehicle in vehicles.items():
        try:
            sysid = int(vid)
        except (TypeError, ValueError):
            continue
        components = (vehicle or {}).get("components") or {}
        for cid, component in components.items():
            try:
                comp = int(cid)
            except (TypeError, ValueError):
                continue
            heartbeat = ((((component or {}).get("messages") or {})
                          .get("HEARTBEAT") or {}).get("message") or {})
            if enum_value(heartbeat, "autopilot") in _REAL_AUTOPILOTS:
                found.append((sysid, comp, enum_value(heartbeat, "mavtype")))
    for sysid, comp, mavtype in found:
        if mavtype in prefer_mavtypes:
            return sysid, comp
    return (found[0][0], found[0][1]) if found else None


class MavlinkClient:
    """Thin, thread-safe wrapper around one mavlink2rest instance."""

    def __init__(self, base_url: Optional[str] = None,
                 gcs_system_id: int = 255, gcs_component_id: int = 240,
                 http_timeout_s: float = 2.0) -> None:
        self.base_url = base_url.rstrip("/") if base_url else None
        self.target_system = 1
        self.target_component = 1
        self.gcs_system_id = gcs_system_id
        self.gcs_component_id = gcs_component_id
        self.http_timeout_s = http_timeout_s
        self._template_cache: dict = {}
        self._param_lock = threading.Lock()
        self._session = requests.Session()

    # -- connection ---------------------------------------------------------
    def connect(self) -> bool:
        """Find mavlink2rest and the autopilot's system/component id."""
        candidates = [self.base_url] if self.base_url else list(CANDIDATE_BASE_URLS)
        for base in candidates:
            vehicles = self._get_json_abs(f"{base}/mavlink/vehicles")
            root = base.rsplit("/mavlink2rest", 1)[0].replace(":6040", "")
            firmware = self._get_json_abs(f"{root}/ardupilot-manager/v1.0/firmware_vehicle_type")
            found = pick_autopilot(vehicles, FIRMWARE_MAVTYPES.get(firmware, ()) if isinstance(firmware, str) else ())
            if found:
                self.base_url = base
                self.target_system, self.target_component = found
                logger.info("Autopilot at %s sys=%s comp=%s", base, *found)
                return True
        return False

    @property
    def connected(self) -> bool:
        return self.base_url is not None

    # -- HTTP ---------------------------------------------------------------
    def _get_json_abs(self, url: str, timeout_s: Optional[float] = None) -> Optional[dict]:
        try:
            r = self._session.get(url, timeout=timeout_s or self.http_timeout_s)
            if r.status_code != 200:
                return None
            body = (r.text or "").strip()
            if not body or body == "None":
                return None
            return json.loads(body)
        except Exception as e:  # noqa: BLE001 - network errors are expected
            logger.debug("GET %s failed: %s", url, e)
            return None

    def _get_json(self, path: str) -> Optional[dict]:
        if not self.base_url:
            return None
        return self._get_json_abs(f"{self.base_url}{path}")

    def _post(self, msg: dict, info: str) -> bool:
        if not self.base_url:
            return False
        try:
            r = self._session.post(f"{self.base_url}/mavlink", json=msg,
                                   timeout=self.http_timeout_s)
            if r.status_code != 200:
                logger.warning("POST %s -> HTTP %s: %s", info, r.status_code, r.text[:200])
                return False
            if (r.text or "").strip().lower().startswith("failed"):
                logger.warning("mavlink2rest rejected %s: %s", info, r.text[:200])
                return False
            return True
        except Exception as e:  # noqa: BLE001
            logger.warning("POST %s failed: %s", info, e)
            return False

    def _envelope(self, message: dict) -> dict:
        msg_type = message["type"]
        template = self._template_cache.get(msg_type)
        if template is None:
            fetched = self._get_json(f"/helper/mavlink?name={msg_type}")
            if isinstance(fetched, dict) and "message" in fetched:
                self._template_cache[msg_type] = fetched
                template = fetched
        body = message
        if template is not None:
            body = copy.deepcopy(template["message"])
            body.update(message)
        return {
            "header": {"system_id": self.gcs_system_id,
                       "component_id": self.gcs_component_id, "sequence": 0},
            "message": body,
        }

    # -- telemetry ----------------------------------------------------------
    def message(self, name: str) -> tuple:
        """Return ``(message, last_update)`` for a cached message."""
        wrapper = self._get_json(
            f"/mavlink/vehicles/{self.target_system}"
            f"/components/{self.target_component}/messages/{name}")
        if not isinstance(wrapper, dict):
            return None, None
        msg = wrapper.get("message")
        stamp = (((wrapper.get("status") or {}).get("time") or {}).get("last_update"))
        return (msg if isinstance(msg, dict) else None), stamp

    def heartbeat(self) -> Optional[dict]:
        return self.message("HEARTBEAT")[0]

    def is_armed(self) -> bool:
        hb = self.heartbeat() or {}
        bits = (hb.get("base_mode") or {}).get("bits", 0) if isinstance(hb.get("base_mode"), dict) else 0
        return bool(int(bits) & 128)

    def custom_mode(self) -> Optional[int]:
        hb = self.heartbeat()
        return None if hb is None else int(hb.get("custom_mode", 0))

    # -- commands -----------------------------------------------------------
    def command_long(self, command: str, params: Optional[dict] = None) -> bool:
        params = params or {}
        message = {
            "type": "COMMAND_LONG",
            "command": {"type": command},
            "target_system": self.target_system,
            "target_component": self.target_component,
            "confirmation": 0,
        }
        for index in range(1, 8):
            message[f"param{index}"] = float(params.get(index, 0.0))
        return self._post(self._envelope(message), command)

    def command_ack(self, command: str, timeout_s: float = 3.0,
                    params: Optional[dict] = None) -> Optional[str]:
        """Send a command and wait for its COMMAND_ACK; returns result name."""
        _, before = self.message("COMMAND_ACK")
        if not self.command_long(command, params):
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg, stamp = self.message("COMMAND_ACK")
            if msg and stamp != before and enum_value(msg, "command") == command:
                return enum_value(msg, "result")
            time.sleep(0.1)
        return None

    def set_servo(self, channel: int, pwm: int) -> bool:
        return self.command_long("MAV_CMD_DO_SET_SERVO", {1: int(channel), 2: int(pwm)})

    def set_mode(self, custom_mode: int) -> Optional[str]:
        # p1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        return self.command_ack("MAV_CMD_DO_SET_MODE", params={1: 1, 2: int(custom_mode)})

    def arm(self, force: bool = True) -> Optional[str]:
        return self.command_ack("MAV_CMD_COMPONENT_ARM_DISARM",
                                params={1: 1, 2: 2989 if force else 0})

    def disarm(self, force: bool = True) -> Optional[str]:
        return self.command_ack("MAV_CMD_COMPONENT_ARM_DISARM",
                                params={1: 0, 2: 21196 if force else 0})

    def motor_test(self, motor: int, pwm: int, timeout_s: float = 1.0) -> bool:
        """One MAV_CMD_DO_MOTOR_TEST frame (PWM throttle, board order).

        ArduSub needs these at >= 2 Hz or it ends the test and disarms;
        callers stream at 10 Hz like the BlueOS PWM setup page does.
        """
        return self.command_long("MAV_CMD_DO_MOTOR_TEST", {
            1: motor, 2: 1, 3: pwm, 4: timeout_s, 5: 1, 6: 2, 7: 0})

    def reboot_autopilot(self) -> Optional[str]:
        return self.command_ack("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", params={1: 1})

    def mount_pitch(self, pitch_deg: float) -> None:
        """Point the mount; both the new and legacy commands are sent so it
        works on ArduSub 4.1 through 4.7."""
        nan = float("nan")
        self.command_long("MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW",
                          {1: pitch_deg, 2: 0, 3: nan, 4: nan, 5: 0, 7: 0})
        # p7 = MAV_MOUNT_MODE_MAVLINK_TARGETING
        self.command_long("MAV_CMD_DO_MOUNT_CONTROL", {1: pitch_deg, 2: 0, 3: 0, 7: 2})

    def mount_mode(self, mode: int) -> None:
        self.command_long("MAV_CMD_DO_MOUNT_CONFIGURE", {1: mode})

    def request_message(self, msg_id: int) -> None:
        self.command_long("MAV_CMD_REQUEST_MESSAGE", {1: msg_id})

    # -- parameters ---------------------------------------------------------
    @staticmethod
    def _decode_param(message: dict) -> dict:
        return {"name": chars_to_str(message.get("param_id")),
                "value": message.get("param_value")}

    def _request_read(self, param_id: str) -> bool:
        return self._post(self._envelope({
            "type": "PARAM_REQUEST_READ",
            "target_system": self.target_system,
            "target_component": self.target_component,
            "param_index": -1,
            "param_id": str_to_chars(param_id),
        }), f"PARAM_REQUEST_READ:{param_id}")

    def _await_param(self, param_id, deadline, reject_stamp, repoke, want=None):
        last_poke = time.monotonic()
        while time.monotonic() < deadline:
            msg, stamp = self.message("PARAM_VALUE")
            if msg is not None:
                decoded = self._decode_param(msg)
                fresh = reject_stamp is None or stamp != reject_stamp
                matches = want is None or (decoded["value"] is not None and
                                           abs(float(decoded["value"]) - want) <= 1e-3 * max(1.0, abs(want)))
                if decoded["name"] == param_id and fresh and matches:
                    return decoded
            if time.monotonic() - last_poke >= 1.0:
                repoke()
                last_poke = time.monotonic()
            time.sleep(0.08)
        return None

    def param_get(self, param_id: str, timeout_s: float = 2.5) -> Optional[float]:
        """Read one parameter; returns None if it doesn't exist / no answer."""
        with self._param_lock:
            deadline = time.monotonic() + timeout_s
            if not self._request_read(param_id):
                raise MavlinkError("mavlink2rest unreachable")
            result = self._await_param(param_id, deadline, None,
                                       lambda: self._request_read(param_id))
        return None if result is None else float(result["value"])

    def param_set(self, param_id: str, value: float, timeout_s: float = 5.0) -> float:
        """Write one parameter and return the autopilot's read-back."""
        message = {
            "type": "PARAM_SET",
            "target_system": self.target_system,
            "target_component": self.target_component,
            "param_id": str_to_chars(param_id),
            "param_value": float(value),
            "param_type": {"type": DEFAULT_PARAM_TYPE},
        }

        def send():
            return self._post(self._envelope(message), f"PARAM_SET:{param_id}={value}")

        with self._param_lock:
            _, before = self.message("PARAM_VALUE")
            deadline = time.monotonic() + timeout_s
            if not send():
                raise MavlinkError("mavlink2rest unreachable")
            # mavlink2rest can deliver a late duplicate of an earlier
            # PARAM_VALUE (e.g. the reply to the read just before this
            # write), so only a reply carrying the new value counts as the ack.
            result = self._await_param(param_id, deadline, before, send,
                                       want=float(value))
        if result is None:
            actual = self.param_get(param_id)
            if actual is not None and abs(actual - float(value)) <= 1e-3 * max(1.0, abs(value)):
                return actual
            raise MavlinkError(f"{param_id} write not confirmed (reads back {actual})")
        return float(result["value"])

    # -- BlueOS helpers -----------------------------------------------------
    def blueos_get(self, path: str, timeout_s: Optional[float] = None) -> Optional[dict]:
        """GET another BlueOS service on the same nginx (e.g. /beacon/...)."""
        if not self.base_url:
            return None
        root = self.base_url.rsplit("/mavlink2rest", 1)[0]
        if ":6040" in root:
            root = root.replace(":6040", "")
        return self._get_json_abs(f"{root}{path}", timeout_s)
