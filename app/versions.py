"""
Firmware and BlueOS version checks.

The autopilot firmware is compared with the newest STABLE release that
BlueOS's own firmware page offers for this board
(/ardupilot-manager/v1.0/available_firmwares). That list comes from
firmware.ardupilot.org, so without internet the built-in
KNOWN_LATEST_STABLE values are used instead (refresh them each release).
"""

from __future__ import annotations

import logging
import re
import time
from typing import Optional

logger = logging.getLogger(__name__)

MIN_BLUEOS = (1, 4, 6)

# Fallback when the vehicle can't reach firmware.ardupilot.org.
KNOWN_LATEST_STABLE = {"ArduSub": (4, 7, 2), "ArduRover": (4, 7, 2)}

FIRMWARE_VEHICLE = {"ArduSub": "Sub", "ArduRover": "Rover"}

# BlueOS UI pages the user is sent to for updates.
AUTOPILOT_PAGE = "/vehicle/autopilot"
VERSION_PAGE = "/tools/version-chooser"

_cache: dict = {}
CACHE_S = 3600


def parse_version(text) -> Optional[tuple]:
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", str(text or ""))
    return tuple(int(x) for x in m.groups()) if m else None


def fmt(v: Optional[tuple]) -> str:
    return ".".join(map(str, v)) if v else "unknown"


def latest_stable(client, firmware: str) -> tuple:
    """Return ``(version, source)`` for the newest STABLE build for this board."""
    vehicle = FIRMWARE_VEHICLE.get(firmware)
    if not vehicle:
        return None, "unsupported firmware"
    cached = _cache.get(firmware)
    if cached and time.monotonic() - cached[2] < CACHE_S:
        return cached[0], cached[1]
    board = client.blueos_get("/ardupilot-manager/v1.0/board") or {}
    board_name = board.get("name") if isinstance(board, dict) else None
    versions = []
    if board_name:
        listing = client.blueos_get(
            f"/ardupilot-manager/v1.0/available_firmwares?vehicle={vehicle}&board_name={board_name}",
            timeout_s=30)
        for fw in listing if isinstance(listing, list) else []:
            name = str(fw.get("name", ""))
            if name.upper().startswith("STABLE"):
                v = parse_version(name)
                if v:
                    versions.append(v)
    if versions:
        result = (max(versions), f"ardupilot.org for {board_name}")
        _cache[firmware] = (*result, time.monotonic())
        return result
    return KNOWN_LATEST_STABLE.get(firmware), "built-in list (vehicle offline)"


def check_versions(client, vehicle: dict) -> dict:
    checks = []

    firmware = vehicle.get("firmware")
    current = parse_version(vehicle.get("firmware_version"))
    latest, source = latest_stable(client, firmware)
    name = f"{firmware} firmware"
    if not current:
        checks.append({"name": name, "result": "WARN", "current": None, "latest": fmt(latest),
                       "detail": "Could not read the firmware version", "update_page": AUTOPILOT_PAGE})
    elif latest and current < latest:
        checks.append({"name": name, "result": "WARN", "current": fmt(current), "latest": fmt(latest),
                       "detail": f"{fmt(current)} installed; latest stable is {fmt(latest)} ({source}). "
                                 "Update in BlueOS → Autopilot Firmware.",
                       "update_page": AUTOPILOT_PAGE})
    else:
        checks.append({"name": name, "result": "PASS", "current": fmt(current), "latest": fmt(latest),
                       "detail": f"{fmt(current)} is the latest stable ({source})"})

    ver = client.blueos_get("/version-chooser/v1.0/version/current")
    tag = ver.get("tag") if isinstance(ver, dict) else None
    blueos = parse_version(tag)
    if blueos is None:
        checks.append({"name": "BlueOS", "result": "WARN", "current": tag, "latest": fmt(MIN_BLUEOS),
                       "detail": f"Could not determine the BlueOS version ({tag or 'unavailable'}); "
                                 f"{fmt(MIN_BLUEOS)} or newer is required",
                       "update_page": VERSION_PAGE})
    elif blueos < MIN_BLUEOS:
        checks.append({"name": "BlueOS", "result": "WARN", "current": tag, "latest": fmt(MIN_BLUEOS),
                       "detail": f"BlueOS {tag} is older than {fmt(MIN_BLUEOS)}. "
                                 "Update in BlueOS → Version Chooser.",
                       "update_page": VERSION_PAGE})
    else:
        checks.append({"name": "BlueOS", "result": "PASS", "current": tag, "latest": fmt(MIN_BLUEOS),
                       "detail": f"BlueOS {tag} (≥ {fmt(MIN_BLUEOS)} required)"})

    overall = "WARN" if any(c["result"] != "PASS" for c in checks) else "PASS"
    return {"result": overall, "checks": checks}
