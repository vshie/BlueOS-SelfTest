"""
Per-run test log.

Each wizard run writes ``selftest_YYYYMMDD_HHMMSS_<vehicle>.log`` (human
readable) and a ``.json`` sidecar into the extension's data folder, which
BlueOS exposes in its File Browser under ``extensions/selftest``.
"""

from __future__ import annotations

import json
import os
import re
import threading
from datetime import datetime, timezone
from typing import Optional


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")


class RunLog:
    def __init__(self, log_dir: str, vehicle: dict, meta: dict) -> None:
        self.log_dir = log_dir
        self._lock = threading.Lock()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        slug = re.sub(r"[^A-Za-z0-9]+", "-", vehicle.get("model") or "vehicle").strip("-")
        self.basename = f"selftest_{stamp}_{slug}"
        self.text_path = os.path.join(log_dir, self.basename + ".log")
        self.json_path = os.path.join(log_dir, self.basename + ".json")
        self.data = {
            "started": _now(),
            "finished": None,
            "meta": meta,
            "vehicle": {k: vehicle.get(k) for k in (
                "model", "firmware", "firmware_version", "frame", "system_id",
                "battery", "outputs")},
            "network": None,
            "devices": None,
            "calibration": [],
            "tests": [],
            "events": [],
        }
        self._write_header()
        self._flush_json()

    # -- writers ------------------------------------------------------------
    def _append(self, text: str) -> None:
        with open(self.text_path, "a") as f:
            f.write(text.rstrip("\n") + "\n")

    def _flush_json(self) -> None:
        tmp = self.json_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.data, f, indent=2, default=str)
        os.replace(tmp, self.json_path)

    def _write_header(self) -> None:
        v, m = self.data["vehicle"], self.data["meta"]
        lines = [
            "=" * 64,
            "  Blue Robotics BlueOS Self Test",
            "=" * 64,
            f"Started        : {self.data['started']}",
            f"Vehicle        : {v.get('model')}",
            f"Firmware       : {v.get('firmware')} {v.get('firmware_version') or ''}".rstrip(),
            f"Frame          : {v.get('frame')}",
            f"MAVLink sysid  : {v.get('system_id')}",
        ]
        for key, label in (("vehicle_name", "Vehicle name"), ("blueos_version", "BlueOS"),
                           ("operator", "Operator"), ("serial", "Serial number"),
                           ("notes", "Notes"), ("extension_version", "Extension")):
            if m.get(key):
                lines.append(f"{label:<15}: {m[key]}")
        lines.append("")
        lines.append("Detected outputs:")
        for o in v.get("outputs") or []:
            if o.get("name"):
                lines.append(f"  SERVO{o['channel']:<2}  function {o['function']:<4} {o['name']}")
        lines.append("")
        self._append("\n".join(lines))

    # -- public -------------------------------------------------------------
    def event(self, text: str) -> None:
        with self._lock:
            self.data["events"].append({"time": _now(), "text": text})
            self._append(f"[{_now()}] {text}")
            self._flush_json()

    def network(self, results: dict) -> None:
        with self._lock:
            self.data["network"] = results
            lat = results.get("latency") or {}
            lines = ["--- Network test (browser <-> vehicle) ---"]
            if results.get("client"):
                lines.append(f"  Client          : {results['client']}")
            if lat:
                lines.append(f"  Latency (ms)    : min {lat.get('min')}  avg {lat.get('avg')}  "
                             f"max {lat.get('max')}  jitter {lat.get('jitter')}  "
                             f"lost {lat.get('lost', 0)}/{lat.get('sent', 0)}")
            if results.get("download_mbps") is not None:
                lines.append(f"  Download (Mbps) : {results['download_mbps']}")
            if results.get("upload_mbps") is not None:
                lines.append(f"  Upload (Mbps)   : {results['upload_mbps']}")
            if results.get("error"):
                lines.append(f"  Error           : {results['error']}")
            if results.get("verdict"):
                lines.append(f"  Result          : {results['verdict']}")
            self._append("\n".join(lines) + "\n")
            self._flush_json()

    def devices(self, result: dict) -> None:
        with self._lock:
            self.data["devices"] = result
            lines = ["--- Network devices & sonars ---", f"  Result          : {result.get('result')}",
                     f"  Summary         : {result.get('summary')}"]
            for c in result.get("checks") or []:
                lines.append(f"  [{c['result']:<4}] {c['name']}: {c['detail']}")
            lines.append("  Hosts:")
            for h in result.get("hosts") or []:
                ports = ",".join(map(str, h.get("open_ports") or [])) or "-"
                lines.append(f"    {h['ip']:<15} {h.get('mac') or '-':<17}  ports {ports:<20} {h.get('identified') or ''}")
            self._append("\n".join(lines) + "\n")
            self._flush_json()

    def calibration(self, entry: dict) -> None:
        with self._lock:
            self.data["calibration"].append(entry)
            lines = ["--- Power sensor calibration ---"]
            lines.append(f"  Reported        : {entry.get('reported_v')} V, {entry.get('reported_a')} A")
            lines.append(f"  Measured        : {entry.get('measured_v')} V, {entry.get('measured_a')} A")
            for name, ch in (entry.get("changes") or {}).items():
                lines.append(f"  {name:<16}: {ch.get('old')} -> {ch.get('new')}"
                             + (" (applied)" if entry.get("applied") else " (not applied)"))
            for w in entry.get("warnings") or []:
                lines.append(f"  WARNING         : {w}")
            if entry.get("skipped"):
                lines.append("  Skipped by user")
            self._append("\n".join(lines) + "\n")
            self._flush_json()

    def test(self, result: dict) -> None:
        with self._lock:
            self.data["tests"].append(result)
            line = f"[{result.get('result'):<7}] {result.get('name')} (SERVO{result.get('channel')})"
            if result.get("fail_at_s") is not None:
                line += f" - flagged at {result['fail_at_s']:.1f} s"
            if result.get("error"):
                line += f" - {result['error']}"
            self._append(line)
            for change in result.get("param_changes") or []:
                self._append(f"            {change}")
            self._flush_json()

    def finish(self, summary: dict) -> None:
        with self._lock:
            self.data["finished"] = _now()
            self.data["summary"] = summary
            lines = ["", "--- Summary ---"]
            for key in ("passed", "failed", "skipped", "errors"):
                lines.append(f"  {key.capitalize():<8}: {summary.get(key, 0)}")
            lines.append(f"  Overall : {summary.get('overall')}")
            lines.append(f"Finished {self.data['finished']}")
            self._append("\n".join(lines))
            self._flush_json()

    @property
    def files(self) -> list:
        return [os.path.basename(self.text_path), os.path.basename(self.json_path)]


def list_logs(log_dir: str) -> list:
    out = []
    try:
        for name in sorted(os.listdir(log_dir), reverse=True):
            if name.startswith("selftest_") and name.endswith(".log"):
                path = os.path.join(log_dir, name)
                out.append({"name": name, "size": os.path.getsize(path)})
    except FileNotFoundError:
        pass
    return out


def read_log(log_dir: str, name: str) -> Optional[str]:
    if not re.fullmatch(r"selftest_[A-Za-z0-9_\-]+\.(log|json)", name):
        return None
    try:
        with open(os.path.join(log_dir, name)) as f:
            return f.read()
    except FileNotFoundError:
        return None
