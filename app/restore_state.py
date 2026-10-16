"""
Restore journal.

Every autopilot parameter the self-test changes is first recorded here
with its original value, and the file is fsync'd before the change is
sent. If the container dies mid-test (power loss, browser closed, crash)
the journal is replayed on the next start, so the vehicle never keeps a
test-only configuration such as SERVO13_FUNCTION=0 or MNT1_TYPE=7.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from typing import Optional

logger = logging.getLogger(__name__)

# Parameters that only take effect after an autopilot reboot.
REBOOT_REQUIRED = {"MNT1_TYPE"}


class RestoreJournal:
    def __init__(self, path: str) -> None:
        self.path = path
        self._lock = threading.Lock()
        self._data = {"params": {}}
        self._load()

    def _load(self) -> None:
        try:
            with open(self.path) as f:
                data = json.load(f)
            if isinstance(data.get("params"), dict):
                self._data = data
        except FileNotFoundError:
            pass
        except Exception as e:  # noqa: BLE001
            logger.error("Could not read restore journal %s: %s", self.path, e)

    def _save(self) -> None:
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._data, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)

    def record(self, name: str, original: float) -> None:
        """Remember the original value (first recording wins)."""
        with self._lock:
            if name not in self._data["params"]:
                self._data["params"][name] = original
                self._save()

    def forget(self, name: str) -> None:
        with self._lock:
            if self._data["params"].pop(name, None) is not None:
                self._save()

    def pending(self) -> dict:
        with self._lock:
            return dict(self._data["params"])

    def original(self, name: str) -> Optional[float]:
        with self._lock:
            return self._data["params"].get(name)

    def needs_reboot(self) -> bool:
        return any(n in REBOOT_REQUIRED for n in self.pending())
