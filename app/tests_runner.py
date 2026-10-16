"""
Hardware test execution.

All actuation is driven from the backend (not the browser) so timing is
steady and every exit path -- pass, SPACE-flagged fail, abort, browser
closed, exception -- goes through the same stop-and-restore code.

Safety model
------------
* The browser polls ``/api/test/state`` (which counts as a keepalive)
  several times a second. If keepalives stop for WATCHDOG_S the running
  test is aborted, outputs return to neutral, the vehicle is disarmed and
  every changed parameter is restored.
* Every parameter change is first written to the RestoreJournal, which is
  replayed at container start if a run was interrupted.
* Only one test runs at a time.

Motors
------
ArduSub only accepts MAV_CMD_DO_MOTOR_TEST while armed and ends the test
(disarm + 10 s cooldown) if commands stop for 500 ms, so a *motor session*
is kept alive across consecutive motor tests: armed in MANUAL, streaming
at 10 Hz, idle motors held at neutral 1500 us. The session ends (disarm,
mode restored) when a non-motor test starts, after MOTOR_SESSION_IDLE_S
without a motor test, or when the watchdog fires. ArduRover arms itself
for motor tests and uses the same streaming loop.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import traceback
from typing import Callable, Optional

from mavlink import MavlinkClient, MavlinkError
from restore_state import RestoreJournal

logger = logging.getLogger(__name__)

TEST_DURATION_S = 5.0
WATCHDOG_S = 1.5
STREAM_HZ = 10.0
MOTOR_SESSION_IDLE_S = 20.0
SUB_MOTOR_COOLDOWN_S = 11.0     # ArduSub: 10 s after a motor test timeout
SUB_MANUAL_MODE = 19
NEUTRAL_PWM = 1500
DEFAULT_MOTOR_OFFSET = 100      # 1600 us: gentle, clearly visible spin
REBOOT_TIMEOUT_S = 120.0


class TestAborted(Exception):
    pass


def triangle(frac: float) -> float:
    """0 -> 1 -> 0 over frac in [0, 1]."""
    frac = min(max(frac, 0.0), 1.0)
    return 1.0 - abs(2.0 * frac - 1.0)


def tilt_angle(frac: float, pitch_min: float, pitch_max: float) -> float:
    """0 -> max -> min -> 0 (one sine period), honouring asymmetric limits."""
    s = math.sin(2.0 * math.pi * min(max(frac, 0.0), 1.0))
    return s * pitch_max if s >= 0 else -s * pitch_min


class Runner:
    def __init__(self, client: MavlinkClient, journal: RestoreJournal,
                 on_result: Callable[[dict], None],
                 on_event: Callable[[str], None]) -> None:
        self.client = client
        self.journal = journal
        self.on_result = on_result
        self.on_event = on_event
        self._lock = threading.RLock()
        self._state = self._idle_state()
        self._thread: Optional[threading.Thread] = None
        self._abort = threading.Event()
        self._fail = threading.Event()
        self._last_keepalive = time.monotonic()
        self._vehicle: dict = {}
        self.settings = {"duration_s": TEST_DURATION_S, "motor_offset": DEFAULT_MOTOR_OFFSET}

        # motor session
        self._ms_lock = threading.Lock()
        self._ms_thread: Optional[threading.Thread] = None
        self._ms_stop = threading.Event()
        self._ms_target: Optional[tuple] = None        # (motor_index, pwm)
        self._ms_touched: set = set()
        self._ms_last_use = 0.0
        self._ms_saved_mode: Optional[int] = None
        self._sub_motor_end = 0.0

        threading.Thread(target=self._watchdog, daemon=True, name="watchdog").start()

    # -- state ----------------------------------------------------------------
    @staticmethod
    def _idle_state() -> dict:
        return {"test_id": None, "phase": "idle", "message": "", "elapsed": 0.0,
                "duration": TEST_DURATION_S, "result": None, "fail_at_s": None,
                "error": None, "pwm": None, "value_label": None}

    def _set(self, **kw) -> None:
        with self._lock:
            self._state.update(kw)

    def state(self, keepalive: bool = True) -> dict:
        if keepalive:
            self.keepalive()
        with self._lock:
            s = dict(self._state)
        s["motor_session"] = self._ms_thread is not None and self._ms_thread.is_alive()
        s["restore_pending"] = self.journal.pending()
        return s

    def keepalive(self) -> None:
        self._last_keepalive = time.monotonic()

    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def set_vehicle(self, vehicle: dict) -> None:
        self._vehicle = vehicle or {}

    # -- public control ---------------------------------------------------------
    def start(self, test: dict) -> Optional[str]:
        if self.busy():
            return "Another test is still running"
        if self.journal.pending():
            # A previous run left parameters changed; restore first.
            self.restore_all()
        self._abort.clear()
        self._fail.clear()
        self.keepalive()
        with self._lock:
            self._state = self._idle_state()
            self._state.update(test_id=test["id"], phase="preparing",
                               message="Preparing", duration=self.settings["duration_s"])
        self._thread = threading.Thread(target=self._run, args=(test,), daemon=True,
                                        name=f"test-{test['id']}")
        self._thread.start()
        return None

    def flag_fail(self) -> None:
        with self._lock:
            if self._state["phase"] == "running" and not self._fail.is_set():
                self._state["fail_at_s"] = round(self._state["elapsed"], 2)
                self._fail.set()

    def abort(self, reason: str = "Aborted by user") -> None:
        if self.busy():
            self._set(error=reason)
            self._abort.set()
        self.end_motor_session(reason)

    # -- watchdog ----------------------------------------------------------------
    def _watchdog(self) -> None:
        while True:
            time.sleep(0.25)
            stale = time.monotonic() - self._last_keepalive > WATCHDOG_S
            ms_alive = self._ms_thread is not None and self._ms_thread.is_alive()
            if stale and (self.busy() or ms_alive):
                if not self._abort.is_set():
                    logger.warning("Browser keepalive lost - aborting test")
                    self.on_event("Browser connection lost - test aborted, outputs stopped")
                    self.abort("Browser connection lost")
            if ms_alive and not self.busy() and \
                    time.monotonic() - self._ms_last_use > MOTOR_SESSION_IDLE_S:
                self.end_motor_session("Motor session idle")

    # -- helpers -------------------------------------------------------------------
    def _check(self) -> None:
        if self._abort.is_set():
            raise TestAborted()

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check()
            time.sleep(min(0.05, max(0.0, end - time.monotonic())))

    def _set_param(self, name: str, value: float, changes: list) -> None:
        original = self.client.param_get(name)
        if original is None:
            raise MavlinkError(f"Could not read {name}")
        self.journal.record(name, original)
        readback = self.client.param_set(name, value)
        changes.append(f"{name}: {original:g} -> {readback:g} (temporary)")

    def _restore_param(self, name: str, changes: list) -> None:
        original = self.journal.original(name)
        if original is None:
            return
        readback = self.client.param_set(name, original)
        if abs(readback - original) > 1e-3:
            raise MavlinkError(f"{name} restore read back {readback}, expected {original}")
        self.journal.forget(name)
        changes.append(f"{name}: restored to {original:g}")

    def _sweep_loop(self, step: Callable[[float], None]) -> Optional[float]:
        """Run ``step(frac)`` at STREAM_HZ for the test duration.

        Returns the elapsed time at which SPACE was pressed, or None."""
        duration = self.settings["duration_s"]
        self._set(phase="running", message="Running", elapsed=0.0)
        t0 = time.monotonic()
        period = 1.0 / STREAM_HZ
        while True:
            self._check()
            elapsed = time.monotonic() - t0
            if self._fail.is_set():
                return self._state["fail_at_s"]
            if elapsed >= duration:
                return None
            self._set(elapsed=round(elapsed, 2))
            step(elapsed / duration)
            time.sleep(max(0.0, period - ((time.monotonic() - t0) % period)))

    # -- main test thread ------------------------------------------------------------
    def _run(self, test: dict) -> None:
        changes: list = []
        result = {"id": test["id"], "name": test["name"], "channel": test["channel"],
                  "category": test["category"], "function": test["function"],
                  "result": None, "fail_at_s": None, "error": None,
                  "param_changes": changes, "time": time.strftime("%H:%M:%S")}
        fail_at = None
        try:
            kind = test["kind"]
            if kind == "motor":
                fail_at = self._run_motor(test)
            else:
                self.end_motor_session("Starting non-motor test")
                if kind == "tilt" and test.get("method") == "mount":
                    fail_at = self._run_mount_tilt(test, changes)
                else:
                    fail_at = self._run_servo_sweep(test, changes)
            result["result"] = "FAIL" if fail_at is not None else "PASS"
            result["fail_at_s"] = fail_at
        except TestAborted:
            result["result"] = "ABORTED"
            result["error"] = self._state.get("error") or "Aborted"
        except Exception as e:  # noqa: BLE001
            logger.error("Test %s failed: %s\n%s", test["id"], e, traceback.format_exc())
            result["result"] = "ERROR"
            result["error"] = str(e)
        finally:
            self._finish(test, changes, result)

    def _finish(self, test: dict, changes: list, result: dict) -> None:
        self._set(phase="restoring", message="Restoring vehicle settings")
        if self._abort.is_set():
            self.end_motor_session("Test aborted")
        try:
            self._restore_test(test, changes)
        except Exception as e:  # noqa: BLE001
            logger.error("Restore after %s failed: %s", test["id"], e)
            result["error"] = (result.get("error") or "") + f" | RESTORE FAILED: {e}"
            if result["result"] in ("PASS", "FAIL"):
                result["result"] = "ERROR"
        self.on_result(result)
        self._set(phase="done", message="", result=result["result"],
                  fail_at_s=result["fail_at_s"], error=result["error"])

    def _restore_test(self, test: dict, changes: list) -> None:
        kind = test["kind"]
        if kind == "motor":
            with self._ms_lock:
                self._ms_target = None
            self._ms_last_use = time.monotonic()
            return
        if kind == "tilt" and test.get("method") == "mount":
            mount = test.get("mount") or {}
            self.client.mount_pitch(0)
            if self.journal.original("MNT1_TYPE") is not None:
                self._restore_param("MNT1_TYPE", changes)
                self._reboot_and_wait(changes, check_abort=False)
            else:
                self.client.mount_mode(int(mount.get("default_mode") or 3))
            return
        # servo sweep: park output, then hand it back to its function
        park = test["pwm_min"] if test["category"] == "lights" else test["pwm_trim"]
        for _ in range(3):
            self.client.set_servo(test["channel"], park)
            time.sleep(0.05)
        self._restore_param(f"SERVO{test['channel']}_FUNCTION", changes)

    # -- servo sweeps (lights, gripper/actuators, focus, zoom, raw tilt) ----------------
    def _run_servo_sweep(self, test: dict, changes: list) -> Optional[float]:
        ch = test["channel"]
        name = f"SERVO{ch}_FUNCTION"
        current = self.client.param_get(name)
        if current is None:
            raise MavlinkError(f"Could not read {name}")
        if int(current) != test["function"] and self.journal.original(name) is None:
            raise MavlinkError(f"{name} is {current:g}, expected {test['function']}; re-run detection")
        # DO_SET_SERVO is refused on outputs owned by a function (lights,
        # actuators, mount...), so the output is temporarily disabled
        # (function 0) for the sweep and restored afterwards.
        self._set(message=f"Taking control of output {ch}")
        self._set_param(name, 0, changes)
        lo, hi = test["pwm_min"], test["pwm_max"]

        def step(frac):
            pwm = int(lo + (hi - lo) * triangle(frac))
            self._set(pwm=pwm, value_label=f"{pwm} µs")
            self.client.set_servo(ch, pwm)

        return self._sweep_loop(step)

    # -- camera tilt through the mount driver ----------------------------------------
    def _run_mount_tilt(self, test: dict, changes: list) -> Optional[float]:
        mount = test.get("mount") or {}
        if self.client.is_armed():
            raise MavlinkError("Vehicle is armed - disarm before the camera tilt test")
        current = self.client.param_get("MNT1_TYPE")
        if current is None:
            raise MavlinkError("Could not read MNT1_TYPE")
        if int(current) == 1:
            # Stabilised servo mount: switch to BrushlessPWM (7), same servo
            # output without stabilisation. Needs a reboot either way.
            self._set(message="Disabling camera stabilization (MNT1_TYPE 1 → 7)")
            self.on_event("Camera tilt: disabling mount stabilization (MNT1_TYPE=7) and rebooting autopilot")
            self._set_param("MNT1_TYPE", 7, changes)
            self._reboot_and_wait(changes)
        pmin = float(mount.get("pitch_min") if mount.get("pitch_min") is not None else -45)
        pmax = float(mount.get("pitch_max") if mount.get("pitch_max") is not None else 45)

        def step(frac):
            angle = tilt_angle(frac, pmin, pmax)
            self._set(pwm=None, value_label=f"{angle:+.0f}°")
            self.client.mount_pitch(angle)

        return self._sweep_loop(step)

    def _reboot_and_wait(self, changes: list, check_abort: bool = True) -> None:
        self._set(phase="rebooting", message="Rebooting autopilot")
        result = self.client.reboot_autopilot()
        changes.append(f"Autopilot reboot requested ({result or 'no ack'})")
        wait_for_autopilot_reboot(self.client, lambda m: self._set(message=m),
                                  self._check if check_abort else None)
        self._set(phase="preparing", message="Autopilot back online")

    # -- motors ------------------------------------------------------------------------
    def _run_motor(self, test: dict) -> Optional[float]:
        firmware = self._vehicle.get("firmware")
        self._ensure_motor_session(firmware)
        idx = int(test["motor_index"])
        offset = int(self.settings["motor_offset"])
        direction = int(test.get("direction") or 1)
        pwm = NEUTRAL_PWM + offset * (1 if direction >= 0 else -1)

        def step(_frac):
            with self._ms_lock:
                self._ms_target = (idx, pwm)
            self._ms_last_use = time.monotonic()
            self._set(pwm=pwm, value_label=f"{pwm} µs")
            if not (self._ms_thread and self._ms_thread.is_alive()):
                raise MavlinkError("Motor test stream stopped (vehicle disarmed?)")

        try:
            return self._sweep_loop(step)
        finally:
            with self._ms_lock:
                self._ms_target = None

    def _ensure_motor_session(self, firmware: str) -> None:
        if self._ms_thread and self._ms_thread.is_alive():
            return
        if firmware == "ArduSub":
            wait = SUB_MOTOR_COOLDOWN_S - (time.monotonic() - self._sub_motor_end)
            while wait > 0:
                self._set(phase="cooldown", message=f"Autopilot motor-test cooldown ({wait:.0f} s)")
                self._sleep(min(wait, 0.5))
                wait = SUB_MOTOR_COOLDOWN_S - (time.monotonic() - self._sub_motor_end)
            if self.client.is_armed():
                raise MavlinkError("Vehicle is already armed by a pilot - disarm it before motor tests")
            self._set(phase="preparing", message="Switching to MANUAL and arming")
            self._ms_saved_mode = self.client.custom_mode()
            if self._ms_saved_mode != SUB_MANUAL_MODE:
                res = self.client.set_mode(SUB_MANUAL_MODE)
                if res not in (None, "MAV_RESULT_ACCEPTED"):
                    raise MavlinkError(f"Could not switch to MANUAL ({res})")
            res = self.client.arm(force=True)
            if res not in (None, "MAV_RESULT_ACCEPTED"):
                raise MavlinkError(f"Arming refused ({res})")
            deadline = time.monotonic() + 4
            while not self.client.is_armed():
                self._check()
                if time.monotonic() > deadline:
                    raise MavlinkError("Vehicle did not arm")
                time.sleep(0.1)
            self.on_event("Motor session: vehicle armed in MANUAL for motor tests")
        else:
            if self.client.is_armed():
                raise MavlinkError("Vehicle is already armed - disarm it before motor tests")
            self.on_event("Motor session started (ArduRover arms itself for motor test)")
        self._ms_stop.clear()
        self._ms_touched = set()
        self._ms_last_use = time.monotonic()
        self._ms_thread = threading.Thread(target=self._motor_stream, args=(firmware,),
                                           daemon=True, name="motor-stream")
        self._ms_thread.start()
        time.sleep(0.3)

    def _motor_stream(self, firmware: str) -> None:
        period = 1.0 / STREAM_HZ
        first_idx = None
        started = time.monotonic()
        tick = 0
        while not self._ms_stop.is_set():
            tick += 1
            t = time.monotonic()
            with self._ms_lock:
                target = self._ms_target
            frames = []
            for m in list(self._ms_touched):
                if not target or m != target[0]:
                    frames.append((m, NEUTRAL_PWM))
            if target:
                frames.append(target)
                self._ms_touched.add(target[0])
                first_idx = target[0] if first_idx is None else first_idx
            if not frames:
                # keep the autopilot's motor test alive with a neutral frame
                frames.append((first_idx if first_idx is not None else
                               (0 if firmware == "ArduSub" else 3), NEUTRAL_PWM))
            for m, pwm in frames:
                self.client.motor_test(m, pwm, timeout_s=1.0)
            # once a second, make sure the autopilot is still in the test
            # (ArduSub disarms itself if it missed frames for 500 ms)
            if tick % int(STREAM_HZ) == 0 and time.monotonic() - started > 2.0 \
                    and not self.client.is_armed():
                logger.warning("Vehicle disarmed during motor session")
                self.on_event("Vehicle disarmed unexpectedly during motor session")
                if firmware == "ArduSub":
                    self._sub_motor_end = time.monotonic()
                break
            time.sleep(max(0.0, period - (time.monotonic() - t)))
        # neutral everything we touched before letting go
        for _ in range(3):
            for m in list(self._ms_touched):
                self.client.motor_test(m, NEUTRAL_PWM, timeout_s=0.5)
            time.sleep(0.05)

    def end_motor_session(self, reason: str = "") -> None:
        thread = self._ms_thread
        if not (thread and thread.is_alive()):
            return
        self._ms_stop.set()
        thread.join(timeout=3)
        firmware = self._vehicle.get("firmware")
        try:
            self.client.disarm(force=True)
            if firmware == "ArduSub":
                self._sub_motor_end = time.monotonic()
                if self._ms_saved_mode is not None and self._ms_saved_mode != SUB_MANUAL_MODE:
                    time.sleep(0.5)
                    self.client.set_mode(self._ms_saved_mode)
        except Exception as e:  # noqa: BLE001
            logger.error("Ending motor session failed: %s", e)
        self.on_event(f"Motor session ended, vehicle disarmed ({reason})")

    # -- crash recovery -------------------------------------------------------------------
    def restore_all(self) -> dict:
        """Restore every journaled parameter (and reboot if MNT1_TYPE changed)."""
        pending = self.journal.pending()
        if not pending:
            return {"restored": [], "errors": []}
        restored, errors = [], []
        reboot = False
        for name in list(pending):
            try:
                self._restore_param(name, restored)
                reboot = reboot or name == "MNT1_TYPE"
            except Exception as e:  # noqa: BLE001
                errors.append(f"{name}: {e}")
        if reboot and not self.client.is_armed():
            self.client.reboot_autopilot()
            restored.append("Autopilot reboot requested (MNT1_TYPE restored)")
            try:
                wait_for_autopilot_reboot(self.client, lambda m: None, None)
            except Exception as e:  # noqa: BLE001
                errors.append(f"reboot: {e}")
        for line in restored:
            self.on_event(f"Restore: {line}")
        return {"restored": restored, "errors": errors}


def wait_for_autopilot_reboot(client: MavlinkClient, report: Callable[[str], None],
                              check: Optional[Callable[[], None]]) -> None:
    """Wait for the HEARTBEAT to go quiet, come back, and params to answer."""
    def _check():
        if check:
            check()

    start = time.monotonic()
    _, last_stamp = client.message("HEARTBEAT")
    last_change = time.monotonic()
    went_down = False
    report("Rebooting autopilot - waiting for it to go offline")
    while time.monotonic() - start < 15:
        _check()
        _, stamp = client.message("HEARTBEAT")
        if stamp != last_stamp:
            last_stamp, last_change = stamp, time.monotonic()
        elif time.monotonic() - last_change > 2.5:
            went_down = True
            break
        time.sleep(0.25)
    if not went_down:
        logger.warning("Heartbeat never stopped after reboot request; continuing")
    report("Waiting for autopilot to come back online")
    changes = 0
    while changes < 3:
        _check()
        if time.monotonic() - start > REBOOT_TIMEOUT_S:
            raise MavlinkError("Autopilot did not come back after reboot")
        _, stamp = client.message("HEARTBEAT")
        if stamp != last_stamp:
            last_stamp = stamp
            changes += 1
        time.sleep(0.3)
    report("Autopilot online - waiting for parameters")
    while time.monotonic() - start < REBOOT_TIMEOUT_S:
        _check()
        try:
            if client.param_get("SYSID_THISMAV", timeout_s=2.0) is not None:
                time.sleep(2.0)   # let the mount/servo libraries initialise
                return
        except MavlinkError:
            pass
        time.sleep(1.0)
    raise MavlinkError("Autopilot parameters unavailable after reboot")
