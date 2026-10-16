"""Unit tests against the simulated autopilot (run: pytest tests/)."""

import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import devices  # noqa: E402
from restore_state import RestoreJournal  # noqa: E402
from sim import SimClient  # noqa: E402
from tests_runner import Runner, centered_sweep, end_to_end, light_ramp, triangle  # noqa: E402
from vehicle import Detector, calibration, classify_output  # noqa: E402


def detect(profile):
    return Detector(SimClient(profile)).run()


def test_detect_bluerov2():
    v = detect("bluerov2")
    assert v["model"] == "BlueROV2"
    cats = [t["category"] for t in v["tests"]]
    assert cats.count("motor") == 6
    assert cats.count("lights") == 2
    assert "actuator" in cats and cats[-1] == "tilt"
    tilt = v["tests"][-1]
    assert tilt["channel"] == 16
    m2 = next(t for t in v["tests"] if t["name"] == "Motor 2")
    assert m2["motor_index"] == 1 and m2["direction"] == -1


def test_detect_heavy_and_boat():
    assert detect("heavy")["model"] == "BlueROV2 Heavy"
    assert sum(t["category"] == "motor" for t in detect("heavy")["tests"]) == 8
    boat = detect("blueboat")
    assert boat["model"] == "BlueBoat"
    motors = {t["function"]: t["motor_index"] for t in boat["tests"]}
    assert motors == {74: 4, 73: 3}


def test_legacy_lights_only_on_old_sub():
    assert classify_output(59, "ArduSub", (4, 1, 2))["category"] == "lights"
    assert classify_output(59, "ArduSub", (4, 5, 6)) is None
    assert classify_output(92, "ArduSub", (4, 5, 7))["category"] == "focus"
    assert classify_output(180, "ArduSub", (4, 5, 7))["category"] == "zoom"


def test_calibration_math_and_warnings():
    params = {"BATT_VOLT_MULT": 11.0, "BATT_AMP_PERVLT": 37.8788}
    out = calibration(16.0, 15.8, 1.3, 1.2, params, "bluerov2")
    assert abs(out["changes"]["BATT_VOLT_MULT"]["new"] - 16.0 * 11.0 / 15.8) < 1e-3
    assert abs(out["changes"]["BATT_AMP_PERVLT"]["new"] - 1.3 * 37.8788 / 1.2) < 1e-3
    assert not out["warnings"]
    out = calibration(18.0, 15.0, None, None, params, "bluerov2")
    assert any("away from" in w for w in out["warnings"])
    assert any("outside the expected" in w for w in out["warnings"])


def test_sweep_shapes():
    assert triangle(0) == 0 and triangle(0.5) == 1 and triangle(1) == 0
    assert centered_sweep(0, 1100, 1500, 1900) == 1500
    assert centered_sweep(0.25, 1100, 1500, 1900) == 1900
    assert centered_sweep(0.75, 1100, 1500, 1900) == 1100
    assert light_ramp(0.05) == 0 and light_ramp(0.5) == 1 and light_ramp(0.97) == 0
    assert end_to_end(0.2) == 1 and end_to_end(0.8) == 0


def make_runner(tmp_path, profile="bluerov2"):
    client = SimClient(profile)
    vehicle = Detector(client).run()
    journal = RestoreJournal(str(tmp_path / ".restore.json"))
    results = []
    runner = Runner(client, journal, on_result=results.append, on_event=lambda t: None)
    runner.settings["duration_s"] = 1.0
    runner.settings["motor_duration_s"] = 1.0
    runner.settings["actuator_duration_s"] = 1.0
    runner.set_vehicle(vehicle)
    return client, vehicle, journal, runner, results


def wait_done(runner, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        runner.keepalive()
        if not runner.busy():
            return
        time.sleep(0.05)
    raise AssertionError("test did not finish")


def test_light_sweep_restores_function(tmp_path):
    client, vehicle, journal, runner, results = make_runner(tmp_path)
    light = next(t for t in vehicle["tests"] if t["category"] == "lights")
    assert runner.start(light) is None
    wait_done(runner)
    assert results[-1]["result"] == "PASS"
    assert client.params[f"SERVO{light['channel']}_FUNCTION"] == 181
    assert journal.pending() == {}


def test_watchdog_abort_restores(tmp_path):
    client, vehicle, journal, runner, results = make_runner(tmp_path)
    runner.settings["actuator_duration_s"] = 10.0
    act = next(t for t in vehicle["tests"] if t["category"] == "actuator")
    runner.start(act)
    time.sleep(3.0)          # no keepalives -> watchdog fires
    wait_done(runner)
    assert results[-1]["result"] == "ABORTED"
    assert client.params["SERVO9_FUNCTION"] == 184
    assert journal.pending() == {}


def test_motor_session_disarms(tmp_path):
    client, vehicle, journal, runner, results = make_runner(tmp_path)
    motor = vehicle["tests"][0]
    runner.start(motor)
    wait_done(runner)
    assert results[-1]["result"] == "PASS"
    assert client.armed and client.mode == 19
    runner.end_motor_session("test")
    assert not client.armed and client.mode == 0


def test_journal_replay(tmp_path):
    path = str(tmp_path / ".restore.json")
    j = RestoreJournal(path)
    j.record("SERVO13_FUNCTION", 181)
    j.record("SERVO13_FUNCTION", 0)       # first recording wins
    assert RestoreJournal(path).pending() == {"SERVO13_FUNCTION": 181}


def test_ping_protocol_framing():
    # Example from the ping-protocol documentation
    assert devices.general_request(5).hex() == "425202000600000005" + "00a100"
    frame = devices.ping_msg(1211, struct.pack("<IB", 1234, 99))
    assert devices.ping_parse_all(b"junk" + frame) == [(1211, struct.pack("<IB", 1234, 99))]


def test_version_checks():
    import versions
    versions._cache.clear()
    sim = SimClient("bluerov2")
    out = versions.check_versions(sim, {"firmware": "ArduSub", "firmware_version": "4.5.7"})
    assert out["result"] == "WARN"
    assert out["checks"][0]["latest"] == "4.7.2" and out["checks"][0]["update_page"]
    assert out["checks"][1]["result"] == "PASS"          # sim BlueOS 1.4.6
    versions._cache.clear()
    assert versions.check_versions(sim, {"firmware": "ArduSub", "firmware_version": "4.7.2"})["result"] == "PASS"

    class Offline:
        def blueos_get(self, *a, **k):
            return None
    versions._cache.clear()
    out = versions.check_versions(Offline(), {"firmware": "ArduRover", "firmware_version": "4.7.2"})
    assert out["checks"][0]["result"] == "PASS" and "built-in" in out["checks"][0]["detail"]
    assert out["checks"][1]["result"] == "WARN"           # BlueOS version unknown
    assert versions.parse_version("1.5.0-beta.2") == (1, 5, 0)


def test_4k_cam_param_check(monkeypatch):
    sim = SimClient("bluerov2")
    vehicle = Detector(sim).run()
    monkeypatch.setattr(devices, "find_4k_cam_manager", lambda: None)
    out = devices.check_4k_cam_params(sim, vehicle)
    rows = {r["category"]: r for r in out["rows"]}
    assert set(rows) == {"focus", "zoom", "tilt"}
    # focus/zoom left at ArduPilot defaults are flagged as not saved
    assert rows["focus"]["result"] == "FAIL" and "defaults" in rows["focus"]["issues"][0]
    assert rows["tilt"]["result"] == "PASS"
    sim.params.update({"SERVO11_MIN": 870, "SERVO11_TRIM": 1639, "SERVO11_MAX": 2130,
                       "SERVO12_MIN": 1000, "SERVO12_TRIM": 1500, "SERVO12_MAX": 2000})
    out = devices.check_4k_cam_params(sim, vehicle)
    assert out["result"] == "WARN"          # values fine, but no 4K Cam Manager to compare with


def test_camera_param_check_tilt_only(monkeypatch):
    sim = SimClient("bluerov2")
    vehicle = {"tests": [{"name": "Camera Tilt", "category": "tilt", "channel": 16, "function": 7}]}
    monkeypatch.setattr(devices, "find_4k_cam_manager", lambda: None)
    out = devices.check_4k_cam_params(sim, vehicle)
    assert out["result"] == "PASS" and out["rows"][0]["max"] == 1900
    assert "no 4K camera" in out["source"]
