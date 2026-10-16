"""Unit tests against the simulated autopilot (run: pytest tests/)."""

import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "app"))

import devices  # noqa: E402
from restore_state import RestoreJournal  # noqa: E402
from sim import SimClient  # noqa: E402
from tests_runner import Runner, tilt_angle, triangle  # noqa: E402
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
    assert tilt["method"] == "mount" and tilt["channel"] == 16
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
    assert tilt_angle(0.25, -30, 45) == 45
    assert abs(tilt_angle(0.75, -30, 45) + 30) < 1e-9


def make_runner(tmp_path, profile="bluerov2"):
    client = SimClient(profile)
    vehicle = Detector(client).run()
    journal = RestoreJournal(str(tmp_path / ".restore.json"))
    results = []
    runner = Runner(client, journal, on_result=results.append, on_event=lambda t: None)
    runner.settings["duration_s"] = 1.0
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
    runner.settings["duration_s"] = 10.0
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
