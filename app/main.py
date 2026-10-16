"""
BlueOS Self Test extension - web server.

Serves the Vue wizard and a small JSON API that detects the vehicle,
runs hardware tests, calibrates the power sensor and writes per-run logs.
"""

from __future__ import annotations

import logging
import os
import threading
import time

from flask import Flask, jsonify, request, send_from_directory, Response

import devices
import versions
from logger import RunLog, list_logs, read_log
from mavlink import MavlinkClient, MavlinkError
from restore_state import RestoreJournal
from tests_runner import Runner
from vehicle import Detector, calibration

EXTENSION_VERSION = "1.3.0"     # keep in sync with the Dockerfile `version` label
PORT = int(os.environ.get("SELFTEST_PORT", "9450"))
LOG_DIR = os.environ.get("SELFTEST_LOG_DIR", "/app/logs")
SIM = os.environ.get("SELFTEST_SIM", "").strip().lower()

TROUBLESHOOTING_URLS = {
    # Filled in as the per-hardware troubleshooting guides are published.
    "network": "",
    "power": "",
    "motor": "",
    "lights": "",
    "actuator": "",
    "focus": "",
    "zoom": "",
    "tilt": "",
}

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("selftest")

os.makedirs(LOG_DIR, exist_ok=True)

if SIM:
    from sim import SimClient
    client = SimClient(SIM)
else:
    client = MavlinkClient()

journal = RestoreJournal(os.path.join(LOG_DIR, ".restore.json"))

state_lock = threading.Lock()
app_state = {
    "connected": False,
    "detect": {"status": "idle", "message": "", "progress": 0.0, "error": None},
    "vehicle": None,
    "run": None,          # RunLog
    "results": {},        # test_id -> latest result
    "versions": None,     # firmware / BlueOS version check
}


def current_log():
    return app_state["run"]


def log_event(text: str) -> None:
    run = current_log()
    if run:
        run.event(text)
    logger.info(text)


def record_result(result: dict) -> None:
    app_state["results"][result["id"]] = result
    run = current_log()
    if run:
        run.test(result)


runner = Runner(client, journal, on_result=record_result, on_event=log_event)

app = Flask(__name__, static_folder="static", static_url_path="/static")


# ---------------------------------------------------------------------------
# background: connect + crash recovery
# ---------------------------------------------------------------------------
def connection_loop() -> None:
    while True:
        ok = client.connected and client.heartbeat() is not None
        if not ok:
            ok = client.connect() and client.heartbeat() is not None
        if ok and not app_state["connected"]:
            logger.info("Autopilot connected")
        app_state["connected"] = ok
        if ok and journal.pending() and not runner.busy():
            logger.warning("Restoring parameters left from an interrupted test: %s", journal.pending())
            try:
                runner.restore_all()
            except Exception as e:  # noqa: BLE001
                logger.error("Startup restore failed: %s", e)
        time.sleep(2.0)


def run_detection() -> None:
    def progress(msg, frac):
        app_state["detect"].update(message=msg, progress=round(frac, 3))

    app_state["detect"] = {"status": "running", "message": "Starting", "progress": 0.0, "error": None}
    try:
        if not client.connected and not client.connect():
            raise RuntimeError("Cannot reach mavlink2rest / autopilot")
        vehicle = Detector(client, progress).run()
        app_state["vehicle"] = vehicle
        runner.set_vehicle(vehicle)
        progress("Checking firmware and BlueOS versions", 0.99)
        try:
            app_state["versions"] = versions.check_versions(client, vehicle)
        except Exception as e:  # noqa: BLE001
            logger.error("Version check failed: %s", e)
            app_state["versions"] = None
        app_state["detect"].update(status="done", progress=1.0, message="Done")
    except Exception as e:  # noqa: BLE001
        logger.error("Detection failed: %s", e)
        app_state["detect"].update(status="error", error=str(e))


def blueos_meta() -> dict:
    meta = {"extension_version": EXTENSION_VERSION}
    try:
        name = client.blueos_get("/beacon/v1.0/vehicle_name")
        if name:
            meta["vehicle_name"] = name if isinstance(name, str) else str(name)
        ver = client.blueos_get("/version-chooser/v1.0/version/current")
        if isinstance(ver, dict):
            meta["blueos_version"] = ver.get("tag") or ver.get("version")
    except Exception:  # noqa: BLE001
        pass
    return meta


# ---------------------------------------------------------------------------
# BlueOS integration
# ---------------------------------------------------------------------------
@app.route("/register_service")
def register_service():
    return jsonify({
        "name": "Self Test",
        "description": "Guided hardware self-test for BlueROV2 and BlueBoat: network, power "
                       "sensor calibration, thrusters, lights, gripper and camera.",
        "icon": "mdi-clipboard-check-outline",
        "company": "Blue Robotics",
        "version": EXTENSION_VERSION,
        "webpage": "https://github.com/vshie/BlueOS-SelfTest",
        "api": "https://github.com/vshie/BlueOS-SelfTest",
        "works_in_relative_paths": True,
    })


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


# ---------------------------------------------------------------------------
# API
# ---------------------------------------------------------------------------
@app.route("/api/status")
def api_status():
    run = current_log()
    return jsonify({
        "connected": app_state["connected"],
        "sim": SIM or None,
        "version": EXTENSION_VERSION,
        "detect": app_state["detect"],
        "vehicle": app_state["vehicle"],
        "run": {"files": run.files, "basename": run.basename} if run else None,
        "results": app_state["results"],
        "versions": app_state["versions"],
        "restore_pending": journal.pending(),
        "troubleshooting": TROUBLESHOOTING_URLS,
        "settings": runner.settings,
    })


@app.route("/api/detect", methods=["POST"])
def api_detect():
    if app_state["detect"]["status"] == "running":
        return jsonify({"ok": True})
    if runner.busy():
        return jsonify({"ok": False, "error": "A test is running"}), 409
    threading.Thread(target=run_detection, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/telemetry")
def api_telemetry():
    runner.keepalive()
    sys_status, _ = client.message("SYS_STATUS")
    servo, _ = client.message("SERVO_OUTPUT_RAW")
    hb = client.heartbeat() or {}
    volts = amps = None
    if sys_status:
        if sys_status.get("voltage_battery") not in (None, 65535):
            volts = round(sys_status["voltage_battery"] / 1000.0, 3)
        if sys_status.get("current_battery") not in (None, -1):
            amps = round(sys_status["current_battery"] / 100.0, 2)
    base_mode = hb.get("base_mode")
    armed = bool((base_mode or {}).get("bits", 0) & 128) if isinstance(base_mode, dict) else False
    return jsonify({
        "voltage": volts, "current": amps, "armed": armed,
        "mode": hb.get("custom_mode"),
        "servo": [servo.get(f"servo{i}_raw") for i in range(1, 17)] if servo else None,
    })


@app.route("/api/session/start", methods=["POST"])
def api_session_start():
    vehicle = app_state["vehicle"]
    if not vehicle:
        return jsonify({"ok": False, "error": "Run detection first"}), 400
    body = request.get_json(silent=True) or {}
    meta = blueos_meta()
    for key in ("operator", "serial", "notes"):
        if body.get(key):
            meta[key] = str(body[key])[:200]
    app_state["results"] = {}
    app_state["run"] = RunLog(LOG_DIR, vehicle, meta)
    app_state["run"].event("Session started")
    if app_state["versions"]:
        app_state["run"].versions(app_state["versions"])
        app_state["results"]["versions"] = {"id": "versions", "result": app_state["versions"]["result"],
                                            "summary": "; ".join(c["detail"] for c in app_state["versions"]["checks"])}
    return jsonify({"ok": True, "files": app_state["run"].files})


@app.route("/api/network", methods=["POST"])
def api_network():
    run = current_log()
    body = request.get_json(silent=True) or {}
    body["client"] = request.headers.get("User-Agent", "")[:160]
    if run:
        run.network(body)
    app_state["results"]["network"] = {"id": "network", "result": body.get("verdict"), **body}
    return jsonify({"ok": True})


@app.route("/api/devices/scan", methods=["POST"])
def api_devices_scan():
    try:
        result = devices.sim_scan() if SIM else devices.run_scan(client)
    except Exception as e:  # noqa: BLE001
        logger.exception("Device scan failed")
        return jsonify({"error": str(e)}), 500
    run = current_log()
    if run:
        run.devices(result)
    app_state["results"]["devices"] = {"id": "devices", "result": result["result"],
                                       "summary": result["summary"]}
    return jsonify(result)


def _reported_power():
    msg, _ = client.message("SYS_STATUS")
    if not msg:
        return None, None
    v = msg.get("voltage_battery")
    a = msg.get("current_battery")
    return (None if v in (None, 65535) else v / 1000.0,
            None if a in (None, -1) else a / 100.0)


def _float_or_none(value):
    try:
        return None if value in (None, "") else float(value)
    except (TypeError, ValueError):
        return None


@app.route("/api/calibration/preview", methods=["POST"])
def api_calibration_preview():
    body = request.get_json(silent=True) or {}
    vehicle = app_state["vehicle"] or {}
    params = {n: client.param_get(n) for n in ("BATT_VOLT_MULT", "BATT_AMP_PERVLT", "BATT_MONITOR")}
    reported_v, reported_a = _reported_power()
    out = calibration(_float_or_none(body.get("measured_v")), reported_v,
                      _float_or_none(body.get("measured_a")), reported_a,
                      params, vehicle.get("model_key"))
    if not params.get("BATT_MONITOR"):
        out["warnings"].insert(0, "BATT_MONITOR is 0 (battery monitoring disabled).")
    out.update(reported_v=reported_v, reported_a=reported_a, params=params)
    return jsonify(out)


@app.route("/api/calibration/apply", methods=["POST"])
def api_calibration_apply():
    body = request.get_json(silent=True) or {}
    vehicle = app_state["vehicle"] or {}
    measured_v = _float_or_none(body.get("measured_v"))
    measured_a = _float_or_none(body.get("measured_a"))
    params = {n: client.param_get(n) for n in ("BATT_VOLT_MULT", "BATT_AMP_PERVLT")}
    reported_v, reported_a = _reported_power()
    out = calibration(measured_v, reported_v, measured_a, reported_a, params, vehicle.get("model_key"))
    wanted = set(body.get("apply") or out["changes"].keys())
    applied = {}
    errors = []
    for name, change in out["changes"].items():
        if name not in wanted:
            continue
        try:
            change["readback"] = client.param_set(name, change["new"])
            applied[name] = change
        except MavlinkError as e:
            errors.append(f"{name}: {e}")
    entry = {"measured_v": measured_v, "measured_a": measured_a,
             "reported_v": reported_v, "reported_a": reported_a,
             "changes": out["changes"], "warnings": out["warnings"],
             "applied": bool(applied), "errors": errors,
             "warnings_acknowledged": bool(body.get("confirmed"))}
    run = current_log()
    if run:
        run.calibration(entry)
    app_state["results"]["power"] = {"id": "power", "result": "ERROR" if errors else "DONE", **entry}
    return jsonify({"ok": not errors, **entry})


@app.route("/api/calibration/skip", methods=["POST"])
def api_calibration_skip():
    reported_v, reported_a = _reported_power()
    entry = {"skipped": True, "reported_v": reported_v, "reported_a": reported_a}
    run = current_log()
    if run:
        run.calibration(entry)
    app_state["results"]["power"] = {"id": "power", "result": "SKIPPED", **entry}
    return jsonify({"ok": True})


def _find_test(test_id):
    for t in (app_state["vehicle"] or {}).get("tests", []):
        if t["id"] == test_id:
            return t
    return None


@app.route("/api/test/start", methods=["POST"])
def api_test_start():
    body = request.get_json(silent=True) or {}
    test = _find_test(body.get("id"))
    if not test:
        return jsonify({"ok": False, "error": "Unknown test"}), 404
    err = runner.start(test)
    if err:
        return jsonify({"ok": False, "error": err}), 409
    return jsonify({"ok": True})


@app.route("/api/test/state")
def api_test_state():
    return jsonify(runner.state(keepalive=True))


@app.route("/api/test/fail", methods=["POST"])
def api_test_fail():
    runner.flag_fail()
    return jsonify({"ok": True})


@app.route("/api/test/abort", methods=["POST"])
def api_test_abort():
    runner.abort("Aborted by user")
    return jsonify({"ok": True})


@app.route("/api/test/skip", methods=["POST"])
def api_test_skip():
    body = request.get_json(silent=True) or {}
    test = _find_test(body.get("id"))
    if not test:
        return jsonify({"ok": False, "error": "Unknown test"}), 404
    record_result({"id": test["id"], "name": test["name"], "channel": test["channel"],
                   "category": test["category"], "function": test["function"],
                   "result": "SKIPPED", "fail_at_s": None, "error": None,
                   "param_changes": [], "time": time.strftime("%H:%M:%S")})
    return jsonify({"ok": True})


@app.route("/api/motors/end", methods=["POST"])
def api_motors_end():
    runner.end_motor_session("Motor tests complete")
    return jsonify({"ok": True})


@app.route("/api/motors/review", methods=["POST"])
def api_motors_review():
    """End-of-sequence thruster review: the tester gets a second chance to
    report thrusters that did not work, without re-running them."""
    body = request.get_json(silent=True) or {}
    reported = set(body.get("failed_ids") or [])
    motors = [t for t in (app_state["vehicle"] or {}).get("tests", []) if t["category"] == "motor"]
    changed = []
    for t in motors:
        prev = app_state["results"].get(t["id"])
        if t["id"] in reported and not (prev and prev["result"] == "FAIL"):
            record_result({"id": t["id"], "name": t["name"], "channel": t["channel"],
                           "category": "motor", "function": t["function"], "result": "FAIL",
                           "fail_at_s": None, "error": "Reported not working at end-of-sequence review",
                           "param_changes": [], "time": time.strftime("%H:%M:%S")})
            changed.append(t["name"])
    failed = [t["name"] for t in motors
              if (app_state["results"].get(t["id"]) or {}).get("result") == "FAIL"]
    log_event("Thruster review: " + (f"not working: {', '.join(failed)}" if failed
                                     else "all thrusters confirmed working"))
    return jsonify({"ok": True, "failed": failed, "newly_reported": changed})


@app.route("/api/settings", methods=["POST"])
def api_settings():
    body = request.get_json(silent=True) or {}
    if "motor_offset" in body:
        runner.settings["motor_offset"] = max(25, min(300, int(body["motor_offset"])))
    if "motor_duration_s" in body:
        runner.settings["motor_duration_s"] = max(1.0, min(20.0, float(body["motor_duration_s"])))
    if "duration_s" in body:
        runner.settings["duration_s"] = max(2.0, min(20.0, float(body["duration_s"])))
    return jsonify({"ok": True, "settings": runner.settings})


@app.route("/api/restore", methods=["POST"])
def api_restore():
    if runner.busy():
        return jsonify({"ok": False, "error": "A test is running"}), 409
    return jsonify({"ok": True, **runner.restore_all()})


@app.route("/api/session/finish", methods=["POST"])
def api_session_finish():
    runner.end_motor_session("Wizard finished")
    run = current_log()
    if not run:
        return jsonify({"ok": False, "error": "No session"}), 400
    tests = [r for k, r in app_state["results"].items() if k not in ("network", "power", "devices", "versions")]
    summary = {
        "passed": sum(r["result"] == "PASS" for r in tests),
        "failed": sum(r["result"] == "FAIL" for r in tests),
        "skipped": sum(r["result"] == "SKIPPED" for r in tests),
        "errors": sum(r["result"] in ("ERROR", "ABORTED") for r in tests),
    }
    summary["overall"] = "PASS" if summary["failed"] == 0 and summary["errors"] == 0 \
        and summary["passed"] > 0 else "FAIL"
    if journal.pending():
        summary["overall"] = "FAIL"
        run.event(f"WARNING: parameters still pending restore: {journal.pending()}")
    run.finish(summary)
    return jsonify({"ok": True, "summary": summary, "files": run.files})


@app.route("/api/logs")
def api_logs():
    return jsonify(list_logs(LOG_DIR))


@app.route("/api/logs/<name>")
def api_log(name):
    text = read_log(LOG_DIR, name)
    if text is None:
        return jsonify({"error": "not found"}), 404
    mimetype = "application/json" if name.endswith(".json") else "text/plain"
    return Response(text, mimetype=mimetype,
                    headers={"Content-Disposition": f"inline; filename={name}"})


if SIM:
    # Stand-ins for BlueOS's network-test service so the wizard can be
    # exercised off-vehicle (the WebSocket latency probe is not simulated).
    @app.route("/network-test/get_file")
    def sim_get_file():
        size = int(request.args.get("size", 100 * 2 ** 20))
        chunk = os.urandom(65536)

        def gen():
            sent = 0
            while sent < size:
                yield chunk
                sent += len(chunk)
        return Response(gen(), mimetype="application/octet-stream")

    @app.route("/network-test/post_file", methods=["POST"])
    def sim_post_file():
        while request.stream.read(1 << 20):
            pass
        return jsonify({"ok": True})


if __name__ == "__main__":
    threading.Thread(target=connection_loop, daemon=True).start()
    from waitress import serve
    logger.info("Self Test %s on port %s%s", EXTENSION_VERSION, PORT, f" (SIM={SIM})" if SIM else "")
    serve(app, host="0.0.0.0", port=PORT, threads=12)
