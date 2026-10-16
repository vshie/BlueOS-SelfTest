# BlueOS Self Test

A BlueOS extension that walks a user, or a production-line tester, through a
non-destructive hardware check of a **BlueROV2**, **BlueROV2 Heavy** or
**BlueBoat**. Every run writes a log file to the vehicle. At the end of the
wizard the log folder opens in the BlueOS File Browser.

## Wizard steps

1. **Vehicle detection.** The extension reads the HEARTBEAT, `FRAME_CONFIG` / `FRAME_CLASS`
   and every `SERVOn_FUNCTION`, and maps each output the same way BlueOS
   *Vehicle Setup → PWM Outputs* does:
   - Motors: functions 33–40 (Sub), or ThrottleLeft/Right 73/74 (BlueBoat).
   - Lights: Lights1/2 181/182. On ArduSub ≤ 4.5.5, RCIN9/RCIN10 also count as lights.
   - Gripper: 28, or Actuator1–6 184–189.
   - Camera tilt (MountTilt): 7.
   - Camera focus: 92.
   - Camera zoom: 180.
2. **Network test.** Measures latency, download and upload between *this browser* and the
   vehicle, using BlueOS's built-in network-test service (`/network-test/*`). These are the
   same endpoints the BlueOS speed-test widget uses. BlueOS's iperf3 server (TCP 5201) is raw
   TCP, which a browser can't reach, so it isn't used here.
3. **Network devices & sonars.**
   - Scans the vehicle subnet (192.168.2.0/24 by default) with an ICMP sweep, the host ARP
     table and a few TCP ports. It identifies a Water Linked DVL, Sonoptix Echo, Cerulean
     sonars and IP cameras.
   - Discovers Ethernet Ping360s with the UDP 30303 "Discovery" broadcast, checks that each
     answers `device_information`, and checks that it returns sonar samples.
   - Lists USB/serial Ping devices through the BlueOS Ping service and talks to them through
     its UDP bridges (Ping1D distance, Ping360 samples).
   - For a Ping1D, checks that DISTANCE_SENSOR forwarding is enabled, that
     `RNGFND1_TYPE=10` / `RNGFND1_ORIENT=25`, and that the autopilot reports live
     rangefinder data.
4. **Power sensor calibration.** Enter the meter-measured voltage and current. The new
   `BATT_VOLT_MULT` / `BATT_AMP_PERVLT` values are computed exactly as BlueOS's
   *Power* page does. Before you can apply, the page warns if a value would be more than
   10 % from the vehicle's default (BlueOS Parameter Repository) or if the pack voltage is
   outside 12.0–16.8 V.
5. **Hardware tests.** Each output is exercised for 5 seconds while you watch.
   - **Press SPACE** (or the red button) if the expected behaviour is *not* happening. This
     records a FAIL immediately, stops the output, and offers Retry / Continue plus a
     troubleshooting-guide link (`TROUBLESHOOTING_URLS` in `app/main.py`).
   - **Motors** run one at a time at ±100 µs from neutral, using `MOT_n_DIRECTION`.
     - ArduSub only accepts motor tests while armed. The extension switches to MANUAL,
       arms, and streams `MAV_CMD_DO_MOTOR_TEST` at 10 Hz, as the BlueOS PWM page does.
       It disarms and restores the previous mode when the motor tests end.
     - On a BlueBoat, a stern-view graphic shows the correct rotation: blades moving
       **up in the middle**.
   - **Lights, gripper/actuators, focus and zoom.** The output's `SERVOn_FUNCTION` is set to 0
     for the test, because `DO_SET_SERVO` is rejected on assigned outputs. The output is swept
     MIN→MAX→MIN, parked, and the original function is restored.
   - **Camera tilt.** If the mount is stabilised (`MNT1_TYPE=1`), it is switched to
     `MNT1_TYPE=7` (same servo, no stabilisation) and the autopilot is rebooted. The tilt is
     then swept through `MNT1_PITCH_MIN…MAX`, `MNT1_TYPE=1` is restored, and the autopilot is
     rebooted again. If the mount was not stabilised, there are no reboots.
6. **Finish.** Shows a summary, the log file names, and an embedded BlueOS File Browser at
   `extensions/selftest`.

## Safety and non-destructive design

- The user must confirm the vehicle is secured before anything moves.
- The backend drives every test. The browser sends keepalives several times a second; if
  they stop for 1.5 s (tab closed, network lost), the test is aborted, outputs return to
  neutral, the vehicle is disarmed, and parameters are restored.
- Before any parameter changes, its original value is written to a restore journal
  (`/usr/blueos/extensions/selftest/.restore.json`). The journal is replayed automatically
  when the extension starts, so a power loss mid-test can't leave a vehicle with
  test-only settings. A "Restore now" banner appears if anything is pending.
- Motor tests refuse to start if a pilot has already armed the vehicle.

## Logs

Each run creates `selftest_YYYYMMDD_HHMMSS_<vehicle>.log` (human-readable) and a matching
`.json` file in `/usr/blueos/extensions/selftest`. The log records:
- Vehicle, firmware, BlueOS version, operator and serial number.
- Network results.
- Device scan results.
- Calibration values before and after.
- Every test result, with its fail time and each parameter changed and restored.

## Installing

In BlueOS go to **Extensions → Installed → +** (manual install):

| Field | Value |
|---|---|
| Extension Identifier | `vshie.selftest` |
| Extension Name | `Self Test` |
| Docker image | `<dockerhub-user>/blueos-selftest` |
| Docker tag | `main` (or a release tag) |

Paste this into **Custom settings** (it's the same as the Dockerfile `permissions` label):

```json
{
  "ExposedPorts": {
    "9450/tcp": {}
  },
  "HostConfig": {
    "CpuPeriod": 100000,
    "CpuQuota": 100000,
    "Binds": [
      "/usr/blueos/extensions/selftest:/app/logs"
    ],
    "ExtraHosts": ["host.docker.internal:host-gateway"],
    "NetworkMode": "host"
  }
}
```

The UI runs on port **9450**, and in the BlueOS sidebar through `works_in_relative_paths`.

## Development

```sh
./scripts/fetch_vendor.sh                 # Vue/Vuetify/MDI/Roboto into app/static/vendor (git-ignored)
python -m venv .venv && . .venv/bin/activate && pip install flask requests waitress pytest
cd app && SELFTEST_SIM=bluerov2 SELFTEST_LOG_DIR=/tmp/selftest python main.py   # or heavy / blueboat
# open http://localhost:9450
pytest tests/
python app/devices.py                      # run the device/sonar scan on its own (on a vehicle)
```

`SELFTEST_SIM` replaces the autopilot with a simulator built on the BlueOS Parameter
Repository defaults. It also provides stand-ins for the network-test download/upload
endpoints.

## CI

`.github/workflows/deploy.yml` uses
[BlueOS-community/Deploy-BlueOS-Extension](https://github.com/BlueOS-community/Deploy-BlueOS-Extension)
to build `linux/arm/v7` and `linux/arm64/v8` images and push them to Docker Hub.

To set it up, add these repository **secrets**:
- `DOCKER_USERNAME`
- `DOCKER_PASSWORD`

Optionally add these **variables**:
- `IMAGE_NAME` (default `selftest`; the action prefixes `blueos-`)
- `MY_NAME`, `MY_EMAIL`, `ORG_NAME`, `ORG_EMAIL`

Keep `EXTENSION_VERSION` in `app/main.py` in sync with the Dockerfile `version` label.
