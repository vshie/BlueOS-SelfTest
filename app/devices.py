"""
Network device scan and sonar verification.

* Scans the vehicle's Ethernet subnet(s) (192.168.2.0/24 by default):
  ICMP ping sweep + host ARP table + a few TCP ports, and identifies known
  devices (Water Linked DVL, Sonoptix Echo, Cerulean sonars, cameras).
* Discovers Ethernet Ping360s with the ASCII "Discovery" broadcast on UDP
  30303 (same as Ping Viewer / BlueOS ping service) and checks that each
  one answers device_information and returns sonar samples.
* Reads the BlueOS Ping service (/ping/v1.0/sensors) for USB/serial Ping
  devices, talks to them through the service's UDP bridges (Ping1D from
  9090 down, Ping360 from 9092 up), and for a Ping1D checks that the
  autopilot is consuming its DISTANCE_SENSOR feed (RNGFND1_TYPE=10).

Everything here is read-only for the vehicle: no parameters are changed.
"""

from __future__ import annotations

import concurrent.futures
import ipaddress
import json
import logging
import socket
import struct
import subprocess
import time
from typing import Optional

import requests

from mavlink import MavlinkClient

logger = logging.getLogger(__name__)

BLUEOS = "http://127.0.0.1"
DEFAULT_SUBNET = "192.168.2.0/24"
TCP_PORTS = (80, 554, 8080, 8081, 16171, 51200)

# Ping protocol message ids (bluerobotics/ping-protocol)
MSG_NACK = 2
MSG_DEVICE_INFORMATION = 4
MSG_GENERAL_REQUEST = 6
MSG_PING1D_FIRMWARE_VERSION = 1200
MSG_PING1D_DISTANCE_SIMPLE = 1211
MSG_PING360_DEVICE_DATA = 2300
MSG_PING360_TRANSDUCER = 2601
DEVICE_TYPES = {0: "Unknown", 1: "Ping Echosounder", 2: "Ping360"}


# ---------------------------------------------------------------------------
# Ping protocol
# ---------------------------------------------------------------------------
def ping_msg(msg_id: int, payload: bytes = b"", src: int = 0, dst: int = 0) -> bytes:
    body = b"BR" + struct.pack("<HHBB", len(payload), msg_id, src, dst) + payload
    return body + struct.pack("<H", sum(body) & 0xFFFF)


def ping_parse_all(buf: bytes) -> list:
    """Return every valid ``(msg_id, payload)`` frame found in ``buf``."""
    out = []
    i = 0
    while True:
        i = buf.find(b"BR", i)
        if i < 0 or len(buf) < i + 10:
            return out
        plen, mid, _src, _dst = struct.unpack_from("<HHBB", buf, i + 2)
        end = i + 8 + plen
        if len(buf) < end + 2:
            return out
        if sum(buf[i:end]) & 0xFFFF == struct.unpack_from("<H", buf, end)[0]:
            out.append((mid, buf[i + 8:end]))
            i = end + 2
        else:
            i += 2


def general_request(msg_id: int) -> bytes:
    return ping_msg(MSG_GENERAL_REQUEST, struct.pack("<H", msg_id))


def ping360_transducer(angle: int = 0, n_samples: int = 1200) -> bytes:
    # mode, gain, angle(gradians), tx_duration(us), sample_period(25 ns ticks),
    # tx_frequency(kHz), n_samples, transmit, reserved
    return ping_msg(MSG_PING360_TRANSDUCER,
                    struct.pack("<BBHHHHHBB", 1, 0, angle, 32, 80, 750, n_samples, 1, 0))


def udp_exchange(host: str, port: int, request: bytes, want: int,
                 timeout: float = 1.5, retries: int = 2) -> Optional[bytes]:
    """Send ``request`` and return the payload of the first ``want`` reply."""
    for _ in range(retries):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.settimeout(0.3)
            s.sendto(request, (host, port))
            deadline = time.monotonic() + timeout
            buf = b""
            while time.monotonic() < deadline:
                try:
                    data, _ = s.recvfrom(65535)
                except socket.timeout:
                    continue
                buf += data
                for mid, payload in ping_parse_all(buf):
                    if mid == want:
                        return payload
                buf = buf[-70000:]
        except OSError as e:
            logger.debug("UDP %s:%s failed: %s", host, port, e)
        finally:
            s.close()
    return None


def device_information(host: str, port: int) -> Optional[dict]:
    payload = udp_exchange(host, port, general_request(MSG_DEVICE_INFORMATION), MSG_DEVICE_INFORMATION)
    if payload and len(payload) >= 6:
        t, rev, ma, mi, pa, _ = struct.unpack("<6B", payload[:6])
        return {"device_type": DEVICE_TYPES.get(t, str(t)), "revision": rev, "firmware": f"{ma}.{mi}.{pa}"}
    # very old Ping1D firmware has no device_information
    payload = udp_exchange(host, port, general_request(MSG_PING1D_FIRMWARE_VERSION), MSG_PING1D_FIRMWARE_VERSION)
    if payload and len(payload) >= 6:
        _t, _m, major, minor = struct.unpack("<BBHH", payload[:6])
        return {"device_type": "Ping Echosounder", "revision": None, "firmware": f"{major}.{minor}"}
    return None


def ping1d_distance(host: str, port: int) -> Optional[dict]:
    payload = udp_exchange(host, port, general_request(MSG_PING1D_DISTANCE_SIMPLE), MSG_PING1D_DISTANCE_SIMPLE)
    if payload and len(payload) >= 5:
        dist, conf = struct.unpack("<IB", payload[:5])
        return {"distance_m": round(dist / 1000.0, 3), "confidence": conf}
    return None


def ping360_samples(host: str, port: int) -> Optional[dict]:
    payload = udp_exchange(host, port, ping360_transducer(), MSG_PING360_DEVICE_DATA, timeout=3.0)
    if payload and len(payload) >= 16:
        n_samples = struct.unpack_from("<H", payload, 12)[0]
        data_len = struct.unpack_from("<H", payload, 14)[0]
        data = payload[16:16 + data_len]
        return {"n_samples": n_samples, "received": len(data),
                "nonzero": sum(1 for b in data if b), "max": max(data) if data else 0}
    return None


def ping360_discovery(timeout: float = 2.0) -> list:
    """Broadcast "Discovery" on UDP 30303 and collect Ping360 replies.

    Sent from an ephemeral port so replies come back to us and not to the
    BlueOS ping service, which binds 30303 itself."""
    found = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.settimeout(0.3)
        s.bind(("", 0))
        for _ in range(2):
            s.sendto(b"Discovery", ("255.255.255.255", 30303))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data, addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            text = data.decode("ascii", errors="ignore")
            if "Discovery" == text.strip():
                continue
            lines = [ln.strip() for ln in text.replace("\r", "").split("\n") if ln.strip()]
            ip = addr[0]
            mac = None
            port = 12345
            for ln in lines:
                if ln.startswith("IP Address:-"):
                    ip = ".".join(str(int(p)) for p in ln.split("-", 1)[1].strip().split("."))
                elif ln.startswith("MAC Address:-"):
                    mac = ln.split("-", 1)[1].strip().replace("-", ":").lower()
                elif ln.startswith("Port:-"):
                    port = int(ln.split("-", 1)[1].strip())
            found[ip] = {"ip": ip, "port": port, "mac": mac,
                         "device": lines[0] if lines else "", "is_ping360": "PING360" in text.upper()}
    except OSError as e:
        logger.warning("Ping360 discovery failed: %s", e)
    finally:
        s.close()
    return list(found.values())


# ---------------------------------------------------------------------------
# Subnet scan
# ---------------------------------------------------------------------------
def blueos_json(path: str, timeout: float = 3.0):
    try:
        r = requests.get(f"{BLUEOS}{path}", timeout=timeout)
        if r.status_code == 200:
            return r.json()
    except Exception as e:  # noqa: BLE001
        logger.debug("GET %s failed: %s", path, e)
    return None


def local_subnets() -> tuple:
    """Return (subnets to scan, this vehicle's own IPs)."""
    nets, own = set(), set()
    eth = blueos_json("/cable-guy/v1.0/ethernet") or []
    for iface in eth if isinstance(eth, list) else []:
        for addr in iface.get("addresses") or []:
            ip = addr.get("ip")
            if not ip:
                continue
            own.add(ip)
            try:
                net = ipaddress.ip_network(f"{ip}/24", strict=False)
                if net.is_private and not net.is_link_local:
                    nets.add(str(net))
            except ValueError:
                pass
    if not nets:
        nets.add(DEFAULT_SUBNET)
    return sorted(nets), own


def icmp_ping(ip: str) -> bool:
    try:
        return subprocess.run(["ping", "-c", "1", "-W", "1", ip], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=3).returncode == 0
    except Exception:  # noqa: BLE001
        return False


def tcp_open(ip: str, port: int, timeout: float = 0.6) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def arp_table() -> dict:
    out = {}
    try:
        with open("/proc/net/arp") as f:
            next(f)
            for line in f:
                parts = line.split()
                if len(parts) >= 4 and parts[3] != "00:00:00:00:00:00":
                    out[parts[0]] = parts[3]
    except OSError:
        pass
    return out


def identify(ip: str, ports: list, own: set) -> Optional[str]:
    if ip in own:
        return "This vehicle (BlueOS)"
    if 80 in ports:
        try:
            r = requests.get(f"http://{ip}/api/v1/about", timeout=1.5)
            if r.ok and "DVL" in str(r.json().get("product_name", "")):
                return f"Water Linked {r.json().get('product_name')}"
        except Exception:  # noqa: BLE001
            pass
    if 16171 in ports:
        return "Water Linked DVL"
    if 8081 in ports or ip.endswith(".42"):
        return "Sonoptix Echo (likely)" if 8081 in ports else None
    if 51200 in ports:
        return "Cerulean sonar (Omniscan / S500)"
    if 554 in ports:
        return "IP camera (RTSP)"
    if ip.endswith(".1"):
        return "Topside computer / router (likely)"
    return None


def scan_subnet(subnet: str, own: set) -> list:
    hosts = [str(h) for h in ipaddress.ip_network(subnet).hosts()]
    with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
        alive = {ip for ip, ok in zip(hosts, pool.map(icmp_ping, hosts)) if ok}
    arp = arp_table()
    net = ipaddress.ip_network(subnet)
    alive |= {ip for ip in arp if ipaddress.ip_address(ip) in net}
    alive |= {ip for ip in own if ipaddress.ip_address(ip) in net}

    def probe(ip):
        ports = [p for p in TCP_PORTS if tcp_open(ip, p)]
        return {"ip": ip, "mac": arp.get(ip), "open_ports": ports, "identified": identify(ip, ports, own)}

    with concurrent.futures.ThreadPoolExecutor(max_workers=32) as pool:
        results = list(pool.map(probe, sorted(alive, key=lambda s: ipaddress.ip_address(s))))
    return results


# ---------------------------------------------------------------------------
# Autopilot rangefinder check
# ---------------------------------------------------------------------------
def message_is_live(client: MavlinkClient, name: str, wait_s: float = 1.6) -> Optional[dict]:
    msg, stamp = client.message(name)
    if msg is None:
        return None
    time.sleep(wait_s)
    msg2, stamp2 = client.message(name)
    return msg2 if msg2 is not None and stamp2 != stamp else None


def check_rangefinder(client: MavlinkClient) -> list:
    checks = []
    rtype = client.param_get("RNGFND1_TYPE")
    orient = client.param_get("RNGFND1_ORIENT")
    if rtype is None:
        checks.append(_c("Autopilot rangefinder config", "WARN", "RNGFND1_TYPE not available"))
    elif int(rtype) != 10:
        checks.append(_c("Autopilot rangefinder config", "FAIL",
                         f"RNGFND1_TYPE={rtype:g}; must be 10 (MAVLink) to use the Ping altimeter from BlueOS"))
    else:
        detail = "RNGFND1_TYPE=10 (MAVLink)"
        res = "PASS"
        if orient is not None and int(orient) != 25:
            res = "FAIL"
            detail += f", but RNGFND1_ORIENT={orient:g}; BlueOS sends orientation 25 (down) so readings are ignored"
        checks.append(_c("Autopilot rangefinder config", res, detail))
    live = message_is_live(client, "RANGEFINDER") or message_is_live(client, "DISTANCE_SENSOR", 0.5)
    if live:
        dist = live.get("distance")
        if dist is None and live.get("current_distance") is not None:
            dist = live["current_distance"] / 100.0
        checks.append(_c("Autopilot consuming DISTANCE_SENSOR", "PASS",
                         f"autopilot reports rangefinder distance {dist} m"))
    else:
        checks.append(_c("Autopilot consuming DISTANCE_SENSOR", "FAIL",
                         "no live RANGEFINDER/DISTANCE_SENSOR from the autopilot"))
    return checks


def _c(name: str, result: str, detail: str = "") -> dict:
    return {"name": name, "result": result, "detail": detail}


# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------
def run_scan(client: MavlinkClient) -> dict:
    t0 = time.monotonic()
    checks: list = []
    subnets, own = local_subnets()

    # Ping devices known to BlueOS's ping service (USB/serial + ethernet)
    sensors = blueos_json("/ping/v1.0/sensors")
    if sensors is None:
        checks.append(_c("BlueOS Ping service", "WARN", "/ping/v1.0/sensors not reachable"))
        sensors = []
    else:
        checks.append(_c("BlueOS Ping service", "PASS", f"{len(sensors)} device(s) reported"))

    ping1d_with_mavlink = False
    ping1d_found = False
    for s in sensors:
        ptype = s.get("ping_type", "UNKNOWN")
        port = s.get("port") or s.get("ethernet_discovery_info") or "?"
        drv = s.get("driver_status") or {}
        udp = drv.get("udp_port")
        label = f"{ptype} on {port}"
        if s.get("ethernet_discovery_info"):
            continue      # verified directly below via discovery
        if not udp:
            checks.append(_c(label, "WARN", "detected but the BlueOS bridge is not running yet"))
            continue
        info = device_information("127.0.0.1", udp)
        if not info:
            checks.append(_c(label, "FAIL", f"no device_information reply through bridge UDP {udp}"))
            continue
        fw = f"fw {info['firmware']}"
        if ptype == "Ping1D":
            ping1d_found = True
            d = ping1d_distance("127.0.0.1", udp)
            if d:
                checks.append(_c(label, "PASS", f"{fw}, distance {d['distance_m']} m @ {d['confidence']}% confidence"))
            else:
                checks.append(_c(label, "FAIL", f"{fw}, but no distance reply"))
            if drv.get("mavlink_driver_enabled"):
                ping1d_with_mavlink = True
            else:
                checks.append(_c(f"{label} → MAVLink", "WARN",
                                 "DISTANCE_SENSOR forwarding is disabled in BlueOS (Ping Sonar Devices page)"))
        elif ptype == "Ping360":
            sm = ping360_samples("127.0.0.1", udp)
            if sm and sm["received"]:
                checks.append(_c(label, "PASS",
                                 f"{fw}, bridge UDP {udp} forwards sonar data ({sm['received']} samples, max {sm['max']})"))
            else:
                checks.append(_c(label, "FAIL", f"{fw}, no sonar data through bridge UDP {udp}"))
        else:
            checks.append(_c(label, "WARN", f"{fw}, unrecognised Ping device"))

    if ping1d_found:
        if ping1d_with_mavlink:
            checks.extend(check_rangefinder(client))

    # Ethernet Ping360s
    discovered = ping360_discovery()
    for d in discovered:
        label = f"{'Ping360' if d['is_ping360'] else d['device'] or 'Ping device'} @ {d['ip']}:{d['port']}"
        info = device_information(d["ip"], d["port"])
        if not info:
            checks.append(_c(label, "FAIL", "answered discovery but not device_information"))
            continue
        sm = ping360_samples(d["ip"], d["port"])
        if sm and sm["received"]:
            checks.append(_c(label, "PASS", f"fw {info['firmware']}, {sm['received']} samples received, max {sm['max']}"))
        else:
            checks.append(_c(label, "FAIL", f"fw {info['firmware']}, no sonar data returned"))

    # Subnet scan
    hosts = []
    for net in subnets:
        hosts.extend(scan_subnet(net, own))
    for d in discovered:
        h = next((h for h in hosts if h["ip"] == d["ip"]), None)
        if h:
            h["identified"] = "Ping360 (Ethernet)"
        else:
            hosts.append({"ip": d["ip"], "mac": d["mac"], "open_ports": [], "identified": "Ping360 (Ethernet)"})
    for h in hosts:
        ident = h.get("identified") or ""
        if any(k in ident for k in ("DVL", "Sonoptix", "Cerulean")):
            checks.append(_c(f"{ident} @ {h['ip']}", "PASS",
                             f"responding on TCP {', '.join(map(str, h['open_ports']))}"))

    sonar_count = sum(1 for c in checks if c["name"] not in (
        "BlueOS Ping service", "Autopilot rangefinder config", "Autopilot consuming DISTANCE_SENSOR")
        and "→ MAVLink" not in c["name"])
    results = [c["result"] for c in checks]
    overall = "FAIL" if "FAIL" in results else ("WARN" if "WARN" in results else "PASS")
    summary = (f"{len(hosts)} host(s) on {', '.join(subnets)}; "
               f"{sonar_count} sonar/device check(s); {time.monotonic() - t0:.0f} s")
    if not sonar_count:
        summary += "; no sonars detected"
    return {"result": overall, "summary": summary, "subnet": ", ".join(subnets),
            "checks": checks, "hosts": hosts, "ping_sensors": sensors}


def sim_scan() -> dict:
    return {
        "result": "WARN", "summary": "Simulated scan", "subnet": DEFAULT_SUBNET,
        "checks": [_c("BlueOS Ping service", "PASS", "2 device(s) reported"),
                   _c("Ping1D on /dev/ttyUSB0", "PASS", "fw 3.29.0, distance 1.234 m @ 100% confidence"),
                   _c("Autopilot rangefinder config", "PASS", "RNGFND1_TYPE=10 (MAVLink)"),
                   _c("Autopilot consuming DISTANCE_SENSOR", "PASS", "autopilot reports rangefinder distance 1.23 m"),
                   _c("Ping360 @ 192.168.2.4:12345", "WARN", "simulated")],
        "hosts": [{"ip": "192.168.2.1", "mac": "aa:bb:cc:00:00:01", "open_ports": [], "identified": "Topside computer / router (likely)"},
                  {"ip": "192.168.2.2", "mac": None, "open_ports": [80], "identified": "This vehicle (BlueOS)"},
                  {"ip": "192.168.2.4", "mac": "54:10:ec:79:7d:c5", "open_ports": [], "identified": "Ping360 (Ethernet)"}],
        "ping_sensors": [],
    }


if __name__ == "__main__":   # manual run: python devices.py
    logging.basicConfig(level=logging.INFO)
    c = MavlinkClient()
    c.connect()
    print(json.dumps(run_scan(c), indent=2))
