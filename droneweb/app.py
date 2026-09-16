"""droneweb — companion-webbgränssnitt på dronepi.

Steg 1: video (MJPEG) + telemetri (batteri/roll/pitch/heading) + Pi-prestanda
(CPU/RAM/nät/temp) + avstängningsknapp. Kör bakom systemd som `axel`, port 8080,
nås över LAN/Tailscale (ingen inloggning — betrott nät).
"""
import os
import re
import subprocess
import threading
import time

import psutil
from flask import (Flask, Response, jsonify, render_template, request,
                   send_from_directory)

from camera import Camera
from camera_hq import HQCamera
from mavlink import MavlinkTelemetry
from precland import PrecLandController
from rangefinder import RangeFinder
from rtk import RtkMonitor

app = Flask(__name__)
cam = Camera()        # imx296 — spårningskamera (alltid på, se camera.py)
hq_cam = HQCamera()   # imx477 — Pilot view FPV stream only
tel = MavlinkTelemetry()
rf = RangeFinder(tel)   # alltid på: reläar TF-Luna DISTANCE_SENSOR till FC från boot (lätt, ingen cv2)
rtkmon = RtkMonitor()   # pollar NTRIP-basen + injektorn i bakgrunden
precland = PrecLandController(cam, tel, rf)   # spårningen kör kontinuerligt från start (Steg 3)
RECORDINGS_DIR = "/home/axel/recordings"

_NET_IFACE = "wlan0"
_net = {"t": time.time(), "rx": None, "tx": None}  # baseline sätts vid första pollen


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video.mjpg")
def video():
    return Response(
        _mjpeg(cam), mimetype="multipart/x-mixed-replace; boundary=FRAME"
    )


@app.route("/video_hq.mjpg")
def video_hq():
    return Response(
        _mjpeg(hq_cam), mimetype="multipart/x-mixed-replace; boundary=FRAME"
    )


def _mjpeg(source):
    for frame in source.frames():
        yield b"--FRAME\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@app.route("/api/telemetry")
def api_telemetry():
    return jsonify(tel.snapshot())


@app.route("/api/time")
def api_time():
    # Pi:ns egen systemklocka (inte webbläsarens) — den som styr filnamn/loggar,
    # och en löpande synlig kontroll efter GPS-tid-fallbacken (mavlink.py).
    return jsonify({"time": time.strftime("%Y-%m-%d %H:%M:%S")})


@app.route("/api/mission")
def api_mission():
    return jsonify(tel.mission_snapshot())


@app.route("/api/mission/read", methods=["POST"])
def api_mission_read():
    tel.read_mission()
    return jsonify({"ok": True})


@app.route("/api/rtk")
def api_rtk():
    st = rtkmon.status()
    t = tel.snapshot()
    st["fix_type"] = t.get("fix_type")
    st["satellites"] = t.get("satellites")
    return jsonify(st)


@app.route("/api/stats")
def api_stats():
    vm = psutil.virtual_memory()
    now = time.time()
    dt = max(0.001, now - _net["t"])
    rx_kbps = tx_kbps = 0.0
    io = psutil.net_io_counters(pernic=True).get(_NET_IFACE)
    if io:
        if _net["rx"] is not None:  # hoppa över första pollen (ingen baslinje än)
            rx_kbps = (io.bytes_recv - _net["rx"]) * 8 / 1000.0 / dt
            tx_kbps = (io.bytes_sent - _net["tx"]) * 8 / 1000.0 / dt
        _net.update(t=now, rx=io.bytes_recv, tx=io.bytes_sent)
    temp = _vcgen_temp()
    return jsonify(
        {
            "cpu_percent": psutil.cpu_percent(),
            "mem_used_mb": round((vm.total - vm.available) / 1e6),
            "mem_total_mb": round(vm.total / 1e6),
            "mem_percent": vm.percent,
            "net_rx_kbps": round(rx_kbps, 1),
            "net_tx_kbps": round(tx_kbps, 1),
            "temp_c": temp,
            "loadavg": round(os.getloadavg()[0], 2),
            "viewers": cam.viewers,
        }
    )


def _vcgen_temp():
    try:
        out = subprocess.check_output(["vcgencmd", "measure_temp"], timeout=2)
        return float(out.decode().split("=")[1].split("'")[0])
    except Exception:
        return None


@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    subprocess.Popen(["sudo", "/sbin/shutdown", "-h", "now"])
    return jsonify({"status": "shutting down"})


# ---- nätverk (hemma-WiFi vs 4G-dongle) ---------------------------------
WIFI_IFACE, WWAN_IFACE = "wlan0", "wwan0"
_WIFI_REVERT_S = 600            # slå på WiFi igen automatiskt efter 10 min (skydd mot utelåsning)
_wifi_revert = {"timer": None, "at": None}
_wifi_lock = threading.Lock()


def _sh(cmd):
    try:
        return subprocess.check_output(cmd, timeout=5, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""


def _network_status():
    m = re.search(r"dev (\S+)", _sh(["ip", "route", "get", "8.8.8.8"]))
    act = m.group(1) if m else None
    ssid = None
    for line in _sh(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"]).splitlines():
        if line.startswith("yes:"):
            ssid = line.split(":", 1)[1]
            break
    ip4 = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", _sh(["ip", "-4", "-o", "addr", "show", WWAN_IFACE]))
    active = "wifi" if act == WIFI_IFACE else ("4g" if act == WWAN_IFACE else "none")
    with _wifi_lock:
        rev_at = _wifi_revert["at"]
    return {
        "active": active,
        "active_iface": act,
        "wifi_enabled": _sh(["nmcli", "radio", "wifi"]) == "enabled",
        "wifi_ssid": ssid,
        "wwan_ip": ip4.group(1) if ip4 else None,
        "revert_in": max(0, int(rev_at - time.time())) if rev_at else None,
    }


def _wifi_set(enabled):
    subprocess.run(["sudo", "nmcli", "radio", "wifi", "on" if enabled else "off"],
                   timeout=10, check=False)


def _wifi_revert_now():
    _wifi_set(True)
    with _wifi_lock:
        _wifi_revert.update(timer=None, at=None)


@app.route("/api/network")
def api_network():
    return jsonify(_network_status())


@app.route("/api/wifi", methods=["POST"])
def api_wifi():
    enabled = bool((request.get_json(force=True, silent=True) or {}).get("enabled"))
    with _wifi_lock:
        if _wifi_revert["timer"]:
            _wifi_revert["timer"].cancel()
        _wifi_revert.update(timer=None, at=None)
    _wifi_set(enabled)
    if not enabled:                      # tvingar 4G → auto-återställ WiFi som skydd mot utelåsning
        t = threading.Timer(_WIFI_REVERT_S, _wifi_revert_now)
        t.daemon = True
        t.start()
        with _wifi_lock:
            _wifi_revert.update(timer=t, at=time.time() + _WIFI_REVERT_S)
    return jsonify(_network_status())


@app.route("/api/precland")
def api_precland():
    return jsonify(precland.get_status())


@app.route("/api/exposure", methods=["POST"])
def api_exposure():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_exposure(
        auto=want.get("auto"),
        exposure_us=want.get("exposure_us"),
        gain=want.get("gain"),
    ))


@app.route("/api/exposure_hq", methods=["GET", "POST"])
def api_exposure_hq():
    if request.method == "POST":
        want = request.get_json(force=True, silent=True) or {}
        hq_cam.set_exposure(
            auto=want.get("auto"),
            exposure_us=want.get("exposure_us"),
            gain=want.get("gain"),
        )
    return jsonify(hq_cam.exposure_status())


@app.route("/api/landscale", methods=["POST"])
def api_landscale():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify({"cmd_scale": precland.set_cmd_scale(want.get("scale", 1.0))})


@app.route("/api/camoffset", methods=["POST"])
def api_camoffset():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_cam_offset(want.get("offset_cm")))


@app.route("/api/markersize", methods=["POST"])
def api_markersize():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_marker_size(want.get("marker_cm")))


@app.route("/api/thresholds", methods=["POST"])
def api_thresholds():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_thresholds(
        aruco_start_agl=want.get("aruco_start_agl"),
        rtk_agl=want.get("rtk_agl"),
        yaw_start_agl=want.get("yaw_start_agl"),
    ))


@app.route("/api/yawalign", methods=["POST"])
def api_yawalign():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_yaw_align(
        enabled=want.get("enabled"),
        rate_degs=want.get("rate_degs"),
    ))


@app.route("/api/targetmode", methods=["POST"])
def api_targetmode():
    want = request.get_json(force=True, silent=True) or {}
    return jsonify(precland.set_target_mode(
        moving=want.get("moving"),
        coast_s=want.get("coast_s"),
    ))


@app.route("/api/record", methods=["POST"])
def api_record():
    want = request.get_json(force=True, silent=True) or {}
    precland.start_recording() if want.get("recording") else precland.stop_recording()
    return jsonify(precland.get_status())


@app.route("/api/recordings")
def api_recordings():
    recs = []
    if os.path.isdir(RECORDINGS_DIR):
        for f in sorted(os.listdir(RECORDINGS_DIR), reverse=True):
            if not f.endswith(".avi"):
                continue
            base = f[:-4]
            p = os.path.join(RECORDINGS_DIR, f)
            has_csv = os.path.exists(os.path.join(RECORDINGS_DIR, base + ".csv"))
            recs.append({"name": base, "size_mb": round(os.path.getsize(p) / 1e6, 1),
                         "csv": has_csv})
    return jsonify(recs)


@app.route("/recordings/<path:fname>")
def download_recording(fname):
    return send_from_directory(RECORDINGS_DIR, fname, as_attachment=True)


@app.route("/api/recordings/delete", methods=["POST"])
def delete_recording():
    name = os.path.basename((request.get_json(force=True, silent=True) or {}).get("name", ""))
    if name:
        for ext in (".avi", ".csv"):
            p = os.path.join(RECORDINGS_DIR, name + ext)
            if os.path.exists(p):
                os.remove(p)
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
