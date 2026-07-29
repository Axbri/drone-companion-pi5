"""droneweb — companion-webbgränssnitt på dronepi.

Steg 1: video (MJPEG) + telemetri (batteri/roll/pitch/heading) + Pi-prestanda
(CPU/RAM/nät/temp) + avstängningsknapp. Kör bakom systemd som `axel`, port 8080,
nås över LAN/Tailscale (ingen inloggning — betrott nät).
"""
import os
import subprocess
import time

import psutil
from flask import Flask, Response, jsonify, render_template, request

from camera import Camera
from mavlink import MavlinkTelemetry

app = Flask(__name__)
cam = Camera()
tel = MavlinkTelemetry()

# precland skapas lazy vid första arm → cv2 importeras först då (sparar RAM i Steg 1)
_precland = None


def _get_precland():
    global _precland
    if _precland is None:
        from precland import PrecLandController
        from rangefinder import RangeFinder
        _precland = PrecLandController(cam, tel, RangeFinder(tel))
    return _precland

_NET_IFACE = "wlan0"
_net = {"t": time.time(), "rx": None, "tx": None}  # baseline sätts vid första pollen


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video.mjpg")
def video():
    return Response(
        _mjpeg(), mimetype="multipart/x-mixed-replace; boundary=FRAME"
    )


def _mjpeg():
    for frame in cam.frames():
        yield b"--FRAME\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@app.route("/api/telemetry")
def api_telemetry():
    return jsonify(tel.snapshot())


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


@app.route("/api/precland", methods=["GET", "POST"])
def api_precland():
    if request.method == "POST":
        want = request.get_json(force=True, silent=True) or {}
        pc = _get_precland()
        pc.arm() if want.get("armed") else pc.disarm()
    if _precland is None:
        return jsonify({"armed": False, "phase": None, "rangefinder_ok": False})
    return jsonify(_precland.get_status())


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
