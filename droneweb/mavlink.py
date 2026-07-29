"""MAVLink-telemetri för droneweb.

Läser telemetri från mavproxy via en egen UDP-endpoint (mavproxy startas med
`--out udpin:127.0.0.1:14551`). Vi skickar heartbeats så mavproxy lär sig vår
adress och forwardar trafik hit. Samma anslutning används i Steg 2 för att
skicka LANDING_TARGET till FC:n (mavproxy forwardar companion→master).

Trådsäker: en bakgrundstråd uppdaterar `self.data`; `snapshot()` läser en kopia.
"""
import math
import threading
import time

from pymavlink import mavutil

DEFAULT_URL = "udpout:127.0.0.1:14551"


class MavlinkTelemetry:
    def __init__(self, url=DEFAULT_URL):
        self.url = url
        self._data = {}
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()   # serialisera sändningar (flera trådar)
        self.master = None
        self._connected = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---- bakgrundstråd --------------------------------------------------
    def _run(self):
        while True:
            try:
                self.master = mavutil.mavlink_connection(
                    self.url, source_system=200, source_component=191
                )
                last_hb = 0.0
                while True:
                    now = time.time()
                    if now - last_hb > 1.0:
                        self._send_heartbeat()
                        last_hb = now
                    msg = self.master.recv_match(blocking=True, timeout=2)
                    if msg is not None:
                        self._handle(msg)
                        with self._lock:
                            self._connected = True
            except Exception:
                with self._lock:
                    self._connected = False
                time.sleep(2)  # reconnecta

    def _send_heartbeat(self):
        with self._send_lock:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )

    def _handle(self, msg):
        t = msg.get_type()
        d = {}
        if t == "ATTITUDE":
            d["roll"] = round(math.degrees(msg.roll), 1)
            d["pitch"] = round(math.degrees(msg.pitch), 1)
            d["yaw"] = round(math.degrees(msg.yaw) % 360, 1)
        elif t == "VFR_HUD":
            d["heading"] = msg.heading
            d["alt"] = round(msg.alt, 1)
            d["groundspeed"] = round(msg.groundspeed, 1)
            d["climb"] = round(msg.climb, 1)
        elif t == "SYS_STATUS":
            d["voltage"] = round(msg.voltage_battery / 1000.0, 2)
            d["current"] = round(msg.current_battery / 100.0, 1)  # cA -> A
            d["battery_remaining"] = msg.battery_remaining        # %
        elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
            d["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                d["mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                pass
        elif t == "GPS_RAW_INT":
            d["fix_type"] = msg.fix_type
            d["satellites"] = msg.satellites_visible
        if d:
            with self._lock:
                self._data.update(d)
                self._data["updated"] = round(time.time(), 1)

    # ---- publikt API ----------------------------------------------------
    def snapshot(self):
        with self._lock:
            out = dict(self._data)
            out["connected"] = self._connected
            return out

    def send_landing_target(self, angle_x, angle_y, distance):
        """Steg 2: skicka LANDING_TARGET (vinklar i rad i kroppsframe, avstånd i m)."""
        if self.master is None:
            return
        with self._send_lock:
            self.master.mav.landing_target_send(
                int(time.time() * 1e6),  # time_usec
                0,                        # target_num
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                float(angle_x), float(angle_y), float(distance),
                0.0, 0.0,                 # size_x, size_y
            )

    def send_distance_sensor(self, cm, min_cm=10, max_cm=800):
        """Relä TF-Luna AGL till FC (nedåtriktad laser)."""
        if self.master is None:
            return
        with self._send_lock:
            self.master.mav.distance_sensor_send(
                int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
                int(min_cm), int(max_cm), int(cm),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                1,                                       # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,  # nedåt = 25
                0,                                       # covariance
            )
