"""MAVLink-telemetri för droneweb.

Läser telemetri från mavproxy via en egen UDP-endpoint (mavproxy startas med
`--out udpin:127.0.0.1:14551`). Vi skickar heartbeats så mavproxy lär sig vår
adress och forwardar trafik hit. Samma anslutning används i Steg 2 för att
skicka LANDING_TARGET till FC:n (mavproxy forwardar companion→master).

Trådsäker: en bakgrundstråd uppdaterar `self.data`; `snapshot()` läser en kopia.
"""
import math
import os
import subprocess
import threading
import time

from pymavlink import mavutil

DEFAULT_URL = "udpout:127.0.0.1:14551"
ATTITUDE_HZ = 20   # HUD smoothness — ArduPilot's default ATTITUDE stream rate is
                   # only ~4Hz; the 921600 baud FC link has plenty of headroom for this.

# ---- systemklocka från FC:ns GPS-tid (fallback, ingen RTC-batteri på Pi:n) ---------
# Pi 5:ns inbyggda RTC har ingen batteribackup här → startar alltid om från noll (epok),
# och NTP kan dröja mycket längre än förväntat i fält (uppmätt: 20+ timmar en gång, se
# PI5-SETUP.md). GPS-tid från FC:n är alltid tillgänglig under flygning och beror inte
# på en server. SYSTEM_TIME.time_unix_usec sätts av ArduPilot från GPS när den har fix —
# vi litar bara på den vid 3D-fix + ett rimlighetstest, och rör ALDRIG klockan om NTP
# redan har synkat (då är NTP mer exakt och får vara auktoritativ).
GPS_TIME_MIN_EPOCH = 1735689600.0   # 2025-01-01 UTC — allt tidigare är uppenbart fel
GPS_TIME_SET_INTERVAL_S = 30        # hur ofta vi försöker, inte varje SYSTEM_TIME-meddelande
NTP_SYNCED_FLAG = "/run/systemd/timesync/synchronized"   # skapas av timesyncd efter lyckad sync

# ---- mission/fence/rally download (MAVLink mission protocol), för Map-fliken --------
FC_SYS, FC_COMP = 1, 1          # ArduPilot FC:s standard sysid/compid
MT_MISSION, MT_FENCE, MT_RALLY = 0, 1, 2
GLOBAL_FRAMES = {0, 3, 5, 6, 10, 11}   # MAV_FRAME_* globala varianter (x=lat*1e7, y=lon*1e7)
FRAME_ALT_LABEL = {0: "AMSL", 3: "rel", 5: "rel", 6: "rel", 10: "terrain", 11: "terrain"}
MISSION_DL_RETRY_S = 2.0
MISSION_DL_MAX_TRIES = 6


class MavlinkTelemetry:
    def __init__(self, url=DEFAULT_URL):
        self.url = url
        self._data = {}
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()   # serialisera sändningar (flera trådar)
        self.master = None
        self._connected = False
        self._mission = {"version": 0, "items": [], "jumps": [], "fence": [], "rally": []}
        self._mdl = {"active": False, "mtype": 0, "count": -1, "items": {},
                     "last_req": 0.0, "tries": 0}
        self._last_gps_time_set = 0.0
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
                rate_requested = False
                while True:
                    now = time.time()
                    if now - last_hb > 1.0:
                        self._send_heartbeat()
                        last_hb = now
                    if self._mdl["active"] and now - self._mdl["last_req"] > MISSION_DL_RETRY_S:
                        self._retry_mission_dl()
                    msg = self.master.recv_match(blocking=True, timeout=2)
                    if msg is not None:
                        self._handle(msg)
                        with self._lock:
                            self._connected = True
                        if not rate_requested and msg.get_type() == "HEARTBEAT" \
                                and msg.get_srcComponent() == 1:
                            self._request_attitude_rate(msg.get_srcSystem(), msg.get_srcComponent())
                            rate_requested = True
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

    def _request_attitude_rate(self, sysid, compid):
        """MAV_CMD_SET_MESSAGE_INTERVAL for ATTITUDE — pushes it well past ArduPilot's
        default ~4Hz stream rate so the Pilot view HUD doesn't look choppy."""
        with self._send_lock:
            self.master.mav.command_long_send(
                sysid, compid,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                int(1e6 / ATTITUDE_HZ),
                0, 0, 0, 0, 0,
            )

    def _handle(self, msg):
        t = msg.get_type()
        # Mission-nedladdning (mission/fence/rally): eget tillstånd + uppföljande
        # sändningar, hanteras separat från det enkla fält-mergen nedan.
        if t == "MISSION_COUNT" and self._mdl["active"] \
                and getattr(msg, "mission_type", 0) == self._mdl["mtype"]:
            with self._lock:
                self._mdl["count"] = msg.count
                self._mdl["items"] = {}
            if msg.count == 0:
                self._finalize_mission()
            else:
                self._request_mission_item(0)
            return
        if t == "MISSION_ITEM_INT" and self._mdl["active"] \
                and getattr(msg, "mission_type", 0) == self._mdl["mtype"]:
            with self._lock:
                self._mdl["items"][msg.seq] = msg
                done = len(self._mdl["items"]) >= self._mdl["count"]
            if done:
                self._finalize_mission()
            else:
                nxt = next(i for i in range(self._mdl["count"]) if i not in self._mdl["items"])
                self._request_mission_item(nxt)
            return

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
        elif t == "GLOBAL_POSITION_INT":
            d["lat"] = msg.lat / 1e7
            d["lon"] = msg.lon / 1e7
        elif t == "MISSION_CURRENT":
            d["cur_wp"] = msg.seq
        elif t == "SYSTEM_TIME":
            self._maybe_set_clock_from_gps(msg.time_unix_usec)
        if d:
            with self._lock:
                self._data.update(d)
                self._data["updated"] = round(time.time(), 1)

    def _maybe_set_clock_from_gps(self, time_unix_usec):
        """Sätter Pi:ns systemklocka från FC:ns GPS-tid — fallback när NTP inte har
        synkat än (ingen RTC-batteri på denna Pi, se PI5-SETUP.md). Rör ALDRIG klockan
        om NTP redan har synkat (då är NTP mer exakt och auktoritativ). Kräver GPS
        3D-fix + ett rimlighetstest mot en fast datumgräns, som skydd mot uppenbart
        trasiga värden innan FC:n har en riktig fix."""
        now = time.time()
        if now - self._last_gps_time_set < GPS_TIME_SET_INTERVAL_S:
            return
        if os.path.exists(NTP_SYNCED_FLAG):
            return
        with self._lock:
            fix_type = self._data.get("fix_type", 0)
        if fix_type < 3:
            return
        epoch = time_unix_usec / 1e6
        if epoch < GPS_TIME_MIN_EPOCH:
            return
        self._last_gps_time_set = now
        try:
            subprocess.run(["sudo", "-n", "date", "-s", "@%.0f" % epoch],
                           timeout=5, check=True, capture_output=True)
            print(f"[mavlink] Satte systemklockan från GPS-tid (fix_type={fix_type}): "
                  f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(epoch))}")
        except Exception as e:
            print(f"[mavlink] VARNING: kunde inte sätta klockan från GPS-tid: {e}")

    # ---- mission/fence/rally-nedladdning ---------------------------------
    def read_mission(self):
        """Starta läsning av mission → fence → rally (kedjad), för Map-fliken."""
        self._start_dl(MT_MISSION)

    def mission_snapshot(self):
        with self._lock:
            return dict(self._mission)

    def _start_dl(self, mtype):
        with self._lock:
            self._mdl.update(active=True, mtype=mtype, count=-1, items={},
                             last_req=time.time(), tries=0)
        with self._send_lock:
            self.master.mav.mission_request_list_send(FC_SYS, FC_COMP, mtype)

    def _request_mission_item(self, seq):
        with self._lock:
            self._mdl["last_req"] = time.time()
        with self._send_lock:
            self.master.mav.mission_request_int_send(FC_SYS, FC_COMP, seq, self._mdl["mtype"])

    def _retry_mission_dl(self):
        """Kallas när nedladdningen stannat > MISSION_DL_RETRY_S — en tappad
        request skulle annars fastna för alltid."""
        with self._lock:
            self._mdl["tries"] += 1
            tries, mt, count = self._mdl["tries"], self._mdl["mtype"], self._mdl["count"]
        if tries > MISSION_DL_MAX_TRIES:
            if mt == MT_FENCE:                # håll kedjan igång så den ändå blir klar
                with self._lock:
                    self._mission["fence"] = []
                self._start_dl(MT_RALLY)
            elif mt == MT_RALLY:
                with self._lock:
                    self._mission["rally"] = []
                    self._mission["version"] += 1
                    self._mdl["active"] = False
            else:
                with self._lock:
                    self._mdl["active"] = False
        elif count <= 0:
            with self._lock:
                self._mdl["last_req"] = time.time()
            with self._send_lock:
                self.master.mav.mission_request_list_send(FC_SYS, FC_COMP, mt)
        else:
            with self._lock:
                missing = [i for i in range(count) if i not in self._mdl["items"]]
            if missing:
                self._request_mission_item(missing[0])

    def _parse_fence(self, by_seq):
        """ArduPilot fence-objekt -> ritbara former (polygoner, cirklar, return point).
        MAV_CMD: 5000 RETURN_POINT, 5001/5002 POLYGON_VERTEX incl/excl (param1 = antal
        hörn), 5003/5004 CIRCLE incl/excl (param1 = radie)."""
        shapes, seqs, i = [], sorted(by_seq), 0
        while i < len(seqs):
            it = by_seq[seqs[i]]
            cmd = it.command
            if cmd in (5001, 5002):
                n = int(it.param1) or 1
                pts = []
                for k in range(n):
                    if i + k >= len(seqs):
                        break
                    v = by_seq[seqs[i + k]]
                    if v.command != cmd:
                        break
                    pts.append([v.x / 1e7, v.y / 1e7])
                shapes.append({"type": "polygon", "inclusion": cmd == 5001, "points": pts})
                i += max(1, len(pts))
            elif cmd in (5003, 5004):
                shapes.append({"type": "circle", "inclusion": cmd == 5003,
                               "lat": it.x / 1e7, "lon": it.y / 1e7, "radius": float(it.param1)})
                i += 1
            elif cmd == 5000:
                shapes.append({"type": "return", "lat": it.x / 1e7, "lon": it.y / 1e7})
                i += 1
            else:
                i += 1
        return shapes

    def _finalize_mission(self):
        mt = self._mdl["mtype"]
        with self._send_lock:
            self.master.mav.mission_ack_send(FC_SYS, FC_COMP, 0, mt)

        if mt == MT_MISSION:
            items, jumps = [], []
            for s in sorted(self._mdl["items"]):
                it = self._mdl["items"][s]
                if it.command == mavutil.mavlink.MAV_CMD_DO_JUMP:   # 177: inga koord., param1 = mål
                    jumps.append({"seq": s, "target": int(it.param1), "repeat": int(it.param2)})
                elif it.frame in GLOBAL_FRAMES and (it.x or it.y):
                    items.append({"seq": s, "lat": it.x / 1e7, "lon": it.y / 1e7,
                                  "alt": round(it.z, 1),
                                  "alt_label": FRAME_ALT_LABEL.get(it.frame, "f%d" % it.frame),
                                  "cmd": it.command})
            with self._lock:
                self._mission["items"] = items
                self._mission["jumps"] = jumps
            self._start_dl(MT_FENCE)
        elif mt == MT_FENCE:
            shapes = self._parse_fence(self._mdl["items"])
            with self._lock:
                self._mission["fence"] = shapes
            self._start_dl(MT_RALLY)
        else:                                                        # MT_RALLY -> klar
            rally = [{"seq": s, "lat": self._mdl["items"][s].x / 1e7, "lon": self._mdl["items"][s].y / 1e7}
                     for s in sorted(self._mdl["items"]) if (self._mdl["items"][s].x or self._mdl["items"][s].y)]
            with self._lock:
                self._mission["rally"] = rally
                self._mission["version"] += 1
                self._mdl["active"] = False

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
                0,                        # time_usec=0 → ArduPilot använder mottagningstid.
                                          # Skicka INTE epok-µs: fel tidsbas kan förvirra
                                          # precland-Kalmanfiltrets latens-kompensering.
                0,                        # target_num
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                float(angle_x), float(angle_y), float(distance),
                0.0, 0.0,                 # size_x, size_y
            )

    def send_condition_yaw(self, heading_deg, rate_degs):
        """MAV_CMD_CONDITION_YAW: sätt auto_yaw-mål (absolut heading, grader, kortaste
        vägen) med given vridhastighet (grader/s). ArduPilots LAND-läge (även med
        precision landing aktiv) läser auto_yaw.get_heading() varje styrcykel oavsett
        pilot-input och rampar dit kontinuerligt själv — stör inte position/höjd-
        styrningen, och kräver inte omsändning för att fortsätta vrida."""
        if self.master is None:
            return
        with self._send_lock:
            self.master.mav.command_long_send(
                FC_SYS, FC_COMP,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
                float(heading_deg) % 360.0, float(rate_degs),
                0,   # param3: riktning, 0 = kortaste vägen
                0,   # param4: 0 = absolut heading (inte relativ)
                0, 0, 0,
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
