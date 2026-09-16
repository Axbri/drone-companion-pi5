#!/home/axel/mav-venv/bin/python
"""roverweb — companion web interface for ArduRover.
Reads telemetry from the mavproxy fan-out (udp:127.0.0.1:14552), pushes it to the
browser over WebSocket, and relays control (arm/mode/manual-drive) back to the FC.
Manual drive uses RC override with a server-side dead-man watchdog.
"""
import asyncio, json, time, threading, socket, os, subprocess
from pymavlink import mavutil
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import Response, JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

# gpiozero (lgpio-backend krävs på Pi 5 — RPi.GPIO funkar EJ) för bumper-GPIO.
# Mjuk import: saknas biblioteket ska webben ändå starta, bara med bumpern
# markerad som "ej tillgänglig". Installera i mav-venv: pip install gpiozero lgpio
try:
    from gpiozero import Button
    GPIO_OK, GPIO_ERR = True, None
except Exception as _gpio_e:      # noqa: BLE001
    Button, GPIO_OK, GPIO_ERR = None, False, str(_gpio_e)

HERE = os.path.dirname(os.path.abspath(__file__))
MAV_URL = "udpout:127.0.0.1:14552"
HOMEBASE = ("homebase", 2101)
WEB_PORT = 8080

# ArduPilot Rover custom modes
ROVER_MODES = {"MANUAL": 0, "ACRO": 1, "STEERING": 3, "HOLD": 4, "LOITER": 5,
               "AUTO": 10, "RTL": 11, "SMART_RTL": 12, "GUIDED": 15}
FIX = {0: "No fix", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS",
       5: "RTK Float", 6: "RTK Fixed", 7: "Static", 8: "PPP"}
RES = {0: "ACCEPTED", 1: "TEMPORARILY REJECTED", 2: "DENIED",
       3: "UNSUPPORTED", 4: "FAILED", 5: "IN PROGRESS"}

state = {
    "link": {"connected": False, "mode": "--", "armed": False},
    "batt": {"voltage": None, "current": None, "remaining": None},
    "gps": {"fix": 0, "fix_str": "--", "sats": 0, "hdop": None, "lat": None, "lon": None},
    "motion": {"speed": 0.0, "heading": 0},
    "pi": {"temp": None, "load": None, "cpu": None, "ram": None},
    "net": {"up_mbps": 0.0, "down_mbps": 0.0, "gbph": 0.0, "iface": ""},
    "video": {"w": 1280, "h": 960, "fps": 30, "bitrate": 0},
    "cur_wp": None,          # MISSION_CURRENT: waypoint the vehicle is navigating to
    "homebase": {"connected": False},
    "msgs": [],   # recent STATUSTEXT / command results
    "dbg": {"age": 999, "active": False, "steer": 0, "throttle": 0},
    # Bumper: fysiska mikroswitchar (V/H). enable = auto-flykt på/av (default PÅ —
    # hårdvaran sitter och Fas 1–3 är verifierade; flykt kräver ändå armad+AUTO/GUIDED).
    # ok = GPIO tillgänglig.
    "bumper": {"left": False, "right": False, "enable": True, "ok": False},
    # Flyktmanöverns tillstånd för UI: fas + antal flykter i fönstret.
    "escape": {"active": False, "phase": "IDLE", "count": 0},
    "ts": 0,
}


def push_msg(sev, text):
    """Append a message to the ring buffer (caller must hold slock)."""
    state["msgs"].append({"sev": int(sev), "text": str(text)})
    del state["msgs"][:-12]
slock = threading.RLock()         # guards state (re-entrant: finalize_mission re-locks)
mlock = threading.Lock()          # guards mav sends
# source_system MUST match the FC's SYSID_MYGCS (default 255) or ArduPilot ignores
# RC_CHANNELS_OVERRIDE (manual drive). Arm/mode don't need the match, but drive does.
mav = mavutil.mavlink_connection(MAV_URL, source_system=255, source_component=190)
target = {"sys": 1, "comp": 1}
drive = {"steer": 0.0, "throttle": 0.0, "ts": 0.0}
# Flykt-kontroll (delas mellan ws-handlern och escape_controller).
#   operator_ts : tidpunkt för senaste operatörsingripande (ws drive/mode/arm) —
#                 avbryter en pågående flykt (operatören vinner alltid).
#   test_side   : "left"/"right" begär ett bänktest (escape_test), None annars.
esc_ctl = {"operator_ts": 0.0, "test_side": None}
mission = {"version": 0, "items": [], "jumps": [], "fence": [], "rally": []}
mdl = {"active": False, "mtype": 0, "count": -1, "items": {}, "last_req": 0.0, "tries": 0}
MT_MISSION, MT_FENCE, MT_RALLY = 0, 1, 2
GLOBAL_FRAMES = {0, 3, 5, 6, 10, 11}    # MAV_FRAME_* global variants (x=lat*1e7, y=lon*1e7)

# ----- Bumper + companion-styrd flyktmanöver (se roverbumper/PLAN.md) -----
# Fysisk frontbygel med en mikroswitch per sida. NC-switch + intern pull-up =
# fail-safe: ej ikörd = pinnen dragen LÅG (switch sluten mot GND), ikörd ELLER
# kabelbrott = pinnen flyter HÖG = TRÄFF. Flykten körs helt i companion-datorn
# via samma RC-override som webb-joysticken (driver-tråden) — ingen inbyggd
# ArduPilot-backning (fälttestat otillräcklig mot nos-mot-vägg).
GPIO_LEFT = 23           # vänster mikroswitch (BCM)
GPIO_RIGHT = 24          # höger mikroswitch (BCM)
BUMPER_DEBOUNCE_S = 0.05
REVERSE_THROTTLE = -0.75  # gas bakåt under backfasen (-1..1)
REVERSE_MIN_S = 2.0      # backa ALLTID minst så länge, även om switchen släpper direkt
                         # (kommer verkligen loss + ger kamera/lidar chans att se hindret)
REVERSE_MAX_S = 2.0      # tak på backfasen även om switcharna aldrig släpper
CLEAR_MARGIN_S = 0.4     # båda switcharna måste vara släppta så länge innan back avslutas
TURN_THROTTLE = 0.0      # ingen gas i svängen -> ren pivot turn (bara styrning på stället)
TURN_STEER = 0.5         # svängkraft/-hastighet (0..1). 0.5 = lugn halvfarts-pivot
TURN_MAX_S = 6.0         # tak på svängen om heading/fusion saknas (höjt för långsammare sväng)
DEFAULT_ESCAPE_ANGLE = 60.0  # föredragen svängvinkel bort från hindret när sidan är öppen
CLEAR_CM = 200           # sektor räknas som "fri att svänga in i" om avstånd >= detta (eller tom)
COOLDOWN_S = 3.0         # ignorera nya triggers en stund efter en flykt
MAX_ESCAPES = 10         # fler flykter än så inom WINDOW_S => rovern sitter fast
WINDOW_S = 120.0         # anti-thrash-fönster (10 flykter inom denna tid => HOLD)
ESCAPE_MODES = ("AUTO", "GUIDED")   # flykt bara i autonoma lägen


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def pwm(x):
    return int(1500 + clamp(x) * 400)   # -1..1 -> 1100..1900


def mav_reader():
    mav.mav.heartbeat_send(6, 8, 0, 0, 0)   # announce as GCS
    last_hb = 0
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1)
            now = time.time()
            if msg is None:
                if now - last_hb > 3:
                    with slock:
                        state["link"]["connected"] = False
                continue
            t = msg.get_type()
            with slock:
                if t == "HEARTBEAT" and msg.type != 6:   # ignore other GCS
                    target["sys"] = msg.get_srcSystem()
                    target["comp"] = msg.get_srcComponent()
                    state["link"]["connected"] = True
                    state["link"]["mode"] = mavutil.mode_string_v10(msg)
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["link"]["armed"] = armed
                    # Auto-starta/stoppa lidarmotorn på arm/disarm-ÖVERGÅNG.
                    if armed != lidar_auto["armed"]:
                        lidar_auto["armed"] = armed
                        set_lidar_motor("on" if armed else "off")
                    last_hb = now
                elif t == "SYS_STATUS":
                    state["batt"]["voltage"] = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
                    state["batt"]["current"] = msg.current_battery / 100.0 if msg.current_battery != -1 else None
                    state["batt"]["remaining"] = msg.battery_remaining if msg.battery_remaining != -1 else None
                elif t == "GPS_RAW_INT":
                    state["gps"]["fix"] = msg.fix_type
                    state["gps"]["fix_str"] = FIX.get(msg.fix_type, str(msg.fix_type))
                    state["gps"]["sats"] = msg.satellites_visible
                    state["gps"]["hdop"] = msg.eph / 100.0 if msg.eph not in (0, 65535) else None
                elif t == "GLOBAL_POSITION_INT":
                    state["gps"]["lat"] = msg.lat / 1e7
                    state["gps"]["lon"] = msg.lon / 1e7
                    if msg.hdg != 65535:
                        state["motion"]["heading"] = msg.hdg / 100.0
                elif t == "VFR_HUD":
                    state["motion"]["speed"] = msg.groundspeed
                    state["motion"]["heading"] = msg.heading
                elif t == "STATUSTEXT":
                    push_msg(msg.severity, msg.text)
                elif t == "COMMAND_ACK":
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        push_msg(2 if msg.result != 0 else 6,
                                 "Arm/disarm: " + RES.get(msg.result, str(msg.result)))
                elif t == "MISSION_CURRENT":
                    state["cur_wp"] = msg.seq
                elif t == "MISSION_COUNT" and mdl["active"] \
                        and getattr(msg, "mission_type", 0) == mdl["mtype"]:
                    mdl["count"] = msg.count
                    mdl["items"] = {}
                    if msg.count == 0:
                        finalize_mission()
                    else:
                        request_mission_item(0)
                elif t == "MISSION_ITEM_INT" and mdl["active"] \
                        and getattr(msg, "mission_type", 0) == mdl["mtype"]:
                    mdl["items"][msg.seq] = msg
                    if len(mdl["items"]) >= mdl["count"]:
                        finalize_mission()
                    else:
                        nxt = next(i for i in range(mdl["count"]) if i not in mdl["items"])
                        request_mission_item(nxt)
                state["ts"] = now
        except Exception:
            time.sleep(0.2)


_prev = [0, 0]
def cpu_percent():
    try:
        with open("/proc/stat") as f:
            v = list(map(int, f.readline().split()[1:]))
        idle = v[3] + v[4]; total = sum(v)
        dt = total - _prev[0]; di = idle - _prev[1]
        _prev[0] = total; _prev[1] = idle
        return round(100 * (1 - di / dt), 1) if dt > 0 else None
    except Exception:
        return None


def read_ram():
    try:
        mem = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":", 1)
                mem[k] = int(v.split()[0])   # kB
        total = mem["MemTotal"] / 1024 / 1024   # GiB
        avail = mem.get("MemAvailable", mem.get("MemFree", 0)) / 1024 / 1024
        used = total - avail
        return {"used_gb": round(used, 2), "total_gb": round(total, 1),
                "pct": round(100 * used / total) if total else 0}
    except Exception:
        return None


def net_iface():
    """Interface carrying the default route (the 4G uplink)."""
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                p = line.split()
                if len(p) > 1 and p[1] == "00000000":   # destination 0.0.0.0
                    return p[0]
    except Exception:
        pass
    return "wlan0"


def net_bytes(iface):
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                if line.split(":")[0].strip() == iface:
                    v = line.split(":")[1].split()
                    return int(v[0]), int(v[8])   # rx_bytes, tx_bytes
    except Exception:
        pass
    return None, None


net_prev = {"t": 0.0, "rx": None, "tx": None, "up": 0.0, "down": 0.0}


# Två detektorer, två filer. V3 (roverav, färgbaserad) och V4 (roverstereo).
# De kan INTE köra samtidigt — båda vill ha cam0 — så vyn visar den som
# faktiskt talar just nu i stället för att blanda ihop dem. Väljs på ÅLDER,
# inte på företräde: den som skriver färskast data är den som kör.
AV_LIVE_PATHS = {"färg": "/dev/shm/roverav_live.json",
                 "stereo": "/dev/shm/roverstereo_live.json"}
STEREO_IMG_PATH = "/dev/shm/roverstereo_live.jpg"
av_live = {"data": None, "age": 999.0}

# 360°-lidarn (roverlidar). Egen sensor, egen fil — sitter på USB-serie och kör
# PARALLELLT med kameradetektorn, så den läses helt fristående och ritas som ett
# eget lager i radarn (blå) ovanpå kamerans sektorer (gröna).
LIDAR_LIVE_PATH = "/dev/shm/roverlidar_live.json"
LIDAR_CMD_PATH = "/dev/shm/roverlidar.cmd"
lidar_live = {"data": None, "age": 999.0}

# Fusionen (roverfusion): den hopslagna hinderbilden som FAKTISKT skickas till
# ArduPilot som OBSTACLE_DISTANCE. Ritas som ett eget lager (röda prickar) ovanpå
# lidar/kamera så man ser exakt vad autopiloten får.
FUSION_LIVE_PATH = "/dev/shm/roverfusion_live.json"
fusion_live = {"data": None, "age": 999.0}


def set_lidar_motor(value):
    """Skriv motorkommando ('on'/'off') till roverlidar-tjänstens triggerfil.

    'on' = tjänsten skickar MotorOn + SetRPM 300. 'off' = MotorOff. Tjänsten
    läser och raderar filen; skriver den bara finns här ett ögonblick."""
    cmd = "on" if str(value).lower() in ("on", "true", "1", "start") else "off"
    try:
        with open(LIDAR_CMD_PATH, "w") as f:
            f.write(cmd)
    except OSError:
        pass


# Auto-styr lidarmotorn på arm/disarm. FLANK-styrt (bara vid ändring), inte
# nivå-styrt: startar vid arm-övergången, stoppar vid disarm-övergången, och rör
# den INTE däremellan — så Start/Stopp-knapparna kan användas manuellt (t.ex. för
# att se radarn med rovern disarmerad) utan att auto-styrningen genast slår över.
# None = ingen heartbeat ännu; första heartbeaten synkar motorn till arm-läget.
lidar_auto = {"armed": None}


def av_reader():
    """Läser detektorernas live-vy (sektorer + status) från /dev/shm.

    Separat process, separat fil — roverweb ska inte kunna störa CV-tjänsten,
    och en död detektor ska synas som gammal data i stället för att blockera
    webbgränssnittet. Därför läses filerna passivt och åldern följer med ut.
    """
    while True:
        best, best_age = None, 999.0
        for source, path in AV_LIVE_PATHS.items():
            try:
                with open(path) as f:
                    d = json.load(f)
                age = max(0.0, time.time() - d.get("t", 0))
                if age < best_age:
                    d.setdefault("source", source)
                    best, best_age = d, age
            except (OSError, ValueError):
                continue
        with slock:
            if best is not None:
                av_live["data"] = best
                av_live["age"] = best_age
            else:
                av_live["age"] = min(av_live["age"] + 0.2, 999.0)
            # Gammal data kastas oavsett var den kom ifrån: en detektor som
            # slutat skriva ska inte lämna en frusen bild kvar på skärmen.
            if av_live["age"] > 3.0:
                av_live["data"] = None
        time.sleep(0.2)


def lidar_reader():
    """Läser 360°-lidarns live-vy från /dev/shm, passivt precis som av_reader.

    En död lidartjänst ska synas som gammal data (och försvinna ur radarn), inte
    frysa fast den sista bilden eller blockera webben."""
    while True:
        try:
            with open(LIDAR_LIVE_PATH) as f:
                d = json.load(f)
            age = max(0.0, time.time() - d.get("t", 0))
            with slock:
                lidar_live["data"] = d
                lidar_live["age"] = age
        except (OSError, ValueError):
            with slock:
                lidar_live["age"] = min(lidar_live["age"] + 0.2, 999.0)
        with slock:
            if lidar_live["age"] > 2.0:
                lidar_live["data"] = None
        time.sleep(0.2)


def fusion_reader():
    """Läser roverfusions live-vy (det som skickas till ArduPilot) från /dev/shm,
    passivt precis som lidar_reader."""
    while True:
        try:
            with open(FUSION_LIVE_PATH) as f:
                d = json.load(f)
            age = max(0.0, time.time() - d.get("t", 0))
            with slock:
                fusion_live["data"] = d
                fusion_live["age"] = age
        except (OSError, ValueError):
            with slock:
                fusion_live["age"] = min(fusion_live["age"] + 0.2, 999.0)
        with slock:
            if fusion_live["age"] > 2.0:
                fusion_live["data"] = None
        time.sleep(0.2)


def pi_reader():
    cpu_percent()
    while True:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                temp = int(f.read()) / 1000.0
        except Exception:
            temp = None
        try:
            load = round(os.getloadavg()[0], 2)
        except Exception:
            load = None
        cpu = cpu_percent()
        hb = False
        try:
            socket.create_connection(HOMEBASE, 2).close(); hb = True
        except Exception:
            pass
        ram = read_ram()
        # network throughput on the uplink interface (bytes/s, EMA-smoothed)
        iface = net_iface()
        wssid, wsig = wifi_current()
        rx, tx = net_bytes(iface)
        now2 = time.time()
        if rx is not None and net_prev["rx"] is not None and rx >= net_prev["rx"] and tx >= net_prev["tx"]:
            dt = now2 - net_prev["t"]
            if dt > 0:
                a = 0.4
                net_prev["down"] = a * (rx - net_prev["rx"]) / dt + (1 - a) * net_prev["down"]
                net_prev["up"] = a * (tx - net_prev["tx"]) / dt + (1 - a) * net_prev["up"]
        net_prev.update(t=now2, rx=rx, tx=tx)
        up, down = net_prev["up"], net_prev["down"]
        with slock:
            state["pi"]["temp"] = round(temp, 1) if temp else None
            state["pi"]["load"] = load
            state["pi"]["cpu"] = cpu
            state["pi"]["ram"] = ram
            state["net"] = {"up_mbps": round(up * 8 / 1e6, 2), "down_mbps": round(down * 8 / 1e6, 2),
                            "gbph": round((up + down) * 3600 / 1e9, 2), "iface": iface,
                            "ssid": wssid, "signal": wsig}
            state["homebase"]["connected"] = hb
        # mission-download retry (a lost request would otherwise stall)
        if mdl["active"] and time.time() - mdl["last_req"] > 2:
            mdl["tries"] += 1
            if mdl["tries"] > 6:
                mt = mdl["mtype"]
                with slock:
                    push_msg(3, "Timeout vid läsning (typ %d) — hoppar vidare" % mt)
                if mt == MT_FENCE:            # keep the chain going so we still finish
                    with slock:
                        mission["fence"] = []
                    _start_dl(MT_RALLY)
                elif mt == MT_RALLY:
                    with slock:
                        mission["rally"] = []
                        mission["version"] += 1
                        mdl["active"] = False
                else:
                    with slock:
                        mdl["active"] = False
            elif mdl["count"] <= 0:
                mdl["last_req"] = time.time()
                with mlock:
                    mav.mav.mission_request_list_send(target["sys"], target["comp"])
            else:
                missing = [i for i in range(mdl["count"]) if i not in mdl["items"]]
                if missing:
                    request_mission_item(missing[0])
        time.sleep(2)


def driver():
    """Dead-man RC-override watchdog. Only overrides in MANUAL/STEERING while armed
    and while a fresh drive command exists (<0.6s). Otherwise neutralises then releases."""
    over = False
    while True:
        now = time.time()
        with slock:
            armed = state["link"]["armed"]
            mode = state["link"]["mode"]
        ts, tc = target["sys"], target["comp"]
        active = armed and mode in ("MANUAL", "STEERING") and (now - drive["ts"] < 0.6)
        with slock:
            state["dbg"] = {"age": round(now - drive["ts"], 2) if drive["ts"] else 999,
                            "active": active, "steer": drive["steer"], "throttle": drive["throttle"]}
        try:
            with mlock:
                if active:
                    mav.mav.rc_channels_override_send(
                        ts, tc, pwm(drive["steer"]), 0, pwm(drive["throttle"]), 0, 0, 0, 0, 0)
                    over = True
                elif over:
                    # Keep sending NEUTRAL, never fully release. Releasing the override looks
                    # like RC loss to ArduPilot (no receiver fitted) -> failsafe -> mode drops
                    # out of MANUAL and the drive buttons grey out. Neutral = stopped + RC alive.
                    mav.mav.rc_channels_override_send(ts, tc, 1500, 0, 1500, 0, 0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.05)   # 20 Hz


# ----- Bumper GPIO-läsning + flyktmanöver -----
def bumper_reader():
    """Läser vänster/höger mikroswitch (GPIO23/24) och uppdaterar state["bumper"].

    NC + intern pull-up => fail-safe: gpiozero Button(pull_up=True).is_pressed är
    True när pinnen är LÅG (switch sluten mot GND) = INTE ikörd. Träff = pinnen HÖG
    (switch öppen ELLER kabelbrott) => is_pressed False. Alltså hit = not is_pressed.
    Debounce: en råläsning måste vara stabil BUMPER_DEBOUNCE_S innan den committas."""
    if not GPIO_OK:
        with slock:
            state["bumper"]["ok"] = False
            push_msg(3, "Bumper: gpiozero saknas (%s) — pip install gpiozero lgpio" % GPIO_ERR)
        return
    try:
        left_btn = Button(GPIO_LEFT, pull_up=True)
        right_btn = Button(GPIO_RIGHT, pull_up=True)
    except Exception as e:      # noqa: BLE001
        with slock:
            state["bumper"]["ok"] = False
            push_msg(3, "Bumper: kunde inte öppna GPIO23/24 (%s)" % e)
        return
    with slock:
        state["bumper"]["ok"] = True
        push_msg(6, "Bumper: GPIO23/24 aktiva (auto-flykt PÅ som standard — flykt kräver armad+AUTO/GUIDED)")
    raw = {"left": None, "right": None}         # senast sedda råläsning
    since = {"left": 0.0, "right": 0.0}         # när råläsningen senast ändrades
    while True:
        now = time.time()
        cur = {"left": not left_btn.is_pressed, "right": not right_btn.is_pressed}
        for side in ("left", "right"):
            if cur[side] != raw[side]:
                raw[side] = cur[side]
                since[side] = now
        with slock:
            for side in ("left", "right"):
                if now - since[side] >= BUMPER_DEBOUNCE_S and state["bumper"][side] != raw[side]:
                    state["bumper"][side] = raw[side]
        time.sleep(0.02)        # 50 Hz


def _drive_refresh(steer, throttle):
    """Sätt drive-dicten som webb-joysticken gör, så driver-tråden håller override."""
    drive["steer"] = clamp(steer)
    drive["throttle"] = clamp(throttle)
    drive["ts"] = time.time()


def _signed_delta(a, b):
    """Minsta signerade a->b-vinkel i (-180, 180]."""
    return (b - a + 540) % 360 - 180


def _fusion_sectors():
    """Färsk fusionsbild (72 sektorer, cm/None, index 0 = rakt fram, medurs) eller None."""
    with slock:
        d = fusion_live["data"]
        age = fusion_live["age"]
    if d and age <= 2.0:
        s = d.get("sectors")
        if s and len(s) >= 72:
            return s
    return None


def _free_angle(side):
    """Vald svängvinkel bort från hindret. side=+1 höger, -1 vänster.

    Returnerar (signerad_vinkel, frihet): vinkel i grader (+ höger / − vänster).
    Söker 10°..170° ut från nosen på flyktsidan. Väljer en FRI sektor (avstånd
    >= CLEAR_CM eller tom) så nära DEFAULT_ESCAPE_ANGLE som möjligt — så att en
    öppen sida ger en rejäl sväng (~60°) i stället för minsta möjliga. Är hela
    sidan blockerad tas den absolut friaste sektorn. DEFAULT_ESCAPE_ANGLE utan fusion.

    ⚠️ Tidigare togs "störst avstånd, oavgjort → minsta vinkel", vilket gjorde att
    en öppen sida (alla sektorer fria) alltid gav 10° — därav de för små svängarna."""
    sectors = _fusion_sectors()
    if not sectors:
        return side * DEFAULT_ESCAPE_ANGLE, -1.0
    clear = []                                  # vinklar (grader) som är fria att svänga in i
    best_i, best_free = None, -1.0
    for i in range(2, 35):                      # 10°..170° från nosen
        idx = i if side > 0 else (72 - i) % 72  # höger = index framåt, vänster = bakåt
        d = sectors[idx]
        free = 1e9 if d is None else d
        if free > best_free:
            best_free, best_i = free, i
        if d is None or d >= CLEAR_CM:
            clear.append(i * 5.0)
    if clear:                                   # närmast önskad vinkel bland de fria
        ang = min(clear, key=lambda a: abs(a - DEFAULT_ESCAPE_ANGLE))
        return side * ang, best_free
    return side * (best_i * 5.0 if best_i else DEFAULT_ESCAPE_ANGLE), best_free


def escape_controller():
    """Tillståndsmaskin för flykten: REVERSE -> TURN -> RESUME -> (cooldown).

    Kör bara armad + AUTO/GUIDED + bumper enable (eller ett bänktest via
    esc_ctl["test_side"]). Operatörsingripande, disarm eller anti-thrash avbryter.
    Driver rovern via drive-dicten + do_mode, exakt som webb-joysticken."""
    esc = {"phase": "IDLE", "t0": 0.0, "start_ts": 0.0, "side": 1, "hit": "V",
           "resume_mode": "AUTO", "clear_t": None, "turn_start": 0.0, "turn_target": 0.0}
    history = []            # starttider för flykter (anti-thrash-fönster)
    cooldown_until = 0.0

    def publish():
        with slock:
            state["escape"]["active"] = esc["phase"] != "IDLE"
            state["escape"]["phase"] = esc["phase"]
            state["escape"]["count"] = len(history)

    def to_idle(msg=None, sev=5):
        _drive_refresh(0.0, 0.0)
        esc["phase"] = "IDLE"
        esc["clear_t"] = None
        if msg:
            with slock:
                push_msg(sev, msg)
        publish()

    while True:
        now = time.time()
        with slock:
            armed = state["link"]["armed"]
            mode = state["link"]["mode"]
            enable = state["bumper"]["enable"]
            left = state["bumper"]["left"]
            right = state["bumper"]["right"]
        active = esc["phase"] != "IDLE"

        # --- globala avbrott på alla aktiva faser ---
        if active:
            if not armed:
                to_idle("Flykt avbruten: disarmerad")
                time.sleep(0.05); continue
            if esc_ctl["operator_ts"] > esc["start_ts"]:
                to_idle("Flykt avbruten: operatörsingripande")
                time.sleep(0.05); continue

        # --- IDLE: leta trigger ---
        if esc["phase"] == "IDLE":
            test_side = esc_ctl["test_side"]
            hit_left = hit_right = False
            bench = False
            if test_side in ("left", "right"):
                esc_ctl["test_side"] = None
                hit_left, hit_right, bench = (test_side == "left"), (test_side == "right"), True
            elif enable and armed and mode in ESCAPE_MODES and (left or right) and now >= cooldown_until:
                history[:] = [t for t in history if now - t < WINDOW_S]
                if len(history) >= MAX_ESCAPES:
                    do_mode("HOLD")
                    with slock:
                        push_msg(2, "Bumper: rovern sitter fast (>%d flykter/%ds) — HOLD + stopp"
                                 % (MAX_ESCAPES, int(WINDOW_S)))
                    cooldown_until = now + WINDOW_S
                    history[:] = []
                else:
                    hit_left, hit_right = left, right
            if hit_left or hit_right:
                # Riktning: träff vänster -> sväng HÖGER (+1), träff höger -> VÄNSTER (-1).
                # Båda (eller bänktest utan tydlig sida) -> mot den friaste sidan.
                if hit_left and not hit_right:
                    side = 1
                elif hit_right and not hit_left:
                    side = -1
                else:
                    side = 1 if _free_angle(1)[1] >= _free_angle(-1)[1] else -1
                esc.update(phase="REVERSE", t0=now, start_ts=now, side=side,
                           hit=("V" if hit_left else "H"), clear_t=None,
                           resume_mode=(mode if mode in ESCAPE_MODES else "AUTO"))
                history.append(now)
                do_mode("MANUAL")
                _drive_refresh(0.0, REVERSE_THROTTLE)
                with slock:
                    push_msg(5, "Bumper %s%s -> flykt: backar" %
                             (esc["hit"], " (test)" if bench else ""))
                publish()
            else:
                publish()
                time.sleep(0.05); continue

        # --- REVERSE: backa tills båda switcharna släppt (eller tak) ---
        if esc["phase"] == "REVERSE":
            _drive_refresh(0.0, REVERSE_THROTTLE)
            if not left and not right:
                if esc["clear_t"] is None:
                    esc["clear_t"] = now
            else:
                esc["clear_t"] = None
            cleared = esc["clear_t"] is not None and now - esc["clear_t"] >= CLEAR_MARGIN_S
            elapsed = now - esc["t0"]
            # Backa alltid minst REVERSE_MIN_S (kom verkligen loss), sedan tills switcharna
            # släppt CLEAR_MARGIN_S — men aldrig längre än REVERSE_MAX_S.
            if (cleared and elapsed >= REVERSE_MIN_S) or elapsed >= REVERSE_MAX_S:
                esc["turn_target"], _free = _free_angle(esc["side"])
                with slock:
                    esc.update(phase="TURN", t0=now, turn_start=state["motion"]["heading"])
                    push_msg(5, "Flykt: svänger %s mot fri vinkel %d°" %
                             ("höger" if esc["side"] > 0 else "vänster",
                              round(abs(esc["turn_target"]))))
                publish()

        # --- TURN: sluten-loop-sväng mot målvinkeln (eller tak) ---
        elif esc["phase"] == "TURN":
            _drive_refresh(esc["side"] * TURN_STEER, TURN_THROTTLE)
            with slock:
                hdg = state["motion"]["heading"]
            progress = _signed_delta(esc["turn_start"], hdg) * esc["side"]
            if progress >= abs(esc["turn_target"]) or now - esc["t0"] >= TURN_MAX_S:
                _drive_refresh(0.0, 0.0)
                esc.update(phase="RESUME", t0=now)
                publish()

        # --- RESUME: neutralisera, återgå till autonomt läge ---
        elif esc["phase"] == "RESUME":
            _drive_refresh(0.0, 0.0)
            if now - esc["t0"] >= 0.3:      # håll neutral en stund innan lägesbytet
                do_mode(esc["resume_mode"])
                cooldown_until = now + COOLDOWN_S
                to_idle("Flykt klar -> %s" % esc["resume_mode"], sev=6)

        time.sleep(0.05)        # 20 Hz


def do_arm(a):
    with mlock:
        mav.mav.command_long_send(target["sys"], target["comp"],
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                  1 if a else 0, 0, 0, 0, 0, 0, 0)


def do_set_wp(seq):
    """Jump the AUTO mission to a given waypoint (what Mission Planner's 'Set WP' does)."""
    try:
        seq = int(seq)
    except (TypeError, ValueError):
        return
    if seq < 0:
        return
    with mlock:
        mav.mav.mission_set_current_send(target["sys"], target["comp"], seq)
    with slock:
        push_msg(6, "Sätter aktuell waypoint till %d" % seq)


def do_mode(name):
    cm = ROVER_MODES.get(name)
    if cm is None:
        return
    with mlock:
        mav.mav.set_mode_send(target["sys"],
                              mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, cm)


def do_shutdown():
    """Stäng av companion-datorn snyggt.

    Finns för att ett hårt strömavbrott på en körande Pi är den SD-korruption
    som står som felmod i plandokumentet — och sedan V3 skriver dessutom
    roverav kontinuerligt till journalen.

    VÄGRAS NÄR FORDONET ÄR ARMERAT. Går Pi:n ned tar den med sig mavproxy, och
    därmed både videon, telemetrin och RC-override-vägen som den manuella
    körningen använder. Att kunna göra det mitt under körning vore att bygga in
    en fjärrstyrd förlust av kontrollen.
    """
    with slock:
        armed = state["link"]["armed"]
    if armed:
        with slock:
            push_msg(4, "Avstängning nekad: rovern är ARMERAD — disarmera först")
        return False
    with slock:
        push_msg(5, "Stänger av companion-datorn ...")
    subprocess.Popen(["sudo", "shutdown", "-h", "now"])
    return True


# ----- Video config (rewrite mediamtx.yml; mediamtx hot-reloads) -----
VIDEO_CFG_PATH = os.path.join(HERE, "mediamtx.yml")
ALLOWED_RES = {(640, 480), (1280, 960), (1640, 1232)}
ALLOWED_FPS = {5, 10, 15, 30, 60}
video = {"w": 1280, "h": 960, "fps": 30}


def video_bitrate():
    # scale target bitrate with pixels*fps so lower settings genuinely use less data
    return max(150000, min(8000000, int(video["w"] * video["h"] * video["fps"] * 0.08)))


def write_video_config():
    br = video_bitrate()
    cfg = ("logLevel: info\napi: yes\npaths:\n  rover:\n    source: rpiCamera\n"
           "    rpiCameraMode: 1640:1232:10:P\n"
           f"    rpiCameraWidth: {video['w']}\n    rpiCameraHeight: {video['h']}\n"
           f"    rpiCameraFPS: {video['fps']}\n    rpiCameraBitrate: {br}\n"
           "    rpiCameraIDRPeriod: 15\n")
    with open(VIDEO_CFG_PATH, "w") as f:
        f.write(cfg)
    with slock:
        state["video"] = {"w": video["w"], "h": video["h"], "fps": video["fps"], "bitrate": br}


def set_video(w, h, fps):
    if (w, h) not in ALLOWED_RES or fps not in ALLOWED_FPS:
        return
    video.update(w=w, h=h, fps=fps)
    write_video_config()            # mediamtx picks it up via hot-reload
    with slock:
        push_msg(6, "Video: %dx%d @ %d fps (~%.1f Mbit/s)" % (w, h, fps, video_bitrate() / 1e6))


# ----- Mission download (MAVLink mission protocol) -----
def request_mission_item(seq):
    mdl["last_req"] = time.time()
    with mlock:
        mav.mav.mission_request_int_send(target["sys"], target["comp"], seq, mdl["mtype"])


def _start_dl(mtype):
    with slock:
        mdl.update(active=True, mtype=mtype, count=-1, items={}, last_req=time.time(), tries=0)
    with mlock:
        mav.mav.mission_request_list_send(target["sys"], target["comp"], mtype)


def start_mission_read():
    _start_dl(MT_MISSION)          # chains on to fence, then rally


def parse_fence(by_seq):
    """ArduPilot fence items -> drawable shapes (polygons, circles, return point)."""
    shapes, seqs, i = [], sorted(by_seq), 0
    while i < len(seqs):
        it = by_seq[seqs[i]]
        cmd = it.command
        # MAV_CMD: 5000 RETURN_POINT, 5001/5002 POLYGON_VERTEX incl/excl (param1 = vertex count),
        #          5003/5004 CIRCLE incl/excl (param1 = radius)
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


def finalize_mission():
    mt = mdl["mtype"]
    with mlock:
        mav.mav.mission_ack_send(target["sys"], target["comp"], 0, mt)

    if mt == MT_MISSION:
        items, jumps = [], []
        for s in sorted(mdl["items"]):
            it = mdl["items"][s]
            if it.command == mavutil.mavlink.MAV_CMD_DO_JUMP:   # 177: no coords, param1 = target
                jumps.append({"seq": s, "target": int(it.param1), "repeat": int(it.param2)})
            elif it.frame in GLOBAL_FRAMES and (it.x or it.y):
                items.append({"seq": s, "lat": it.x / 1e7, "lon": it.y / 1e7, "cmd": it.command})
        with slock:
            mission["items"] = items
            mission["jumps"] = jumps
        _start_dl(MT_FENCE)
    elif mt == MT_FENCE:
        shapes = parse_fence(mdl["items"])
        with slock:
            mission["fence"] = shapes
        _start_dl(MT_RALLY)
    else:                                                        # MT_RALLY -> done
        rally = [{"seq": s, "lat": mdl["items"][s].x / 1e7, "lon": mdl["items"][s].y / 1e7}
                 for s in sorted(mdl["items"]) if (mdl["items"][s].x or mdl["items"][s].y)]
        with slock:
            mission["rally"] = rally
            mission["version"] += 1
            mdl["active"] = False
            extra = "".join(" + DO_JUMP -> WP %d" % j["target"] for j in mission["jumps"])
            push_msg(6, "Läst: %d waypoints%s, %d fence-former, %d rally"
                     % (len(mission["items"]), extra, len(mission["fence"]), len(rally)))


# ----- WiFi / nätverk (NetworkManager via nmcli) -----
# Bakgrund: Pi:n nådde inte en mobil 4G-router på annan plats för att BARA
# hemnätet var sparat — NetworkManager autoansluter bara till SPARADE nät. Det
# här låter operatören se/lägga till/välja nät från webben: lägg till 4G-nätet
# HEMMA (på tailnet) innan resan, så autoansluter Pi:n där. Skrivning kräver
# rättigheter -> sudo (samma mönster som do_shutdown; axel har NOPASSWD:ALL).
WIFI_IFACE = "wlan0"


def _run(cmd, timeout=20):
    """Kör ett kommando, returnera (rc, stdout, stderr). Kastar aldrig."""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except Exception as e:
        return 1, "", str(e)


def _nmcli(args, sudo=False, timeout=20):
    return _run((["sudo", "nmcli"] if sudo else ["nmcli"]) + args, timeout)


def _terse(line):
    """Dela en nmcli -t-rad på OESCAPAD ':' (nmcli escapar ':' i värden som '\\:')."""
    out, cur, esc = [], "", False
    for ch in line:
        if esc:
            cur += ch; esc = False
        elif ch == "\\":
            esc = True
        elif ch == ":":
            out.append(cur); cur = ""
        else:
            cur += ch
    out.append(cur)
    return out


def wlan_ip():
    _rc, out, _e = _run(["ip", "-4", "-o", "addr", "show", "dev", WIFI_IFACE])
    for part in out.replace("/", " ").split():
        bits = part.split(".")
        if len(bits) == 4 and all(b.isdigit() for b in bits) and not part.startswith("169.254."):
            return part
    return None


def wifi_current():
    """(ssid, signal 0-100) för aktivt wlan0-wifi, annars (None, None)."""
    _rc, out, _e = _nmcli(["-t", "-f", "ACTIVE,SSID,SIGNAL", "device", "wifi"])
    for line in out.splitlines():
        f = _terse(line)
        if f and f[0] == "yes":
            sig = f[2] if len(f) > 2 and f[2].isdigit() else None
            return (f[1] or None), (int(sig) if sig else None)
    return None, None


def wifi_active_conn():
    """NAMN på NM-anslutningen som är aktiv på wlan0 (för revert + forget-skydd)."""
    _rc, out, _e = _nmcli(["-t", "-f", "NAME,DEVICE", "connection", "show", "--active"])
    for line in out.splitlines():
        f = _terse(line)
        if len(f) >= 2 and f[1] == WIFI_IFACE:
            return f[0]
    return None


def wifi_saved():
    """Sparade wifi-anslutningar: [{name, autoconnect}]."""
    _rc, out, _e = _nmcli(["-t", "-f", "NAME,TYPE,AUTOCONNECT", "connection", "show"])
    res = []
    for line in out.splitlines():
        f = _terse(line)
        if len(f) >= 2 and "wireless" in f[1]:
            res.append({"name": f[0], "autoconnect": len(f) > 2 and f[2] == "yes"})
    return res


def wifi_scan():
    """Nät i närheten: [{ssid, signal, security}] deduperat på ssid (starkast kvar)."""
    _rc, out, _e = _nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list"], timeout=15)
    best = {}
    for line in out.splitlines():
        f = _terse(line)
        if not f or not f[0]:
            continue
        ssid, sig = f[0], (int(f[1]) if len(f) > 1 and f[1].isdigit() else 0)
        if ssid not in best or sig > best[ssid]["signal"]:
            best[ssid] = {"ssid": ssid, "signal": sig, "security": (f[2] if len(f) > 2 else "") or "öppet"}
    return sorted(best.values(), key=lambda x: -x["signal"])


def wifi_internet_ok(timeout=3):
    """Har wlan0 IP OCH internet? (avgör auto-återgången)."""
    if not wlan_ip():
        return False
    for host in (("1.1.1.1", 53), ("8.8.8.8", 53)):
        try:
            socket.create_connection(host, timeout).close()
            return True
        except Exception:
            continue
    return False


def wifi_add(ssid, password):
    """Skapar en sparad wifi-anslutning. Kopplar INTE om (priority 0, add
    aktiverar inte av sig själv). Autoconnect på => ansluter när nätet är i
    räckvidd senare, vilket är hela poängen för 4G-fallet."""
    ssid = (ssid or "").strip()
    if not ssid:
        return False, "SSID saknas"
    # Ersätt ev. gammal med samma namn (annars 'already exists') — men ALDRIG
    # den aktiva anslutningen, det skulle fälla nätet vi sitter på.
    if ssid != wifi_active_conn():
        _nmcli(["connection", "delete", ssid], sudo=True)
    args = ["connection", "add", "type", "wifi", "con-name", ssid,
            "ifname", WIFI_IFACE, "ssid", ssid,
            "connection.autoconnect", "yes", "connection.autoconnect-priority", "0"]
    if password:
        args += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password]
    rc, out, err = _nmcli(args, sudo=True, timeout=25)
    if rc == 0:
        with slock:
            push_msg(6, "WiFi-nät '%s' sparat (autoansluter när i räckvidd)" % ssid)
        return True, None
    return False, (err.strip() or out.strip() or "nmcli-fel")


def wifi_forget(name):
    name = (name or "").strip()
    if not name:
        return False, "namn saknas"
    if name == wifi_active_conn():
        return False, "Kan inte radera det AKTIVA nätet (du sitter på det)"
    rc, _out, err = _nmcli(["connection", "delete", name], sudo=True)
    if rc == 0:
        with slock:
            push_msg(6, "WiFi-nät '%s' borttaget" % name)
        return True, None
    return False, (err.strip() or "nmcli-fel")


def wifi_connect_revert(name):
    """Byt aktivt nät med AUTO-ÅTERGÅNG: verifiera internet inom 25 s, annars gå
    tillbaka till föregående nät. Körs i egen tråd så webbsvaret hinner ut innan
    wlan0 kopplas om. Servern överlever nätbytet (lyssnar på 0.0.0.0); operatören
    når Pi:n på samma tailnet-IP så snart internet är uppe igen."""
    prev = wifi_active_conn()
    with slock:
        push_msg(5, "Byter WiFi till '%s' … auto-återgång om det misslyckas" % name)
    _nmcli(["connection", "up", name], sudo=True, timeout=35)
    ok = False
    deadline = time.time() + 25
    while time.time() < deadline:
        if wifi_internet_ok():
            ok = True
            break
        time.sleep(3)
    if ok:
        with slock:
            push_msg(6, "WiFi bytt till '%s' — internet OK" % name)
    elif prev and prev != name:
        _nmcli(["connection", "up", prev], sudo=True, timeout=35)
        with slock:
            push_msg(3, "'%s' gav inget internet — återgick till '%s'" % (name, prev))
    else:
        with slock:
            push_msg(3, "'%s' gav inget internet, inget föregående nät att återgå till" % name)


app = FastAPI()


@app.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()

    overlay = {"on": False}

    async def sender():
        last_mv = -1
        tick = 0
        while True:
            tick += 1
            with slock:
                snap = json.dumps(state)
                mv = mission["version"]
                mpayload = json.dumps({"type": "mission", "version": mv,
                                       "items": mission["items"], "jumps": mission["jumps"],
                                       "fence": mission["fence"],
                                       "rally": mission["rally"]}) if mv != last_mv else None
            await websocket.send_text(snap)
            if mpayload is not None:
                await websocket.send_text(mpayload)
                last_mv = mv
            # CV-överlägget är ~2 kB per bildruta och skickas därför BARA när
            # klienten bett om det, och i 5 Hz i stället för 10. Över 4G är
            # skillnaden mot att alltid skicka ungefär 70 MB i timmen.
            if overlay["on"] and tick % 2 == 0:
                with slock:
                    d = av_live["data"]
                    age = av_live["age"]
                    ld = lidar_live["data"]
                    lage = lidar_live["age"]
                    fd = fusion_live["data"]
                    fage = fusion_live["age"]
                if d is not None:
                    await websocket.send_text(json.dumps(
                        {"type": "av", "age": round(age, 2), **d}))
                # Lidarn är en egen sensor och skickas som eget lager. Den ~1,5 kB
                # dist_cm-arrayen går bara ut när överlägget är på, i 5 Hz som
                # av-datan — samma sparsamhet mot 4G-länken.
                if ld is not None:
                    await websocket.send_text(json.dumps(
                        {"type": "lidar", "age": round(lage, 2), **ld}))
                # Fusionen = det som FAKTISKT skickas till ArduPilot (röda prickar).
                if fd is not None:
                    await websocket.send_text(json.dumps(
                        {"type": "fusion", "age": round(fage, 2), **fd}))
            await asyncio.sleep(0.1)

    task = asyncio.create_task(sender())
    try:
        while True:
            m = json.loads(await websocket.receive_text())
            c = m.get("cmd")
            if c == "drive":
                drive["steer"] = clamp(float(m.get("steer", 0)))
                drive["throttle"] = clamp(float(m.get("throttle", 0)))
                drive["ts"] = time.time()
                esc_ctl["operator_ts"] = drive["ts"]   # operatören avbryter en pågående flykt
            elif c == "arm":
                do_arm(bool(m.get("value")))
                esc_ctl["operator_ts"] = time.time()
            elif c == "mode":
                do_mode(str(m.get("value")))
                esc_ctl["operator_ts"] = time.time()
            elif c == "bumper_enable":
                with slock:
                    state["bumper"]["enable"] = bool(m.get("value"))
                    push_msg(6, "Auto-flykt %s" % ("PÅ" if state["bumper"]["enable"] else "av"))
            elif c == "escape_test":
                # Bänktest (hjul upp): tvinga en virtuell träff och kör hela sekvensen.
                esc_ctl["test_side"] = "right" if str(m.get("side")) == "right" else "left"
            elif c == "set_wp":
                do_set_wp(m.get("value"))
            elif c == "read_mission":
                start_mission_read()
            elif c == "set_overlay":
                overlay["on"] = bool(m.get("value"))
            elif c == "set_video":
                set_video(int(m.get("w")), int(m.get("h")), int(m.get("fps")))
            elif c == "lidar_motor":
                set_lidar_motor(m.get("value"))
            elif c == "shutdown":
                # Kräver en uttrycklig nyckel, inte bara kommandonamnet. Ett
                # tappat eller upprepat meddelande ska inte kunna släcka Pi:n.
                if m.get("confirm") == "SHUTDOWN":
                    do_shutdown()
    except WebSocketDisconnect:
        pass
    finally:
        task.cancel()


@app.get("/stereo.jpg")
def stereo_jpg():
    """Stereotjänstens live-bild.

    Finns för att videopanelen är SVART under en stereokörning: mediamtx måste
    stoppas för att stereo ska få cam0, så RTSP-strömmen existerar inte då.
    Bilden är alltså operatörens enda vy, och den kommer ur samma arrayer som
    hinderbeslutet — inte ur en separat kamerakedja som skulle kunna visa något
    annat än det tjänsten räknar på.

    Går som vanlig HTTP i stället för över websocketen: ~25 kB per bild i 2 Hz
    är för mycket för telemetrikanalen, och webbläsaren cachar och hämtar om
    den själv utan att servern behöver hålla reda på vem som tittar.
    """
    try:
        with open(STEREO_IMG_PATH, "rb") as f:
            data = f.read()
    except OSError:
        return Response(status_code=404)
    return Response(content=data, media_type="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.get("/wifi/list")
def api_wifi_list():
    ssid, sig = wifi_current()
    return JSONResponse({
        "current": {"ssid": ssid, "signal": sig, "ip": wlan_ip(), "name": wifi_active_conn()},
        "saved": wifi_saved(),
        "available": wifi_scan(),
    })


@app.post("/wifi/add")
async def api_wifi_add(req: Request):
    b = await req.json()
    ok, err = wifi_add(b.get("ssid"), b.get("password"))
    return JSONResponse({"ok": ok, "error": err}, status_code=200 if ok else 400)


@app.post("/wifi/forget")
async def api_wifi_forget(req: Request):
    b = await req.json()
    ok, err = wifi_forget(b.get("name"))
    return JSONResponse({"ok": ok, "error": err}, status_code=200 if ok else 400)


@app.post("/wifi/connect")
async def api_wifi_connect(req: Request):
    b = await req.json()
    name = (b.get("name") or "").strip()
    if not name:
        return JSONResponse({"ok": False, "error": "namn saknas"}, status_code=400)
    # Egen tråd: webbsvaret ska ut INNAN wlan0 kopplas om (annars tappas svaret).
    threading.Thread(target=wifi_connect_revert, args=(name,), daemon=True).start()
    return JSONResponse({"ok": True, "message": "Byte startat — auto-återgång om det misslyckas"})


app.mount("/", StaticFiles(directory=os.path.join(HERE, "static"), html=True), name="static")


if __name__ == "__main__":
    write_video_config()   # ensure mediamtx.yml matches defaults (mediamtx hot-reloads)
    for fn in (mav_reader, pi_reader, driver, av_reader, lidar_reader, fusion_reader,
               bumper_reader, escape_controller):
        threading.Thread(target=fn, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=WEB_PORT, log_level="warning")
