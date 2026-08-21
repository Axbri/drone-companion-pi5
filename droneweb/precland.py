"""Precisionslandnings-CV för droneweb (Steg 2).

Två detektorer på lores 640x480:
  - ArUco (DICT_4X4_50, ID 0)  -> exakt mål när nära
  - orange cirkel (HSV)         -> mål från hög höjd

Returnerar målets pixelposition + vilken detektor, och räknar om pixel->vinkel
(angle_x/angle_y i rad) för LANDING_TARGET. Fas-logik + MAVLink-sändning sker i
app.py (som har AGL från rangefinder.py och mavlink-anslutningen).

Modulen är fristående testbar:  python precland.py <bild> [målstorlek_px]
"""
import math
import os
import queue
import threading
import time

import cv2
import numpy as np

# ---- konfig ------------------------------------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_ID = 0

# Röd-orange platta i HSV (OpenCV H 0-180). Uppmätt i sol: H≈0, S≈240, V≈214.
# H ligger nära 0 → hue-wrap → två H-band (nära 0 ELLER nära 180). Hög S/V ger
# specificitet mot grönt gräs (H~40-80). Kalibrerat 2026-07-30, tunas vid behov.
ORANGE_H_LO = 12             # matcha H <= 12 ...
ORANGE_H_HI = 165            # ... ELLER H >= 165 (röd-wrap)
ORANGE_S_MIN = 130
ORANGE_V_MIN = 90
COLOR_MIN_AREA = 80          # px^2, minsta blob (~10 px diameter)

# IMX219 full-FOV vid CV-upplösningen (approx; förfinas med kalibrering)
CV_W, CV_H = 640, 480
HFOV_DEG, VFOV_DEG = 62.2, 48.8
FX = (CV_W / 2) / math.tan(math.radians(HFOV_DEG) / 2)
FY = (CV_H / 2) / math.tan(math.radians(VFOV_DEG) / 2)
CX, CY = CV_W / 2.0, CV_H / 2.0

# Kamerakalibrering: ArUco-markörens verkliga sidlängd (svarta fyrkanten), meter.
# MÄT den utskrivna markören och sätt rätt värde — hela kalibreringen beror på detta.
# 0.30 = landnings-markörens ArUco (flyg). Bänk-test-markören var 0.145.
MARKER_M = 0.30
ASSUMED_F = FX               # antagen brännvidd (px) att jämföra uppmätt mot

PHASE_COLOR, PHASE_ARUCO, PHASE_RTK = "COLOR", "ARUCO", "RTK-HOLD"

# Fast exponering under CV-loopen (mot motion blur på höjd — Fynd 1). Kort slutartid
# fryser rörelsen så rutan blir skarp trots kameravibration; auto-exponering (default)
# väljer ibland lång slutartid i starkt ljus och smetar ut markören. Tuna 1500-2500 µs
# efter ljuset; höj gain om för mörkt (men mycket gain = brus som stör detekteringen).
# Återgår till auto när precland avaktiveras (för normal FPV-video).
CV_EXPOSURE_US = 2000
CV_GAIN = 2.0

# Styrskala: skalar vinkelfelet som skickas till ArduPilot innan LANDING_TARGET.
# 1.0 = fullt fel (som kameran ser det). <1.0 = mildare korrektioner (mot översläng vid
# latens) — drönaren konvergerar ändå geometriskt om landningshastigheten är låg.
# Justeras live i webben. Detta är i praktiken det precland-gain ArduPilot saknar.
CMD_SCALE_DEFAULT = 1.0


class Target:
    __slots__ = ("u", "v", "source", "aruco_id", "radius", "corners")

    def __init__(self, u, v, source, aruco_id=None, radius=None, corners=None):
        self.u, self.v, self.source = u, v, source
        self.aruco_id, self.radius, self.corners = aruco_id, radius, corners

    def angles(self):
        """(angle_x, angle_y) i rad rel. kameraxeln. +x=höger, +y=nedåt i bilden.
        Kamera-monteringsmappning till kroppsframe (BODY_FRD) görs i app.py."""
        ax = math.atan2(self.u - CX, FX)
        ay = math.atan2(self.v - CY, FY)
        return ax, ay

    def offset_norm(self):
        """Normaliserad offset [-1,1] från bildcentrum (för UI)."""
        return (self.u - CX) / (CV_W / 2), (self.v - CY) / (CV_H / 2)

    def px_size(self):
        """Genomsnittlig sidlängd (px) för ArUco-fyrkanten, ur de fyra hörnen.
        För kamerakalibrering: f = px_size * avstånd / markör_verklig_storlek."""
        if self.corners is None:
            return None
        p = self.corners
        return sum(math.hypot(*(p[(i + 1) % 4] - p[i])) for i in range(4)) / 4.0

    def yaw_error_rad(self):
        """Vinkelfel (rad) mellan markörens tryckta "upp" (samma vektor som
        riktningspilen i annotate()) och bildens "upp" (=nosen, se PLND_YAW_ALIGN-
        kommentaren nedan). 0 = pilen pekar rakt upp. Positivt = pilen lutar mot
        höger i bilden. Bara giltigt för ArUco (kräver corners)."""
        if self.corners is None:
            return None
        up = _marker_up_vector(self.corners)
        return math.atan2(up[0], -up[1])


def _marker_up_vector(corners):
    """Markörens tryckta "upp" i bildkoordinater: mitt på topp-kanten minus mitt på
    botten-kanten. corners[0..3] är medsols f.o.m. markörens tryckta topp-vänster-
    hörn, så vektorn följer markörens tryckta orientering när den roterar i bild."""
    p = corners
    return ((p[0] + p[1]) - (p[2] + p[3])) / 2.0


class PrecLandDetector:
    def __init__(self):
        params = cv2.aruco.DetectorParameters()
        self._aruco = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT), params
        )
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def detect_aruco(self, gray):
        corners, ids, _ = self._aruco.detectMarkers(gray)
        if ids is None:
            return None
        for c, i in zip(corners, ids.flatten()):
            if i == ARUCO_ID:
                pts = c.reshape(4, 2)
                u, v = pts.mean(axis=0)
                return Target(float(u), float(v), "aruco", aruco_id=int(i), corners=pts)
        return None

    def detect_color(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, ORANGE_S_MIN, ORANGE_V_MIN], np.uint8),
                              np.array([ORANGE_H_LO, 255, 255], np.uint8))
        m2 = cv2.inRange(hsv, np.array([ORANGE_H_HI, ORANGE_S_MIN, ORANGE_V_MIN], np.uint8),
                              np.array([179, 255, 255], np.uint8))
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < COLOR_MIN_AREA:
            return None
        (u, v), r = cv2.minEnclosingCircle(c)
        return Target(float(u), float(v), "color", radius=float(r))

    def annotate(self, bgr, target, phase=None, agl=None):
        """Ritar bildanalys-grafiken (visas i webben)."""
        cv2.drawMarker(bgr, (int(CX), int(CY)), (120, 120, 120),
                       cv2.MARKER_CROSS, 20, 1)
        if target is not None:
            u, v = int(target.u), int(target.v)
            col = (0, 255, 0) if target.source == "aruco" else (0, 165, 255)
            if target.source == "color" and target.radius:
                cv2.circle(bgr, (u, v), int(target.radius), col, 2)
            if target.source == "aruco" and target.corners is not None:
                cv2.polylines(bgr, [target.corners.astype(np.int32)], True, col, 2)
                # Riktningspil: markörens tryckta "upp" (se _marker_up_vector).
                # Samma vektor används av Target.yaw_error_rad() för gir-inriktning.
                up = _marker_up_vector(target.corners)
                tip = (u + up[0] * 0.7, v + up[1] * 0.7)
                cv2.arrowedLine(bgr, (u, v), (int(round(tip[0])), int(round(tip[1]))),
                                col, 2, tipLength=0.35)
            cv2.line(bgr, (int(CX), int(CY)), (u, v), col, 2)
            cv2.circle(bgr, (u, v), 4, col, -1)
            ax, ay = target.angles()
            cv2.putText(bgr, "%s  ax=%.1f ay=%.1f deg" % (
                target.source.upper(), math.degrees(ax), math.degrees(ay)),
                (8, CV_H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        hud = []
        if phase:
            hud.append(phase)
        if agl is not None:
            hud.append("AGL %.2fm" % agl)
        if hud:
            cv2.putText(bgr, "  ".join(hud), (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return bgr


# ---- kamera→kropp-rotationen görs av ArduPilot, INTE här ---------------
# 2026-08-19, rotfelet hittat: vi transformerade bild→kropp HÄR (en tidigare
# image_to_body(), body_x=-ay_img/body_y=ax_img — bänktest bekräftade att DEN
# var geometriskt rätt) och skickade sedan resultatet som angle_x/angle_y i
# LANDING_TARGET. Men ArduPilots egen mottagare gör SAMMA rotation internt:
# AC_PrecLand_MAVLink::handle_msg() (github.com/ArduPilot/ardupilot,
# libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp) bygger siktlinjen som
#   vec_body_frd = { -tan(angle_y), tan(angle_x), 1.0 }   // (forward, right, down)
# dvs den FÖRVÄNTAR SIG rå bildvinkel (kamerans egna ax/ay), inte en redan
# kropps-roterad vinkel — och roterar själv till BODY_FRD. Vi roterade två
# gånger → nettoeffekt 90°, exakt matchande flygtestets fyra videobilder
# (mål fram→styrde höger, mål bak→vänster, mål vänster→fram, mål höger→bak;
# alla fyra stämmer med denna dubbelrotations-hypotes, testad i efterhand).
# Fix: skicka RÅ ax/ay direkt (nedan) — ingen egen kropps-transform.
# PLND_YAW_ALIGN ska vara 0 (bänktestat: kameran är monterad exakt som
# ArduPilot antar, bild-topp = nos, ingen fysisk vridning att kompensera för).

# ---- gir-inriktning mot markören (CONDITION_YAW) ------------------------
# ArduPilots LAND-läge (inkl. precision landing) läser auto_yaw.get_heading() varje
# styrcykel oavsett pilot-input (mode.cpp: land_run_horizontal_control() avslutas
# alltid med attitude_control->input_thrust_vector_heading(thrust, auto_yaw.get_
# heading())) — helt frikopplat från position/höjd-styrningen. MAV_CMD_CONDITION_YAW
# sätter auto_yaw i FIXED-läge mot en absolut heading med given grader/s, och FC:n
# rampar dit kontinuerligt själv (ingen omsändning krävs för att fortsätta vrida,
# se autoyaw.cpp) — drönaren vrider alltså SAMTIDIGT som den sjunker, stannar inte
# upp. Se PrecLandController._control_tick / mavlink.send_condition_yaw.
#
# Bänktestat 2026-08-20 (utan props/arm): roterade drönaren för hand ovanför en
# fast markör och jämförde beräknad mål-heading (yaw + YAW_ERR_SIGN*yaw_err_deg)
# före/efter — oförändrad (296.8° vs 295.8°) trots 90° handvridning, vilket
# bekräftar tecknet (rätt tecken → mål-heading oberoende av dronens aktuella
# riktning; fel tecken hade gett ~dubbla, motsatta utslaget). YAW_ERR_SIGN=1
# bekräftat korrekt för denna kameramontering. Verklig flygrotation (med props)
# EJ testad än — se PRECISION-LANDING.md för flygtestplan.
YAW_ERR_SIGN = 1


REC_DIR = "/home/axel/recordings"
CSV_HEADER = ("t,mode,armed,roll,pitch,yaw,heading,alt,agl,batt_v,batt_pct,"
              "fix,sats,phase,source,ox,oy,ax_deg,ay_deg,sent,scale,"
              "yaw_err_deg,yaw_cmd_deg,yaw_align\n")


def _f(v, nd=3):
    if v is None:
        return ""
    return ("%.*f" % (nd, v)) if isinstance(v, float) else str(v)


class PrecLandController:
    """Kör CV-loopen när precland är armerat ELLER inspelning pågår: väljer fas
    efter AGL, detekterar, skickar LANDING_TARGET (bara armerat, utom RTK-fasen),
    matar annoterad video, och spelar in synkad video (.avi) + datalogg (.csv)."""

    RATE_HZ = 40          # kontroll-loopens tak (matchar kamera-FPS). Tung JPEG-encode + inspelning
                          # körs i separat writer-tråd, så detekt+skick av LANDING_TARGET tightas
                          # (mot översläng/latens — Fynd 2). Pi 5, 2026-08-18: 40 Hz stabilt/repeterbart
                          # (39.7-40.2 Hz uppmätt över 18s). >40 (testat 60/90) gav samma latens men
                          # ostabil takt (30-53 Hz) pga GIL-delning med app.js webb-pollningen — inte
                          # kamera/ISP-bunden. 40 är alltså sweetspoten, inte en hård kamera-gräns.
    REC_FPS = 8           # AVI-fps-metadata (nominell). Verklig inspelningstakt är writer-begränsad;
                          # CSV:ns t-kolumn är den exakta tiden per ruta (ruta N = CSV-rad N).
    ARUCO_MAX_AGL = 3.5   # m — under detta föredras ArUco framför färg
    RTK_AGL = 0.5         # m — under detta: sluta skicka, RTK håller x/y
    _Q_MAX = 8            # writer-köns tak → drop-oldest (håller RAM nere på Pi 3A+)

    YAW_RATE_DEFAULT = 30.0        # deg/s, live-justerbar i webben
    YAW_RATE_MIN, YAW_RATE_MAX = 5.0, 90.0
    YAW_CMD_HZ = 4                 # CONDITION_YAW skickas mycket glesare än LANDING_TARGET;
                                    # auto_yaw rampar själv kontinuerligt mellan uppdateringarna

    def __init__(self, cam, tel, rangefinder=None):
        self.cam, self.tel, self.rf = cam, tel, rangefinder
        self.det = PrecLandDetector()
        self._armed = False
        self._recording = False
        self._thread = None
        self._tx = 0
        self._q = None       # writer-kö (skapas i _run)
        # CV-exponering (justeras live från webben för fältjustering)
        self._auto_exposure = False
        self._exposure_us = CV_EXPOSURE_US
        self._gain = CV_GAIN
        self._cmd_scale = CMD_SCALE_DEFAULT   # styrskala för LANDING_TARGET (live)
        self._yaw_align = False               # av = beter sig precis som innan denna funktion
        self._yaw_rate = self.YAW_RATE_DEFAULT
        self._last_yaw_tx = 0.0
        self._lat_ms = None                   # Pi-latens (kamera-fångst → skick), EWMA
        self._loop_hz = None                  # verklig loop-takt, EWMA
        self._last_tick = None
        self._st_lock = threading.Lock()
        self._status = {"phase": None, "source": None, "agl": None,
                        "offset": None, "tx": 0, "sent": False, "rec_file": None,
                        "calib": None, "latency_ms": None, "loop_hz": None,
                        "yaw_err_deg": None}

    @property
    def armed(self):
        return self._armed

    def _ensure_thread(self):
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def arm(self):
        self._armed = True
        self._ensure_thread()

    def disarm(self):
        self._armed = False

    def _apply_exposure(self):
        """Skjut aktuellt exponerings-läge till kameran. No-op om kameran ej är
        igång (camera.set_cv_exposure gardar) → gäller då vid nästa Aktivera."""
        if self._auto_exposure:
            self.cam.set_cv_exposure(False)
        else:
            self.cam.set_cv_exposure(True, self._exposure_us, self._gain)

    def exposure_status(self):
        return {"auto": self._auto_exposure, "exposure_us": self._exposure_us,
                "gain": round(self._gain, 1)}

    def set_cmd_scale(self, s):
        """Styrskala [0.1, 1.0] för LANDING_TARGET-vinklarna. Live."""
        self._cmd_scale = float(max(0.1, min(1.0, s)))
        return round(self._cmd_scale, 2)

    def yaw_align_status(self):
        return {"enabled": self._yaw_align, "rate_degs": round(self._yaw_rate, 1)}

    def set_yaw_align(self, enabled=None, rate_degs=None):
        """Av/på för girinriktning mot markören + vridhastighet (grader/s). Live.
        Av = ingen CONDITION_YAW skickas alls, dvs. exakt som innan denna funktion."""
        if enabled is not None:
            self._yaw_align = bool(enabled)
        if rate_degs is not None:
            self._yaw_rate = float(max(self.YAW_RATE_MIN, min(self.YAW_RATE_MAX, rate_degs)))
        return self.yaw_align_status()

    def set_exposure(self, auto=None, exposure_us=None, gain=None):
        """Ställ CV-exponering live från webben (fältjustering). Klämmer värden och
        applicerar direkt om CV-loopen kör; annars gäller de vid nästa Aktivera."""
        if auto is not None:
            self._auto_exposure = bool(auto)
        if exposure_us is not None:
            self._exposure_us = int(max(100, min(20000, exposure_us)))
        if gain is not None:
            self._gain = float(max(1.0, min(16.0, gain)))
        self._apply_exposure()
        return self.exposure_status()

    def start_recording(self):
        self._recording = True
        self._ensure_thread()

    def stop_recording(self):
        self._recording = False

    def get_status(self):
        with self._st_lock:
            s = dict(self._status)
        s["armed"] = self._armed
        s["recording"] = self._recording
        s["exposure"] = self.exposure_status()
        s["cmd_scale"] = round(self._cmd_scale, 2)
        s["yaw_align"] = self.yaw_align_status()
        rf = self.rf.status() if self.rf else {"ok": False}
        s["rangefinder_ok"] = rf.get("ok", False)
        return s

    def _open_recording(self):
        os.makedirs(REC_DIR, exist_ok=True)
        name = "rec_" + time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(REC_DIR, name)
        w = cv2.VideoWriter(path + ".avi", cv2.VideoWriter_fourcc(*"MJPG"),
                            self.REC_FPS, (CV_W, CV_H))
        c = open(path + ".csv", "w")
        c.write(CSV_HEADER)
        with self._st_lock:
            self._status["rec_file"] = name
        return w, c

    def _run(self):
        """Kontroll-loop: bara detektera + skicka LANDING_TARGET (låg latens).
        Den tunga vyn (annotera + JPEG + inspelning + CSV) lämnas av till en
        writer-tråd via en bounded kö, så skick-takten inte bromsas av encoden."""
        with self.cam.cv_hold():
            self.cam.set_external_source(True)
            self._apply_exposure()
            self._last_tick = None
            self._q = queue.Queue(maxsize=self._Q_MAX)
            wt = threading.Thread(target=self._writer_loop, daemon=True)
            wt.start()
            try:
                while self._armed or self._recording:
                    t0 = time.time()
                    self._control_tick()
                    time.sleep(max(0.0, 1.0 / self.RATE_HZ - (time.time() - t0)))
            finally:
                self._q.put(None)        # signalera writer att flusha + avsluta
                wt.join(timeout=3)
                self._q = None
                self.cam.set_cv_exposure(False)
                self.cam.set_external_source(False)

    def _control_tick(self):
        """Latens-kritisk: detektera målet och skicka LANDING_TARGET direkt."""
        yuv, sensor_ts = self.cam.capture_lores_ts()
        gray = yuv[:CV_H, :CV_W]
        agl = self.rf.agl() if self.rf else None
        aruco = self.det.detect_aruco(gray)
        phase = self._decide_phase(agl, aruco)

        # BGR behövs bara för färgdetektering (COLOR-fas) och för annoterad video.
        need_video = self._recording or self.cam.viewers > 0
        bgr = None
        if phase == PHASE_COLOR or need_video:
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV420p2BGR)

        if phase == PHASE_ARUCO:
            target = aruco
        elif phase == PHASE_COLOR:
            target = self.det.detect_color(bgr)
        else:
            target = None

        ax = ay = ox = oy = yaw_err_deg = yaw_cmd_deg = None
        sent = False
        if target is not None:
            ax, ay = target.angles()
            ox, oy = target.offset_norm()
            if phase != PHASE_RTK and self._armed:
                k = self._cmd_scale                      # styrskala: mildare korrektion
                self.tel.send_landing_target(ax * k, ay * k, agl if agl else 0.0)
                sent = True
                self._tx += 1
            if target.source == "aruco":
                # Beräknas alltid (även av-slaget) så felet + mål-heading syns i
                # webben/CSV:n för bänktest av YAW_ERR_SIGN — se kommentaren vid
                # konstanten ovan. yaw_cmd_deg räknas oberoende av om det faktiskt
                # skickas (throttlat nedan), så loggen får full 40Hz upplösning.
                ye = target.yaw_error_rad()
                if ye is not None:
                    yaw_err_deg = math.degrees(ye)
                    cur_yaw = self.tel.snapshot().get("yaw")
                    if cur_yaw is not None:
                        yaw_cmd_deg = (cur_yaw + YAW_ERR_SIGN * yaw_err_deg) % 360.0
                        if self._yaw_align and self._armed and phase == PHASE_ARUCO:
                            now_yaw = time.time()
                            if now_yaw - self._last_yaw_tx >= 1.0 / self.YAW_CMD_HZ:
                                self.tel.send_condition_yaw(yaw_cmd_deg, self._yaw_rate)
                                self._last_yaw_tx = now_yaw

        # kamerakalibrering: implied markavstånd (cm) = agl*tan(vinkel), och uppmätt
        # brännvidd ur ArUco-markörens px-storlek + känd fysisk storlek + agl.
        calib = None
        if target is not None and agl:
            gx, gy = agl * math.tan(ax), agl * math.tan(ay)
            calib = {"goff_cm": round(100 * math.hypot(gx, gy)), "px": None,
                     "f_meas": None, "assumed_f": round(ASSUMED_F)}
            if target.source == "aruco":
                px = target.px_size()
                if px:
                    calib["px"] = round(px, 1)
                    calib["f_meas"] = round(px * agl / MARKER_M)

        # Pi-latens (kamera-fångst → nu, strax efter skick) + verklig loop-takt, EWMA
        now = time.time()
        if self._last_tick is not None:
            dt = now - self._last_tick
            if dt > 0:
                hz = 1.0 / dt
                self._loop_hz = hz if self._loop_hz is None else 0.8 * self._loop_hz + 0.2 * hz
        self._last_tick = now
        if sensor_ts:
            lat = (time.clock_gettime(time.CLOCK_BOOTTIME) * 1e9 - sensor_ts) / 1e6
            if 0 <= lat < 2000:
                self._lat_ms = lat if self._lat_ms is None else 0.8 * self._lat_ms + 0.2 * lat

        with self._st_lock:
            self._status.update(
                phase=phase, source=(target.source if target else None),
                agl=(round(agl, 2) if agl is not None else None),
                offset=([round(ox, 3), round(oy, 3)] if target else None),
                tx=self._tx, sent=sent, calib=calib,
                yaw_err_deg=(round(yaw_err_deg, 1) if yaw_err_deg is not None else None),
                latency_ms=(round(self._lat_ms, 1) if self._lat_ms else None),
                loop_hz=(round(self._loop_hz, 1) if self._loop_hz else None),
            )

        # lämna av tung vy/inspelning till writer-tråden (icke-kritisk väg)
        if need_video and bgr is not None:
            self._enqueue((bgr, target, phase, agl, sent, self._armed,
                           self._recording, ox, oy, ax, ay, time.time(),
                           self.tel.snapshot(), yaw_err_deg, yaw_cmd_deg))

    def _decide_phase(self, agl, aruco):
        if agl is None:                       # ingen AGL (t.ex. bänk utan TF-Luna) → detektionsbaserat
            return PHASE_ARUCO if aruco is not None else PHASE_COLOR
        if agl < self.RTK_AGL:
            return PHASE_RTK
        if aruco is not None and agl < self.ARUCO_MAX_AGL:
            return PHASE_ARUCO
        return PHASE_COLOR

    def _enqueue(self, item):
        try:
            self._q.put_nowait(item)
        except queue.Full:
            try:
                self._q.get_nowait()          # släpp äldsta → web hålls färsk, inspelning tappar en ruta
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(item)
            except queue.Full:
                pass

    def _writer_loop(self):
        """Bakgrund: annotera, encoda JPEG (web), skriv inspelning + CSV."""
        writer = csvf = None
        try:
            while True:
                try:
                    item = self._q.get(timeout=1.0)
                except queue.Empty:
                    if writer is not None and not self._recording:
                        writer.release(); csvf.close(); writer = csvf = None
                    continue
                if item is None:
                    break
                (bgr, target, phase, agl, sent, armed, recording,
                 ox, oy, ax, ay, t, tel, yaw_err_deg, yaw_cmd_deg) = item
                out = self.det.annotate(bgr, target, phase=phase, agl=agl)
                ok, jpg = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok:
                    self.cam.push_frame(jpg.tobytes())
                if recording:
                    if writer is None:
                        writer, csvf = self._open_recording()
                    writer.write(out)
                    csvf.write(self._row(t, tel, armed, agl, phase, target,
                                         ox, oy, ax, ay, sent, yaw_err_deg, yaw_cmd_deg))
                elif writer is not None:
                    writer.release(); csvf.close(); writer = csvf = None
        finally:
            if writer is not None:
                writer.release()
            if csvf is not None:
                csvf.close()

    def _row(self, t, tel, armed, agl, phase, target, ox, oy, ax, ay, sent,
             yaw_err_deg, yaw_cmd_deg):
        return ",".join([
            "%.3f" % t, str(tel.get("mode", "")), "1" if armed else "0",
            _f(tel.get("roll")), _f(tel.get("pitch")), _f(tel.get("yaw")), _f(tel.get("heading")),
            _f(tel.get("alt")), _f(agl), _f(tel.get("voltage")), _f(tel.get("battery_remaining")),
            _f(tel.get("fix_type")), _f(tel.get("satellites")), phase or "",
            (target.source if target else ""), _f(ox), _f(oy),
            _f(math.degrees(ax) if ax is not None else None),
            _f(math.degrees(ay) if ay is not None else None), "1" if sent else "0",
            "%.2f" % self._cmd_scale, _f(yaw_err_deg), _f(yaw_cmd_deg),
            "1" if self._yaw_align else "0",
        ]) + "\n"


# ---- fristående test ---------------------------------------------------
if __name__ == "__main__":
    import sys

    src = cv2.imread(sys.argv[1])
    target_px = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    # simulera "sett från höjd": lägg plattan i target_px storlek på grå 640x480
    scaled = cv2.resize(src, (target_px, target_px))
    frame = np.full((CV_H, CV_W, 3), 110, np.uint8)
    y0, x0 = (CV_H - target_px) // 2, (CV_W - target_px) // 2
    frame[y0:y0 + target_px, x0:x0 + target_px] = scaled

    det = PrecLandDetector()
    a = det.detect_aruco(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    c = det.detect_color(frame)
    print("platta %d px i 640x480:" % target_px)
    print("  ArUco:", "id=%d @ (%.0f,%.0f)" % (a.aruco_id, a.u, a.v) if a else "MISS")
    print("  Color:", "r=%.0f @ (%.0f,%.0f)" % (c.radius, c.u, c.v) if c else "MISS")
    out = det.annotate(frame, a or c, phase="TEST")
    cv2.imwrite("/tmp/precland_test.png", out)
    print("  -> /tmp/precland_test.png")
