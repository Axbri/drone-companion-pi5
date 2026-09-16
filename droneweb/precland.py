"""Precisionslandnings-CV för droneweb (Steg 3: alltid-på gråskale-spårning).

Änd 2026-08-21: färgdetektering (orange platta, för mål på hög höjd) borttagen.
Bänktest (1:4-skalad markör, se PRECISION-LANDING.md) visade att ArUco-räckvidden
var den faktiska begränsningen, inte upplösningen — färgläget gav ingen extra
räckvidd i praktiken. Upplösningen höjd 640x480 → 1024x768 (Pi 5 har gott om
marginal jämfört med Pi 3A+ som satte den gamla gränsen) för att utöka ArUco-
räckvidden istället, med samma geometri: fler px/grad → markören syns på större
avstånd. Kameran används numera bara gråskala (Y-planet av YUV420) — ingen
kulör behövs längre utan färgdetektering.

Systemet kör alltid (ingen manuell Arm/Disarm längre): PrecLandController startar
sin loop direkt vid konstruktion och går för appens hela livstid. Detta är säkert
eftersom ArduPilots AC_PrecLand redan bara agerar på LANDING_TARGET i relevanta
lägen/faser (se kommentaren vid gir-inriktningen nedan för det ena undantaget,
CONDITION_YAW, som INTE har samma inbyggda spärr och därför läges-grindas här).

Returnerar målets pixelposition, och räknar om pixel->vinkel (angle_x/angle_y i
rad) för LANDING_TARGET. Fas-logik (ARUCO vs RTK-HOLD) + MAVLink-sändning sker
i PrecLandController (denna modul); AGL kommer från rangefinder.py.

Modulen är fristående testbar:  python precland.py <bild> [målstorlek_px]
"""
import json
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

# Camera model at the CV resolution (= camera.py CAPTURE_SIZE). 2026-09-14: IMX219
# replaced by IMX296 (global shutter, mono) with a wide-angle CS lens, run at its native
# 1456x1088 (see camera.py for the resolution trade-off). F_PX measured on the bench:
# 14.5 cm marker at 94 cm → 142.7 px side → f = 142.7 * 0.94 / 0.145 ≈ 925 px
# (HFOV ≈ 76°, VFOV ≈ 61°). Measured near image centre — lens distortion at the edges
# is not modelled. Re-measure (web "Camera calibration" card, f_meas vs assumed_f) if
# the lens is changed or refocused. Previous: IMX219 at 1024x768, HFOV 62.2 / VFOV
# 48.8 → f ≈ 848 px; the IMX296 at 1024x768 (ISP-scaled) measured f = 651 px.
# With camera_calib_imx296.json present (the normal case since 2026-09-15) this pinhole
# is only the fallback and the reference frame for rectified pixel quantities (px_size,
# calibration card): angles come from the fisheye model in CameraModel below.
CV_W, CV_H = 1456, 1088
F_PX = 925.0
FX = FY = F_PX
HFOV_DEG = math.degrees(2 * math.atan((CV_W / 2) / FX))
VFOV_DEG = math.degrees(2 * math.atan((CV_H / 2) / FY))
CX, CY = CV_W / 2.0, CV_H / 2.0

# Lens calibration (camera-calibration/calib_solve.py output). When present, ArUco points
# are undistorted through it (fisheye or pinhole+distortion) before angles are computed,
# so the target bearing is right out at the image edges where the wide lens is far from
# a pinhole. Without the file the plain F_PX pinhole above is used.
MAX_NORM = math.tan(math.radians(70))   # |x/z| beyond 70° off-axis = model diverged
CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calib_imx296.json")


class CameraModel:
    """Pixel → normalised camera coordinates (x/z, y/z), calibrated or plain pinhole."""

    def __init__(self, path=CALIB_PATH):
        self.K = self.D = None
        self.fisheye = False
        self.desc = f"uncalibrated pinhole F_PX={F_PX:.0f}"
        try:
            with open(path) as f:
                c = json.load(f)
            if tuple(c["size"]) != (CV_W, CV_H):
                raise ValueError(f"calibration size {c['size']} != CV {CV_W}x{CV_H}")
            self.K = np.array(c["K"], np.float64)
            self.D = np.array(c["D"], np.float64)
            self.fisheye = c["model"] == "fisheye"
            self.desc = (f"{c['model']} f=({self.K[0, 0]:.0f},{self.K[1, 1]:.0f}) "
                         f"rms {c['rms']} px, {c['n_views']} views, {c['date']}")
        except FileNotFoundError:
            pass
        except Exception as e:      # malformed file: fall back rather than take the app down
            print(f"[precland] WARNING: ignoring {path}: {e}")
        print(f"[precland] camera model: {self.desc}")

    def normalize(self, pts):
        """(N,2) px → (N,2) normalised (x/z, y/z) on the undistorted camera."""
        p = np.asarray(pts, np.float64).reshape(-1, 1, 2)
        pin = (p.reshape(-1, 2) - (CX, CY)) / (FX, FY)
        if self.K is None:
            return pin
        und = cv2.fisheye.undistortPoints if self.fisheye else cv2.undistortPoints
        n = und(p, self.K, self.D).reshape(-1, 2)
        # The distortion polynomial is only valid inside the calibrated region; outside
        # it (or if the iterative inverse fails) it diverges — fall back to the pinhole
        # for that point rather than send a wild bearing to the FC.
        bad = ~np.isfinite(n).all(axis=1) | (np.abs(n) > MAX_NORM).any(axis=1)
        if bad.any():
            n[bad] = pin[bad]
        return n

    def rectify_px(self, pts):
        """(N,2) px → undistorted px on the F_PX pinhole (CX, CY), so pixel-unit quantities
        (marker px size for the calibration card, yaw vector) keep their old meaning."""
        return self.normalize(pts) * (FX, FY) + (CX, CY)

    def preview_maps(self, size, zoom=0.72):
        """Remap tables that undistort AND downscale the full frame to `size` in one pass —
        for the web preview only (the CV loop and the recording stay on the raw image).
        zoom < 1 pulls the edges in so the whole (now wider) view fits, black corners are
        the fisheye's own footprint. None without a calibration."""
        if self.K is None:
            return None
        sx, sy = size[0] / CV_W, size[1] / CV_H
        newK = np.array([[self.K[0, 0] * sx * zoom, 0, size[0] / 2.0],
                         [0, self.K[1, 1] * sy * zoom, size[1] / 2.0],
                         [0, 0, 1]])
        if self.fisheye:
            return cv2.fisheye.initUndistortRectifyMap(self.K, self.D, np.eye(3), newK, size, cv2.CV_16SC2)
        return cv2.initUndistortRectifyMap(self.K, self.D, None, newK, size, cv2.CV_16SC2)


CAM = CameraModel()

# Kamerakalibrering: ArUco-markörens verkliga sidlängd (svarta fyrkanten), meter.
# MÄT den utskrivna markören och sätt rätt värde — hela kalibreringen beror på detta.
# 0.30 = landnings-markörens ArUco (flyg). Bänk-test-markören var 0.145.
MARKER_M = 0.30
ASSUMED_F = FX               # antagen brännvidd (px) att jämföra uppmätt mot
# Marker-derived range (Target.range_m) vs a tape measure on the bench, 2026-09-15: 1.02 m
# at 1.00 and 2.04 m at 2.00 — a pure 2 % scale (no offset), i.e. the calibrated f is ~2 %
# high. Corrected here rather than in the calibration. (Lidar read 0.97 / 1.97 in the same
# test: a constant -3 cm from its mounting position, not a scale error.)
MARKER_RANGE_SCALE = 1.0 / 1.02

PHASE_WAIT, PHASE_ARUCO, PHASE_RTK = "WAIT", "ARUCO", "RTK-HOLD"

# Fast exponering under CV-loopen (mot motion blur på höjd — Fynd 1). Kort slutartid
# fryser rörelsen så rutan blir skarp trots kameravibration; auto-exponering (default)
# väljer ibland lång slutartid i starkt ljus och smetar ut markören. Tuna 1500-2500 µs
# efter ljuset; höj gain om för mörkt (men mycket gain = brus som stör detekteringen).
CV_EXPOSURE_US = 2000
CV_GAIN = 2.0

# Styrskala: skalar vinkelfelet som skickas till ArduPilot innan LANDING_TARGET.
# 1.0 = fullt fel (som kameran ser det). <1.0 = mildare korrektioner (mot översläng vid
# latens) — drönaren konvergerar ändå geometriskt om landningshastigheten är låg.
# Justeras live i webben. Detta är i praktiken det precland-gain ArduPilot saknar.
CMD_SCALE_DEFAULT = 1.0

# Kamerans fram/bak-offset från kroppens rotationscentrum (CG), cm, kroppens X-axel
# (framåt positivt — samma FRD-konvention som resten av modulen). ArduPilots egna
# PLND_CAM_POS_*-parametrar verkade INTE ha effekt via MAVLink-backend'en (testat
# 2026-08-21) — okänt om det är en begränsning i AC_PrecLand_MAVLink eller fel
# konfigurerat, men kompenseras därför här istället. Default -16,5 cm = kameran sitter
# 16,5 cm BAKOM CG (uppmätt). Live-justerbar i webben om kameran flyttas.
CAM_OFFSET_DEFAULT_CM = -16.5
CAM_OFFSET_MIN_CM, CAM_OFFSET_MAX_CM = -30.0, 30.0

# Nära marken kräver full offset-korrigering att markören syns nära/utanför bildkanten
# (bekräftat i flygdata 2026-08-23: 0% ArUco-detektion 0,3-0,6 m AGL med -16,5 cm —
# vid perfekt CG-centrering måste kameran då se målet vid atan(0,165/agl) ≈ ≥24° från
# bildcentrum, vilket redan är halva VFOV på ~0,36 m). En "perfekt" korrigering som
# tappar målet är sämre än en delvis korrigering som behåller spårningen (RTK-hold tar
# ändå hand om den sista resten). Håller korrigeringen inom en säker andel av halva
# VFOV, oavsett verkligt spårningsfel också bidrar till samma bildkant.
CAM_OFFSET_MAX_FOV_FRAC = 0.6   # andel av halva VFOV som korrigeringen ensam får kräva


def _apply_cam_offset(ax, ay, agl, offset_fwd_m):
    """Räknar om målets siktlinjevinkel (uppmätt från KAMERAN) till motsvarande vinkel
    som skulle setts från CG, givet kamerans kända fasta fram/bak-offset. Utan detta
    ger en ren gir (ingen verklig translation) en falsk skenbar målrörelse, och
    positionskorrektionen blir systematiskt fel proportionellt mot offset/agl.

    Geometri (kroppens FRD, X=fram/Y=höger/Z=ned, kameran pekar rakt ner, monterad utan
    egen vridning relativt kroppen): målets position rel. kameran är
    (fram, höger, ned) = agl * (-tan(ay), tan(ax), 1) — se kommentaren om
    AC_PrecLand_MAVLink::handle_msg() ovan för samma konvention. Lägg till kamerans
    kända position rel. CG (offset_fwd_m, 0, 0) för att få målets position rel. CG,
    och räkna om till vinkel med samma konvention.

    offset_fwd_m klipps (inte den slutliga vinkeln — verkligt spårningsfel ska inte
    klippas bort) till max CAM_OFFSET_MAX_FOV_FRAC av halva VFOV vid given agl, så
    korrigeringen tonas ned mjukt nära marken istället för att tvinga målet ur bild."""
    if agl is None or agl <= 0 or offset_fwd_m == 0.0:
        return ax, ay
    max_offset_m = agl * math.tan(CAM_OFFSET_MAX_FOV_FRAC * math.radians(VFOV_DEG / 2.0))
    offset_fwd_m = math.copysign(min(abs(offset_fwd_m), max_offset_m), offset_fwd_m)
    fwd_cam = -math.tan(ay) * agl
    right_cam = math.tan(ax) * agl
    fwd_cg = fwd_cam + offset_fwd_m
    down_cg = agl                          # ingen vertikal/sidled kamera-offset hanterad
    return math.atan2(right_cam, down_cg), math.atan2(-fwd_cg, down_cg)


# Marker corners in the marker's own frame, in the order cv2.SOLVEPNP_IPPE_SQUARE
# requires (TL, TR, BR, BL with y up) — the same order ArUco returns image corners in.
_MARKER_OBJ = np.array([[-0.5, 0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, -0.5, 0]],
                       np.float64) * MARKER_M


class Target:
    __slots__ = ("u", "v", "source", "aruco_id", "corners", "_pose")

    def __init__(self, u, v, source, aruco_id=None, corners=None):
        self.u, self.v, self.source = u, v, source
        self.aruco_id, self.corners = aruco_id, corners
        self._pose = None

    def range_m(self):
        """(los_range_m, axis_dist_m) from the marker's apparent size and shape: solvePnP
        on the four undistorted corners (known MARKER_M square). los = straight-line
        distance camera→marker centre (what LANDING_TARGET.distance means), axis = its
        component along the optical axis (≈ AGL when the camera points straight down).
        None without corners or if the solve fails. Replaces the lidar above its range."""
        if self.corners is None:
            return None
        if self._pose is None:
            img = CAM.normalize(self.corners).reshape(4, 1, 2)
            ok, _, tvec = cv2.solvePnP(_MARKER_OBJ, img, np.eye(3), None,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
            t = tvec.ravel() * MARKER_RANGE_SCALE
            self._pose = (float(np.linalg.norm(t)), float(t[2])) if ok and t[2] > 0 else False
        return self._pose or None

    def angles(self):
        """(angle_x, angle_y) i rad rel. kameraxeln. +x=höger, +y=nedåt i bilden.
        Kamera-monteringsmappning till kroppsframe (BODY_FRD) görs i app.py."""
        x, y = CAM.normalize([[self.u, self.v]])[0]
        return math.atan(x), math.atan(y)

    def offset_norm(self):
        """Normaliserad offset [-1,1] från bildcentrum (för UI)."""
        return (self.u - CX) / (CV_W / 2), (self.v - CY) / (CV_H / 2)

    def px_size(self):
        """Genomsnittlig sidlängd (px) för ArUco-fyrkanten, ur de fyra hörnen.
        För kamerakalibrering: f = px_size * avstånd / markör_verklig_storlek."""
        if self.corners is None:
            return None
        p = CAM.rectify_px(self.corners)
        return sum(math.hypot(*(p[(i + 1) % 4] - p[i])) for i in range(4)) / 4.0

    def yaw_error_rad(self):
        """Vinkelfel (rad) mellan markörens tryckta "upp" (samma vektor som
        riktningspilen i annotate()) och bildens "upp" (=nosen, se PLND_YAW_ALIGN-
        kommentaren nedan). 0 = pilen pekar rakt upp. Positivt = pilen lutar mot
        höger i bilden. Bara giltigt för ArUco (kräver corners)."""
        if self.corners is None:
            return None
        up = _marker_up_vector(CAM.rectify_px(self.corners))
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

    def annotate(self, bgr, target, phase=None, agl=None, text=True):
        """Ritar bildanalys-grafiken (visas i webben). `bgr` är en gråskalebild
        konverterad till 3-kanalig (cv2.COLOR_GRAY2BGR) bara för att overlayn ska
        synas i färg — själva bilden bär ingen kulörinformation."""
        cv2.drawMarker(bgr, (int(CX), int(CY)), (120, 120, 120),
                       cv2.MARKER_CROSS, 20, 1)
        if target is not None:
            u, v = int(target.u), int(target.v)
            # Grön = ARUCO-fasen (aktivt styrande). Röd = WAIT/RTK-HOLD — detektering
            # körs där numera också (räckviddstest), men inget skickas; röd markerar
            # tydligt att det bara är diagnostik.
            col = (0, 255, 0) if phase == PHASE_ARUCO else (0, 0, 255)
            if target.corners is not None:
                cv2.polylines(bgr, [target.corners.astype(np.int32)], True, col, 2)
                # Riktningspil: markörens tryckta "upp" (se _marker_up_vector).
                # Samma vektor används av Target.yaw_error_rad() för gir-inriktning.
                up = _marker_up_vector(target.corners)
                tip = (u + up[0] * 0.7, v + up[1] * 0.7)
                cv2.arrowedLine(bgr, (u, v), (int(round(tip[0])), int(round(tip[1]))),
                                col, 2, tipLength=0.35)
            cv2.line(bgr, (int(CX), int(CY)), (u, v), col, 2)
            cv2.circle(bgr, (u, v), 4, col, -1)
        if text:
            self.hud_text(bgr, target, phase, agl)
        return bgr

    @staticmethod
    def hud_text(img, target, phase=None, agl=None):
        """Text overlay, sized to `img` — drawn separately from the geometry so the
        undistorted preview can add it AFTER the remap (else the text warps too)."""
        h, w = img.shape[:2]
        k = w / CV_W                      # 1.0 on the full frame, 0.5 on the preview
        if target is not None:
            col = (0, 255, 0) if phase == PHASE_ARUCO else (0, 0, 255)
            ax, ay = target.angles()
            cv2.putText(img, "%s  ax=%.1f ay=%.1f deg" % (
                phase or "ARUCO", math.degrees(ax), math.degrees(ay)),
                (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, max(0.4, 0.5 * k * 1.4), col, 1)
        hud = []
        if phase:
            hud.append(phase)
        if agl is not None:
            hud.append("AGL %.2fm" % agl)
        if hud:
            cv2.putText(img, "  ".join(hud), (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.5, 0.6 * k * 1.4), (255, 255, 255), 2)


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
# bekräftar tecknet. YAW_ERR_SIGN=1 bekräftat korrekt, även flygverifierat
# (Flygtest #5, PRECISION-LANDING.md) — fyra landningar, alla konvergerade snabbt
# till <1° fel oavsett startriktning.
#
# LÄGES-GRIND (tillagd 2026-08-21, i samband med alltid-på-spårning): till skillnad
# från LANDING_TARGET — som AC_PrecLand redan själv bara agerar på i relevanta
# lägen/faser — är CONDITION_YAWs auto_yaw-mål "sticky" och dess beteende vid
# lägesbyte (t.ex. om ett mål sätts i ett läge som inte konsumerar auto_yaw, och
# sedan LAND/RTL aktiveras senare) är INTE verifierat. Nu när armering alltid är
# på (ingen pilot-avsiktlig "arm precis innan landning" längre) skickas
# CONDITION_YAW bara när flygläget faktiskt är LAND eller RTL (de lägen som
# flygtestats, Flygtest #4/#5) — inte i t.ex. LOITER/STABILIZE där en tillfällig
# markör-siktning annars skulle kunna lämna ett kvarglömt gir-mål.
YAW_ERR_SIGN = 1
YAW_MODES = ("LAND", "RTL")


REC_DIR = "/home/axel/recordings"
CSV_HEADER = ("t,mode,armed,roll,pitch,yaw,heading,alt,agl,batt_v,batt_pct,"
              "fix,sats,phase,source,ox,oy,ax_deg,ay_deg,sent,scale,"
              "yaw_err_deg,yaw_cmd_deg,yaw_align,loop_hz,latency_ms,"
              "agl_src,mrange,magl\n")     # agl = effective (lidar or marker); mrange = marker
                                         # line-of-sight range, magl = marker-derived AGL


def _f(v, nd=3):
    if v is None:
        return ""
    return ("%.*f" % (nd, v)) if isinstance(v, float) else str(v)


class PrecLandController:
    """Kör CV-loopen kontinuerligt från appens start (ingen manuell Arm/Disarm
    längre): detekterar ArUco varje ruta OAVSETT fas (räckviddstest — se annotate(),
    röd overlay utanför ARUCO-fasen), men skickar bara LANDING_TARGET/CONDITION_YAW
    när fasen faktiskt är ARUCO. Fas väljs efter AGL — WAIT (ovanför aruco_start_agl),
    ARUCO (rtk_agl..aruco_start_agl), RTK-HOLD (under rtk_agl) — matar en nedskalad
    live-förhandsvisning (bara när någon tittar), och spelar in synkad video (.avi) +
    datalogg (.csv) på begäran (oberoende av spårningen, som alltid går). Höjdtrösklarna
    är live-justerbara, se set_thresholds()."""

    RATE_HZ = 40          # kontroll-loopens tak (matchar kamera-FPS). Tung JPEG-encode + inspelning
                          # körs i separat writer-tråd, så detekt+skick av LANDING_TARGET tightas
                          # (mot översläng/latens — Fynd 2). Pi 5, 2026-08-18: 40 Hz stabilt/repeterbart
                          # vid 640x480 — EJ omverifierat vid 1024x768 (2026-08-21), se loop_hz i webben.
    REC_FPS = 8           # AVI-fps-metadata (nominell). Verklig inspelningstakt är writer-begränsad;
                          # CSV:ns t-kolumn är den exakta tiden per ruta (ruta N = CSV-rad N).
    _Q_MAX = 8            # writer-köns tak → drop-oldest (håller RAM nere)

    # Höjdtrösklar — live-justerbara i webben (för experiment under flygning). Default-
    # värdena är startpunkter, inte hårda gränser; se set_thresholds().
    ARUCO_START_AGL_DEFAULT = 20.0         # m — above this: WAIT phase, nothing sent even if seen.
                                           # 7→12 (2026-09-15) when the marker's own range estimate
                                           # took over above the lidar; →20 (2026-09-16) after flight
                                           # test #8 acquired the 30 cm marker at 11-20 m. Matches
                                           # PLND_ALT_MAX=20 on the FC; the web slider lowers it.
    ARUCO_START_MIN, ARUCO_START_MAX = 1.0, 20.0
    RTK_AGL_DEFAULT = 0.3                  # m — under denna: sluta skicka, RTK håller x/y
    RTK_AGL_MIN, RTK_AGL_MAX = 0.1, 2.0
    YAW_START_AGL_DEFAULT = 5.0            # m — girinriktning börjar inte förrän under denna höjd
    YAW_START_MIN, YAW_START_MAX = 0.5, 10.0

    YAW_RATE_DEFAULT = 45.0        # deg/s, live-justerbar i webben
    YAW_RATE_MIN, YAW_RATE_MAX = 5.0, 90.0
    YAW_CMD_HZ = 4                 # CONDITION_YAW skickas mycket glesare än LANDING_TARGET;
                                    # auto_yaw rampar själv kontinuerligt mellan uppdateringarna

    PUSH_HZ = 20                        # live-förhandsvisningens takt (halva RATE_HZ) — sparar bandbredd
    PUSH_SIZE = (CV_W // 2, CV_H // 2)   # halva upplösningen för samma anledning (728x544)
    PREVIEW_UNDISTORT = False            # remap the web preview through the lens calibration
                                         # (CameraModel.preview_maps). Off by default to save
                                         # CPU on the writer thread — the calibration itself is
                                         # always applied to the ArUco measurement regardless.
                                         # Verified visually 2026-09-15 (straight door frames).

    ENQUEUE_HZ = 20   # takt för BÅDE live-förhandsvisning och inspelning (CSV/AVI) till writer-
                      # tråden. Inspelning körde tidigare varje tick (40Hz) obegränsat — writer-
                      # trådens annotate() gör mer jobb när ett mål faktiskt hittas (ritar ut box/
                      # pil/text), vilket konkurrerar om GIL:en med kontroll-tråden. Uppmätt i
                      # flygdata 2026-08-24: loop_hz var ALDRIG ≥25Hz i samma sekund som en
                      # detektion lyckades, konsekvent över 28 flygningar — 0% overlap. Halverad
                      # inspelningstakt ger grövre tidsupplösning i CSV/AVI men en stabilare
                      # kontroll-loop (som styr LANDING_TARGET/CONDITION_YAW-takten, vilket spelar
                      # större roll för landningen än videons tidsupplösning).

    def __init__(self, cam, tel, rangefinder=None):
        self.cam, self.tel, self.rf = cam, tel, rangefinder
        self.det = PrecLandDetector()
        self._preview_maps = CAM.preview_maps(self.PUSH_SIZE) if self.PREVIEW_UNDISTORT else None
        self._recording = False
        self._last_armed = False   # för att detektera disarm (True→False), inte bara "är disarmerad"
        self._tx = 0
        self._q = queue.Queue(maxsize=self._Q_MAX)
        self._last_push_ts = 0.0
        self._last_enqueue_ts = 0.0
        # CV-exponering (justeras live från webben för fältjustering)
        self._auto_exposure = False
        self._exposure_us = CV_EXPOSURE_US
        self._gain = CV_GAIN
        self._cmd_scale = CMD_SCALE_DEFAULT   # styrskala för LANDING_TARGET (live)
        self._cam_offset_m = CAM_OFFSET_DEFAULT_CM / 100.0   # kamerans fram/bak-offset från CG
        self._aruco_start_agl = self.ARUCO_START_AGL_DEFAULT
        self._rtk_agl = self.RTK_AGL_DEFAULT
        self._yaw_start_agl = self.YAW_START_AGL_DEFAULT
        self._yaw_align = True                # av = ingen CONDITION_YAW skickas alls
        self._yaw_rate = self.YAW_RATE_DEFAULT
        self._last_yaw_tx = 0.0
        self._lat_ms = None                   # Pi-latens (kamera-fångst → skick), EWMA
        self._loop_hz = None                  # verklig loop-takt, EWMA
        self._last_tick = None
        self._st_lock = threading.Lock()
        self._status = {"phase": None, "source": None, "agl": None, "agl_src": None,
                        "marker_range": None,
                        "offset": None, "tx": 0, "sent": False, "rec_file": None,
                        "calib": None, "latency_ms": None, "loop_hz": None,
                        "yaw_err_deg": None}
        self._apply_exposure()
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._writer_loop, daemon=True).start()

    def _apply_exposure(self):
        """Skjut aktuellt exponerings-läge till kameran."""
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

    def cam_offset_status(self):
        return {"offset_cm": round(self._cam_offset_m * 100.0, 1)}

    def set_cam_offset(self, offset_cm=None):
        """Kamerans fram/bak-offset från CG (cm, framåt positivt). Live — flytta
        kameran fysiskt och uppdatera detta värde, ingen omdeploy behövs."""
        if offset_cm is not None:
            self._cam_offset_m = float(max(CAM_OFFSET_MIN_CM, min(CAM_OFFSET_MAX_CM, offset_cm))) / 100.0
        return self.cam_offset_status()

    def threshold_status(self):
        return {"aruco_start_agl": round(self._aruco_start_agl, 1),
                "rtk_agl": round(self._rtk_agl, 2),
                "yaw_start_agl": round(self._yaw_start_agl, 1)}

    def set_thresholds(self, aruco_start_agl=None, rtk_agl=None, yaw_start_agl=None):
        """Höjdtrösklar (AGL, meter), live-justerbara för experiment under flygning:
        aruco_start_agl = ovanför denna letas inte ens efter ArUco (fas WAIT),
        rtk_agl = under denna slutar sändningen, RTK håller (fas RTK-HOLD),
        yaw_start_agl = under denna får girinriktning börja (kräver även yaw_align på).
        Klämmer var för sig — ingen inbördes ordning tvingas fram."""
        if aruco_start_agl is not None:
            self._aruco_start_agl = float(max(self.ARUCO_START_MIN, min(self.ARUCO_START_MAX, aruco_start_agl)))
        if rtk_agl is not None:
            self._rtk_agl = float(max(self.RTK_AGL_MIN, min(self.RTK_AGL_MAX, rtk_agl)))
        if yaw_start_agl is not None:
            self._yaw_start_agl = float(max(self.YAW_START_MIN, min(self.YAW_START_MAX, yaw_start_agl)))
        return self.threshold_status()

    def yaw_align_status(self):
        return {"enabled": self._yaw_align, "rate_degs": round(self._yaw_rate, 1)}

    def set_yaw_align(self, enabled=None, rate_degs=None):
        """Av/på för girinriktning mot markören + vridhastighet (grader/s). Live.
        Av = ingen CONDITION_YAW skickas alls."""
        if enabled is not None:
            self._yaw_align = bool(enabled)
        if rate_degs is not None:
            self._yaw_rate = float(max(self.YAW_RATE_MIN, min(self.YAW_RATE_MAX, rate_degs)))
        return self.yaw_align_status()

    def set_exposure(self, auto=None, exposure_us=None, gain=None):
        """Ställ CV-exponering live från webben (fältjustering). Klämmer värden och
        applicerar direkt."""
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

    def stop_recording(self):
        self._recording = False

    def get_status(self):
        with self._st_lock:
            s = dict(self._status)
        s["armed"] = True             # spårningen är alltid på (Steg 3, 2026-08-21)
        s["recording"] = self._recording
        s["exposure"] = self.exposure_status()
        s["cmd_scale"] = round(self._cmd_scale, 2)
        s["cam_offset"] = self.cam_offset_status()
        s["yaw_align"] = self.yaw_align_status()
        s["thresholds"] = self.threshold_status()
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
        """Kontroll-loop: detektera + skicka LANDING_TARGET (låg latens), kontinuerligt
        för appens hela livstid. Den tunga vyn (annotera + JPEG + inspelning + CSV)
        lämnas av till en writer-tråd via en bounded kö, så skick-takten inte bromsas
        av encoden."""
        self._last_tick = None
        while True:
            t0 = time.time()
            self._control_tick()
            time.sleep(max(0.0, 1.0 / self.RATE_HZ - (time.time() - t0)))

    def _control_tick(self):
        """Latens-kritisk: detektera målet och skicka LANDING_TARGET direkt."""
        yuv, sensor_ts = self.cam.capture_ts()
        gray = yuv[:CV_H, :CV_W]
        tel_snap = self.tel.snapshot()

        # Stoppa inspelning automatiskt vid disarm (kant-triggat: bara True→False, inte
        # "är för tillfället disarmerad" — annars skulle det aldrig gå att spela in på
        # bänken utan att drönaren är armerad). _last_armed initieras False så första
        # riktiga läsningen aldrig ger ett falskt larm innan telemetri kommit in.
        armed_now = tel_snap.get("armed")
        if self._last_armed and armed_now is False and self._recording:
            self.stop_recording()
        if armed_now is not None:
            self._last_armed = armed_now

        # Detekterar alltid, oavsett fas (även WAIT/RTK-HOLD) — för räckviddstest (kan
        # den se markören högre upp / lägre ner än de aktiva trösklarna?). Sändning är
        # explicit grindad på phase == PHASE_ARUCO nedan, INTE på om ett mål hittades.
        aruco = self.det.detect_aruco(gray)
        target = aruco

        # AGL: lidar (TF-Luna, valid 0.1-8 m) when it has a value, else the marker's own
        # range estimate (solvePnP on its corners, tilt-corrected with the FC attitude) —
        # this is what lets tracking start above the lidar's range. The lidar-only value
        # `agl` is kept for the calibration card (marker-derived would be circular there).
        agl = self.rf.agl() if self.rf else None
        mrange = target.range_m() if target is not None else None
        magl = None
        if mrange is not None:
            roll, pitch = tel_snap.get("roll"), tel_snap.get("pitch")
            tilt = (math.cos(math.radians(roll)) * math.cos(math.radians(pitch))
                    if roll is not None and pitch is not None else 1.0)
            magl = mrange[1] * max(tilt, 0.5)
        if agl is not None:
            agl_eff, agl_src = agl, "lidar"
        elif magl is not None:
            agl_eff, agl_src = magl, "marker"
        else:
            agl_eff, agl_src = None, None
        phase = self._decide_phase(agl_eff)

        # Gråskala→BGR bara för overlay-färg i förhandsvisning/inspelning (cvtColor
        # GRAY2BGR är en billig kanal-replikering, inte en riktig färgkonvertering —
        # ingen kulörinformation finns kvar att konvertera från sedan färgläget togs bort).
        # Både förhandsvisning och inspelning throttlas till ENQUEUE_HZ (se konstanten) —
        # inspelning körde tidigare varje tick obegränsat, vilket gav writer-tråden
        # (annotate+skriv) betydligt mer GIL-konkurrens med kontroll-tråden.
        now_ve = time.time()
        need_video = (self._recording or self.cam.viewers > 0) and (
            now_ve - self._last_enqueue_ts >= 1.0 / self.ENQUEUE_HZ)
        if need_video:
            self._last_enqueue_ts = now_ve
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if need_video else None

        ax = ay = ax_s = ay_s = ox = oy = yaw_err_deg = yaw_cmd_deg = None
        sent = False
        if target is not None:
            ax, ay = target.angles()                      # rå, kamerans egen mätning (kalibrering)
            ax_s, ay_s = _apply_cam_offset(ax, ay, agl_eff, self._cam_offset_m)  # korrigerad, CG-referens
            ox, oy = target.offset_norm()
            if phase == PHASE_ARUCO:
                k = self._cmd_scale                      # styrskala: mildare korrektion
                # distance: ArduPilot uses it instead of its rangefinder when > 0
                # (AC_PrecLand::construct_pos_meas_using_rangefinder). Lidar AGL when valid
                # (flight-proven), else the marker's line-of-sight range; 0 if neither.
                dist = agl if agl else (mrange[0] if mrange else 0.0)
                self.tel.send_landing_target(ax_s * k, ay_s * k, dist)
                sent = True
                self._tx += 1

            # Beräknas alltid (även utanför ARUCO-fasen, även av-slaget) så felet +
            # mål-heading syns i webben/CSV:n som diagnostik. yaw_cmd_deg räknas
            # oberoende av om det faktiskt skickas (grindat nedan).
            ye = target.yaw_error_rad()
            if ye is not None:
                yaw_err_deg = math.degrees(ye)
                cur_yaw = tel_snap.get("yaw")
                if cur_yaw is not None:
                    yaw_cmd_deg = (cur_yaw + YAW_ERR_SIGN * yaw_err_deg) % 360.0
                    if (phase == PHASE_ARUCO and self._yaw_align
                            and tel_snap.get("mode") in YAW_MODES
                            and agl_eff is not None and agl_eff <= self._yaw_start_agl):
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
                agl=(round(agl_eff, 2) if agl_eff is not None else None), agl_src=agl_src,
                marker_range=(round(mrange[0], 2) if mrange else None),
                offset=([round(ox, 3), round(oy, 3)] if target else None),
                tx=self._tx, sent=sent, calib=calib,
                yaw_err_deg=(round(yaw_err_deg, 1) if yaw_err_deg is not None else None),
                latency_ms=(round(self._lat_ms, 1) if self._lat_ms else None),
                loop_hz=(round(self._loop_hz, 1) if self._loop_hz else None),
            )

        # lämna av tung vy/inspelning till writer-tråden (icke-kritisk väg). `sent`
        # skickas inte med — _row() räknar om den (phase != RTK and target) själv.
        # ax_s/ay_s (kamera-offset-korrigerade, det som faktiskt skickades) loggas i
        # CSV:n, inte de råa ax/ay (de används bara internt för kalibreringsdiagnostik).
        if need_video and bgr is not None:
            self._enqueue((bgr, target, phase, agl_eff,
                           self._recording, ox, oy, ax_s, ay_s, time.time(),
                           tel_snap, yaw_err_deg, yaw_cmd_deg,
                           self._loop_hz, self._lat_ms, agl_src, mrange, magl))

    def _decide_phase(self, agl):
        if agl is None:                        # ingen AGL (t.ex. bänk utan TF-Luna) -> detektera ändå
            return PHASE_ARUCO
        if agl < self._rtk_agl:
            return PHASE_RTK
        if agl > self._aruco_start_agl:
            return PHASE_WAIT
        return PHASE_ARUCO

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
        """Bakgrund: annotera, mata en nedskalad/frame-rate-begränsad live-förhands-
        visning (bara när någon tittar), skriv inspelning (full upplösning) + CSV."""
        writer = csvf = None
        try:
            while True:
                try:
                    item = self._q.get(timeout=1.0)
                except queue.Empty:
                    if writer is not None and not self._recording:
                        writer.release(); csvf.close(); writer = csvf = None
                    continue
                (bgr, target, phase, agl, recording,
                 ox, oy, ax, ay, t, tel, yaw_err_deg, yaw_cmd_deg,
                 loop_hz, lat_ms, agl_src, mrange, magl) = item

                do_push = False
                if self.cam.viewers > 0:
                    now = time.time()
                    if now - self._last_push_ts >= 1.0 / self.PUSH_HZ:
                        self._last_push_ts = now
                        do_push = True

                # annotate() ritar på hela CV_W x CV_H-rutan — bara värt kostnaden om den
                # faktiskt ska visas (do_push) eller sparas (recording), inte annars
                # (t.ex. en enqueue som bara nådde tröskeln pga inspelning medan preview-
                # takten redan har sin egen frame nyss).
                undist = self._preview_maps is not None
                out = (self.det.annotate(bgr, target, phase=phase, agl=agl, text=not undist)
                       if (do_push or recording) else None)

                if do_push:
                    if undist:      # undistorted preview (see CameraModel): remap the geometry
                        small = cv2.remap(out, *self._preview_maps, cv2.INTER_LINEAR)
                        self.det.hud_text(small, target, phase, agl)    # ...then the text, unwarped
                    else:
                        small = cv2.resize(out, self.PUSH_SIZE, interpolation=cv2.INTER_AREA)
                    ok, jpg = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                    if ok:
                        self.cam.push_frame(jpg.tobytes())
                if undist and recording:
                    self.det.hud_text(out, target, phase, agl)          # recording keeps the raw view + text

                if recording:
                    if writer is None:
                        writer, csvf = self._open_recording()
                    writer.write(out)
                    csvf.write(self._row(t, tel, agl, phase, target,
                                         ox, oy, ax, ay, yaw_err_deg, yaw_cmd_deg,
                                         loop_hz, lat_ms, agl_src, mrange, magl))
                elif writer is not None:
                    writer.release(); csvf.close(); writer = csvf = None
        finally:
            if writer is not None:
                writer.release()
            if csvf is not None:
                csvf.close()

    def _row(self, t, tel, agl, phase, target, ox, oy, ax, ay,
             yaw_err_deg, yaw_cmd_deg, loop_hz, lat_ms, agl_src=None, mrange=None, magl=None):
        sent = phase == PHASE_ARUCO and target is not None
        return ",".join([
            "%.3f" % t, str(tel.get("mode", "")), "1",
            _f(tel.get("roll")), _f(tel.get("pitch")), _f(tel.get("yaw")), _f(tel.get("heading")),
            _f(tel.get("alt")), _f(agl), _f(tel.get("voltage")), _f(tel.get("battery_remaining")),
            _f(tel.get("fix_type")), _f(tel.get("satellites")), phase or "",
            (target.source if target else ""), _f(ox), _f(oy),
            _f(math.degrees(ax) if ax is not None else None),
            _f(math.degrees(ay) if ay is not None else None), "1" if sent else "0",
            "%.2f" % self._cmd_scale, _f(yaw_err_deg), _f(yaw_cmd_deg),
            "1" if self._yaw_align else "0", _f(loop_hz, 1), _f(lat_ms, 1),
            agl_src or "", _f(mrange[0] if mrange else None), _f(magl),
        ]) + "\n"


# ---- fristående test ---------------------------------------------------
if __name__ == "__main__":
    import sys

    src = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
    target_px = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    # simulera "sett från höjd": lägg plattan i target_px storlek på grå CV_W x CV_H
    scaled = cv2.resize(src, (target_px, target_px))
    frame = np.full((CV_H, CV_W), 110, np.uint8)
    y0, x0 = (CV_H - target_px) // 2, (CV_W - target_px) // 2
    frame[y0:y0 + target_px, x0:x0 + target_px] = scaled

    det = PrecLandDetector()
    a = det.detect_aruco(frame)
    print("platta %d px i %dx%d:" % (target_px, CV_W, CV_H))
    print("  ArUco:", "id=%d @ (%.0f,%.0f)" % (a.aruco_id, a.u, a.v) if a else "MISS")
    out = det.annotate(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), a, phase="TEST")
    cv2.imwrite("/tmp/precland_test.png", out)
    print("  -> /tmp/precland_test.png")
