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

import cv2
import numpy as np

# ---- konfig ------------------------------------------------------------
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_ID = 0

# safety-orange i HSV (OpenCV H 0-180). Tunas på plats.
ORANGE_LO = np.array([5, 120, 80], np.uint8)
ORANGE_HI = np.array([25, 255, 255], np.uint8)
COLOR_MIN_AREA = 80          # px^2, minsta blob (~10 px diameter)

# IMX219 full-FOV vid CV-upplösningen (approx; förfinas med kalibrering)
CV_W, CV_H = 640, 480
HFOV_DEG, VFOV_DEG = 62.2, 48.8
FX = (CV_W / 2) / math.tan(math.radians(HFOV_DEG) / 2)
FY = (CV_H / 2) / math.tan(math.radians(VFOV_DEG) / 2)
CX, CY = CV_W / 2.0, CV_H / 2.0

PHASE_COLOR, PHASE_ARUCO, PHASE_RTK = "COLOR", "ARUCO", "RTK-HOLD"


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
        mask = cv2.inRange(hsv, ORANGE_LO, ORANGE_HI)
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
