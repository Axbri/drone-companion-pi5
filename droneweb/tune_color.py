"""Kalibrera HSV-trösklarna för orange/röda landningsplattan mot verkligt ljus.

Tar en bild från droneweb-kameran (via MJPEG-strömmen), hittar plattan via
ArUco-markören, och samplar de faktiska pixlarna i den orange/röda ringen runt
markören → skriver ut uppmätt HSV + rekommenderade ORANGE-trösklar för precland.py.

Kör på Pi:n:  /home/axel/droneweb-venv/bin/python /home/axel/droneweb/tune_color.py
Kräver att hela plattan (markör + färgring) syns i kameran. Spara även debug-bilder
(/tmp/tune_frame.png, /tmp/tune_mask.png). Obs: hue-wrap — röd ligger nära H=0 OCH 180.
"""
import urllib.request
import cv2
import numpy as np


def grab_frame(url="http://localhost:8080/video.mjpg"):
    s = urllib.request.urlopen(url, timeout=6)
    buf = b""
    for _ in range(400):
        buf += s.read(8192)
        a = buf.find(b"\xff\xd8"); b = buf.rfind(b"\xff\xd9")
        if a != -1 and b > a:
            f = cv2.imdecode(np.frombuffer(buf[a:b + 2], np.uint8), cv2.IMREAD_COLOR)
            if f is not None:
                return f
    return None


def main():
    frame = grab_frame()
    if frame is None:
        print("FEL: fick ingen bild"); return
    H, W = frame.shape[:2]
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if ids is None:
        print("INGEN markör i bild — rikta kameran så hela plattan syns"); return
    pts = corners[0].reshape(4, 2)
    cx, cy = pts.mean(0)
    r = np.mean([np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]) / 2.0

    # sampla färgringen: cirkel(1.9r) minus vit kvadrat(1.5r halv)
    mask = np.zeros((H, W), np.uint8)
    cv2.circle(mask, (int(cx), int(cy)), int(1.9 * r), 255, -1)
    cv2.rectangle(mask, (int(cx - 1.5 * r), int(cy - 1.5 * r)),
                  (int(cx + 1.5 * r), int(cy + 1.5 * r)), 0, -1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    samp = hsv[mask == 255]
    print("samplade %d px runt plattan" % len(samp))
    for i, n in enumerate("HSV"):
        ch = samp[:, i]
        print("  %s: p2=%3d p50=%3d p98=%3d" %
              (n, np.percentile(ch, 2), np.percentile(ch, 50), np.percentile(ch, 98)))
    s2, v2 = int(np.percentile(samp[:, 1], 2)), int(np.percentile(samp[:, 2], 2))
    print("Sätt i precland.py: ORANGE_S_MIN≈%d  ORANGE_V_MIN≈%d  (H: wrap nära 0/180 om röd)"
          % (max(80, s2 - 40), max(60, v2 - 40)))
    cv2.imwrite("/tmp/tune_frame.png", frame)


if __name__ == "__main__":
    main()
