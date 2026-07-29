"""Genererar landningsplattan för droneweb precisionslandning (Steg 2).

Producerar två PNG:er (40 px/cm):
  - marker_40cm.png    : 40 cm vit kvadrat med 30 cm ArUco (DICT_4X4_50, ID 0) centrerad.
                         Skriv ut i 40 cm — den vita kanten är ArUco:ns quiet-zone.
  - pad_full_62cm.png  : hela plattan (62 cm orange cirkel + vit kvadrat + markör) som referens.

Kör på Pi:n:  /home/axel/droneweb-venv/bin/python generate_marker.py
(cv2 finns i venv:en). ArUco-ordbok/ID måste matcha precland.py.
"""
import cv2
import numpy as np

PX_PER_CM = 40
MARKER_CM = 30          # svart ArUco-mönster
WHITE_CM = 40           # vit kvadrat (quiet-zone runt markören)
CIRCLE_CM = 62          # orange cirkel (ytterdiameter)
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_ID = 0
ORANGE_BGR = (0, 100, 255)   # safety-orange (R255 G100 B0) — matcha HSV i precland.py

dic = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
mpx = MARKER_CM * PX_PER_CM
marker = cv2.cvtColor(cv2.aruco.generateImageMarker(dic, ARUCO_ID, mpx), cv2.COLOR_GRAY2BGR)

# 1) markör centrerad på vit kvadrat
wpx = WHITE_CM * PX_PER_CM
white = np.full((wpx, wpx, 3), 255, np.uint8)
o = (wpx - mpx) // 2
white[o:o + mpx, o:o + mpx] = marker
cv2.imwrite("/tmp/marker_40cm.png", white)

# 2) hela plattan: orange fylld cirkel + vit kvadrat + markör
cpx = CIRCLE_CM * PX_PER_CM
pad = np.full((cpx, cpx, 3), 255, np.uint8)
cv2.circle(pad, (cpx // 2, cpx // 2), cpx // 2, ORANGE_BGR, -1)
o2 = (cpx - wpx) // 2
pad[o2:o2 + wpx, o2:o2 + wpx] = white
cv2.imwrite("/tmp/pad_full_62cm.png", pad)

print("OK: marker_40cm.png", white.shape[:2], "| pad_full_62cm.png", pad.shape[:2],
      "| dict=DICT_4X4_50 id=%d" % ARUCO_ID)
