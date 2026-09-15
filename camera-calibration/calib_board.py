"""ChArUco board definition shared by calib_capture.py / calib_solve.py, plus a print
generator. Run on the Pi (cv2 lives in the venv):

  /home/axel/droneweb-venv/bin/python calib_board.py   -> /tmp/charuco_a4.png

Print charuco_a4.png on A4 landscape at 100 % / "actual size" (no fit-to-page), then
MEASURE one square with a ruler and pass --square <mm> to calib_solve.py if it is not
35.0 mm. Dictionary is deliberately not DICT_4X4_50 (the landing marker's), so a stray
landing marker in view can never be mistaken for board markers.
"""
import cv2

SQUARES_X, SQUARES_Y = 7, 5          # 7x5 squares = 245x175 mm at 35 mm — fits A4 landscape
SQUARE_MM = 35.0
MARKER_MM = 26.0                     # ~0.75 of the square
DICT = cv2.aruco.DICT_5X5_100

DPI = 300
A4_PX = (int(297 / 25.4 * DPI), int(210 / 25.4 * DPI))   # landscape (w, h)


def make_board(square_mm=SQUARE_MM):
    """CharucoBoard in metres (square size is the only thing the measurement changes)."""
    scale = square_mm / SQUARE_MM
    return cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), square_mm / 1000.0, MARKER_MM * scale / 1000.0,
        cv2.aruco.getPredefinedDictionary(DICT))


if __name__ == "__main__":
    import numpy as np
    sq_px = int(round(SQUARE_MM / 25.4 * DPI))
    board_px = (SQUARES_X * sq_px, SQUARES_Y * sq_px)
    img = make_board().generateImage(board_px, marginSize=0, borderBits=1)
    page = np.full((A4_PX[1], A4_PX[0]), 255, np.uint8)
    x0, y0 = (A4_PX[0] - board_px[0]) // 2, (A4_PX[1] - board_px[1]) // 2
    page[y0:y0 + board_px[1], x0:x0 + board_px[0]] = img
    cv2.putText(page, "IMX296 calibration board  7x5 squares, 35 mm  DICT_5X5_100  print 100%",
                (x0, A4_PX[1] - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    cv2.imwrite("/tmp/charuco_a4.png", page)
    print("OK /tmp/charuco_a4.png", page.shape, "square px", sq_px)
