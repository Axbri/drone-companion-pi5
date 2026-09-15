"""Capture ChArUco views from the IMX296 for lens calibration. Run on the Pi with the
camera free:

  sudo systemctl stop droneweb
  /home/axel/droneweb-venv/bin/python calib_capture.py [--n 40] [--out /home/axel/calib]
  sudo systemctl start droneweb

Live view with overlay: http://dronepi5.local:8081/  — green = corners in the current
frame, blue dots = corners already captured (aim to cover the whole frame, especially
the corners and edges, with the board tilted in all directions and at 2-3 distances).
A frame is saved automatically when the board is seen with enough corners and has
moved since the last capture. Stops after --n frames or Ctrl-C.
"""
import argparse
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2

from calib_board import make_board

SENSOR_MODEL = "imx296"
SIZE = (1456, 1088)          # native — calibration must be done at the resolution precland uses
MIN_CORNERS = 8
MIN_MOVE_PX = 50             # mean corner displacement before a new frame is accepted
MIN_INTERVAL_S = 0.7

latest_jpeg = None
jpeg_lock = threading.Condition()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    with jpeg_lock:
                        jpeg_lock.wait(timeout=2)
                        buf = latest_jpeg
                    if buf:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b'<body style="margin:0;background:#000"><img src="/stream.mjpg" '
                             b'style="width:100vw;max-height:100vh;object-fit:contain"></body>')


def camera_num():
    for info in Picamera2.global_camera_info():
        if info.get("Model") == SENSOR_MODEL:
            return info["Num"]
    raise SystemExit(f"no {SENSOR_MODEL} camera found")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--out", default="/home/axel/calib")
    ap.add_argument("--port", type=int, default=8081)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    detector = cv2.aruco.CharucoDetector(make_board())
    cam = Picamera2(camera_num())
    cam.configure(cam.create_video_configuration(
        main={"size": SIZE, "format": "YUV420"}, controls={"FrameRate": 30}, buffer_count=2))
    cam.start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"live view: http://dronepi5.local:{args.port}/   saving to {args.out}", flush=True)

    global latest_jpeg
    saved = len([f for f in os.listdir(args.out) if f.endswith(".png")])
    coverage = []            # all corners captured so far, for the overlay
    last_pts, last_t = None, 0.0
    W, H = SIZE
    try:
        while saved < args.n:
            gray = cam.capture_array("main")[:H, :W]
            ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
            n = 0 if ch_ids is None else len(ch_ids)
            accepted = False
            if n >= MIN_CORNERS:
                pts = ch_corners.reshape(-1, 2)
                moved = last_pts is None or np.linalg.norm(pts.mean(0) - last_pts.mean(0)) > MIN_MOVE_PX
                if moved and time.time() - last_t > MIN_INTERVAL_S:
                    saved += 1
                    cv2.imwrite(os.path.join(args.out, f"calib_{saved:03d}.png"), gray)
                    coverage.extend(pts.tolist())
                    last_pts, last_t, accepted = pts, time.time(), True
                    print(f"saved {saved}/{args.n}  corners {n}", flush=True)

            small = cv2.cvtColor(cv2.resize(gray, (W // 2, H // 2)), cv2.COLOR_GRAY2BGR)
            for x, y in coverage:
                cv2.circle(small, (int(x / 2), int(y / 2)), 2, (255, 120, 0), -1)
            if n:
                for x, y in ch_corners.reshape(-1, 2):
                    cv2.circle(small, (int(x / 2), int(y / 2)), 4, (0, 255, 0), 1)
            col = (0, 255, 255) if accepted else (255, 255, 255)
            cv2.putText(small, f"saved {saved}/{args.n}   corners {n}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
            ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
            with jpeg_lock:
                latest_jpeg = buf.tobytes()
                jpeg_lock.notify_all()
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.close()
        srv.shutdown()
    print(f"done: {saved} frames in {args.out}", flush=True)
    os._exit(0)     # picamera2/stream threads otherwise keep the process alive after "done"


if __name__ == "__main__":
    main()
