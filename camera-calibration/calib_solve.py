"""Solve the IMX296 lens calibration from calib_capture.py frames. Run on the Pi:

  /home/axel/droneweb-venv/bin/python calib_solve.py [--dir /home/axel/calib]
        [--square 35.0] [--out /home/axel/camera_calib_imx296.json]

Fits both the OpenCV fisheye model (Kannala-Brandt, 4 coefficients — right for wide
lenses) and the plain pinhole model (k1..k3 + p1 p2), drops views with outlier
reprojection error, and writes the model with the lower RMS as JSON for precland.py:

  {"model": "fisheye"|"pinhole", "size": [w, h], "K": 3x3, "D": [...], "rms": px, ...}

Also writes <out>.undistorted.jpg (last view, undistorted) for a visual sanity check.
"""
import argparse
import glob
import json
import math
import os
import time

import cv2
import numpy as np

from calib_board import make_board


def collect(paths, board):
    detector = cv2.aruco.CharucoDetector(board)
    obj_all, img_all, used = [], [], []
    for p in paths:
        gray = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < 6:
            continue
        obj, img = board.matchImagePoints(ch_corners, ch_ids)
        obj_all.append(obj.astype(np.float64)); img_all.append(img.astype(np.float64)); used.append(p)
    return obj_all, img_all, used, gray.shape[::-1]


def per_view_error(obj, img, K, D, rvecs, tvecs, fisheye):
    errs = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        if fisheye:
            proj, _ = cv2.fisheye.projectPoints(o.reshape(-1, 1, 3), r, t, K, D)
        else:
            proj, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
    return errs


def solve_pinhole(obj, img, size):
    o = [x.astype(np.float32) for x in obj]; i = [x.astype(np.float32) for x in img]
    rms, K, D, rv, tv = cv2.calibrateCamera(o, i, size, None, None)
    return rms, K, D.ravel(), rv, tv


def solve_fisheye(obj, img, size):
    K = np.zeros((3, 3)); D = np.zeros((4, 1))
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-8)
    o = [x.reshape(-1, 1, 3) for x in obj]; i = [x.reshape(-1, 1, 2) for x in img]
    rms, K, D, rv, tv = cv2.fisheye.calibrate(o, i, size, K, D, flags=flags, criteria=crit)
    return rms, K, D.ravel(), rv, tv


def fit(name, solver, obj, img, size, fisheye):
    """Solve, drop views > 3x median error (or > 2 px), solve again."""
    rms, K, D, rv, tv = solver(obj, img, size)
    errs = per_view_error(obj, img, K, D, rv, tv, fisheye)
    thr = max(2.0, 3 * float(np.median(errs)))
    keep = [k for k, e in enumerate(errs) if e <= thr]
    if len(keep) < len(obj) and len(keep) >= 6:
        rms, K, D, rv, tv = solver([obj[k] for k in keep], [img[k] for k in keep], size)
        errs = per_view_error([obj[k] for k in keep], [img[k] for k in keep], K, D, rv, tv, fisheye)
    print(f"{name:8s} rms {rms:.3f} px  views {len(keep)}/{len(obj)}  "
          f"worst view {max(errs):.2f} px  f=({K[0,0]:.1f},{K[1,1]:.1f}) c=({K[0,2]:.1f},{K[1,2]:.1f})  D={np.round(D,4).tolist()}")
    return rms, K, D, len(keep)


def edge_angle(K, D, size, fisheye, pt):
    p = np.array(pt, np.float64).reshape(1, 1, 2)
    n = (cv2.fisheye.undistortPoints(p, K, D) if fisheye else cv2.undistortPoints(p, K, D)).ravel()
    return math.degrees(math.atan(n[0])), math.degrees(math.atan(n[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/home/axel/calib")
    ap.add_argument("--square", type=float, default=35.0, help="measured square size, mm")
    ap.add_argument("--out", default="/home/axel/camera_calib_imx296.json")
    args = ap.parse_args()

    board = make_board(args.square)
    paths = sorted(glob.glob(os.path.join(args.dir, "calib_*.png")))
    obj, img, used, size = collect(paths, board)
    print(f"{len(used)}/{len(paths)} views usable, image size {size}")
    if len(used) < 10:
        raise SystemExit("need at least 10 usable views")

    results = {}
    for name, solver, fe in (("pinhole", solve_pinhole, False), ("fisheye", solve_fisheye, True)):
        try:
            results[name] = fit(name, solver, obj, img, size, fe)
        except cv2.error as e:
            print(f"{name}: failed ({str(e).splitlines()[0]})")
    best = min(results, key=lambda k: results[k][0])
    rms, K, D, n = results[best]
    fe = best == "fisheye"
    w, h = size
    hx = edge_angle(K, D, size, fe, (w - 1, h / 2))[0] - edge_angle(K, D, size, fe, (0, h / 2))[0]
    vy = edge_angle(K, D, size, fe, (w / 2, h - 1))[1] - edge_angle(K, D, size, fe, (w / 2, 0))[1]
    out = {"model": best, "size": [w, h], "K": np.round(K, 4).tolist(), "D": np.round(D, 6).tolist(),
           "rms": round(rms, 4), "n_views": n, "square_mm": args.square,
           "hfov_deg": round(hx, 2), "vfov_deg": round(vy, 2),
           "date": time.strftime("%Y-%m-%d"), "sensor": "imx296"}
    with open(args.out, "w") as f:
        json.dump(out, f, indent=1)
    print(f"-> {args.out}: {best}, rms {rms:.3f} px, HFOV {hx:.1f} VFOV {vy:.1f} (pinhole-equivalent at edges)")

    gray = cv2.imread(used[-1], cv2.IMREAD_GRAYSCALE)
    if fe:
        newK = K.copy(); newK[0, 0] *= 0.7; newK[1, 1] *= 0.7   # zoom out so edges stay visible
        m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, size, cv2.CV_16SC2)
        und = cv2.remap(gray, m1, m2, cv2.INTER_LINEAR)
    else:
        und = cv2.undistort(gray, K, D)
    cv2.imwrite(args.out + ".undistorted.jpg", np.hstack([gray, und]))


if __name__ == "__main__":
    main()
