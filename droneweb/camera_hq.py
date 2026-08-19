"""imx477 (HQ, 12MP) camera via picamera2 — FPV video only, for the Pilot view tab.

No lores stream, no CV, no external-source hook (that's all camera.py/imx219, the
precland camera). Same on-demand start/stop via reference counting as camera.py, so
the sensor is idle (no CPU/bandwidth) whenever nobody is on the Pilot view tab.
"""
import threading

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

from camera import StreamingOutput

SENSOR_MODEL = "imx477"
MAIN_SIZE = (1296, 972)   # 4:3, downscaled from 4056x3040 for a reasonable FPV bitrate/CPU cost
FPS = 30


def _find_camera_num(model=SENSOR_MODEL):
    """Find the Picamera2 index for a given sensor model, so this always opens the
    HQ camera regardless of which CSI port it's plugged into. See camera.py."""
    for info in Picamera2.global_camera_info():
        if info.get("Model") == model:
            return info["Num"]
    print(f"[camera_hq] WARNING: no {model} camera found, falling back to Num 0 "
          f"({[i.get('Model') for i in Picamera2.global_camera_info()]})")
    return 0


class HQCamera:
    def __init__(self):
        self._picam2 = Picamera2(_find_camera_num())
        cfg = self._picam2.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "YUV420"},
            controls={"FrameRate": FPS},
            buffer_count=2,
        )
        self._picam2.configure(cfg)
        self._output = StreamingOutput()
        self._lock = threading.Lock()
        self._viewers = 0
        self._started = False
        # exposure (adjustable live from the web UI; re-applied whenever the camera
        # (re)starts, since it's stopped/started on demand per Pilot view tab visits)
        self._auto_exposure = True
        self._exposure_us = 2000
        self._gain = 2.0

    def _acquire(self):
        with self._lock:
            self._viewers += 1
            if not self._started:
                self._picam2.start()
                self._picam2.start_encoder(MJPEGEncoder(), FileOutput(self._output))
                self._started = True
                self._apply_exposure_locked()

    def _release(self):
        with self._lock:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers == 0 and self._started:
                self._picam2.stop_encoder()
                self._picam2.stop()
                self._started = False

    def frames(self):
        """Generator: JPEG bytes for the MJPEG stream. Starts/stops on demand."""
        self._acquire()
        try:
            while True:
                with self._output.condition:
                    if not self._output.condition.wait(timeout=5):
                        continue
                    frame = self._output.frame
                if frame:
                    yield frame
        finally:
            self._release()

    @property
    def viewers(self):
        return self._viewers

    def _apply_exposure_locked(self):
        """Push the current exposure mode to the camera. No-op if not started (caller
        holds self._lock); applied again on the next start."""
        if not self._started:
            return
        if self._auto_exposure:
            self._picam2.set_controls({"AeEnable": True})
        else:
            self._picam2.set_controls({
                "AeEnable": False,
                "ExposureTime": int(self._exposure_us),
                "AnalogueGain": float(self._gain),
            })

    def exposure_status(self):
        return {"auto": self._auto_exposure, "exposure_us": self._exposure_us,
                "gain": round(self._gain, 1)}

    def set_exposure(self, auto=None, exposure_us=None, gain=None):
        """Set exposure live from the web UI. Clamps values and applies immediately
        if the camera is running; otherwise applied on the next start."""
        with self._lock:
            if auto is not None:
                self._auto_exposure = bool(auto)
            if exposure_us is not None:
                self._exposure_us = int(max(100, min(30000, exposure_us)))
            if gain is not None:
                self._gain = float(max(1.0, min(16.0, gain)))
            self._apply_exposure_locked()
        return self.exposure_status()
