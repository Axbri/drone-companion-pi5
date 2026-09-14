"""IMX296 camera (Raspberry Pi Global Shutter, mono) via picamera2 for droneweb —
always-on tracking camera (Steg 3).

Change 2026-09-14: IMX219 (rolling shutter, colour) replaced by IMX296 (global
shutter, mono, 1456x1088 @ max 60 fps, a single sensor mode). Global shutter = no
rolling-shutter skew of the marker under vibration/rotation; mono = the Y plane is
the sensor's real pixels (no Bayer interpolation). The lens is interchangeable
(C/CS mount) → FOV/focal length in precland.py MUST be recalibrated on a lens change.

Änd 2026-08-21: den separata rå-FPV-vyn (HW-MJPEG, för att bara titta på den
nedåtriktade kameran) är borttagen — den var bara till för att experimentera med
kameran innan precisionslandningssystemet fanns, och används inte i normal flygning
(FPV-flygning använder den andra kameran, imx477/camera_hq.py). All video från den
här kameran är nu spårningens egen annoterade förhandsvisning (precland.py), som
går kontinuerligt (ingen hold/viewer-styrd start/stopp av kameran längre — bara en
enda ström, startad en gång och igång för appens hela livstid).
"""
import io
import threading

from picamera2 import Picamera2

SENSOR_MODEL = "imx296"   # precland camera; the Pi 5 also has an imx477 (12MP, FPV) on the other
                          # CSI port — Picamera2() without an argument takes Num 0, which is not
                          # guaranteed to be the right camera, so look it up by model name.

CAPTURE_SIZE = (1024, 768)   # ISP downscales from the sensor's 1456x1088 (ScalerCrop 3,0,1450,1088 =
                              # full FOV). Kept from the IMX219 era: bench 2026-09-14 gave ArUco
                              # detection 46 ms/frame here (~15 Hz loop, same as the 2026-08-24
                              # flight log) vs 71 ms (~11 Hz) at native 1456x1088 — the range gain
                              # from full resolution does not justify the loop-rate drop.
FPS = 40                      # sensor does 60; 40 is plenty since the CV loop runs ~15 Hz and
                              # buffer_count=1 hands out the freshest frame anyway.


class StreamingOutput(io.BufferedIOBase):
    """Tar emot JPEG-frames (från precland-writer-tråden) och delar senaste till läsare."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def writable(self):
        return True

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def _find_camera_num(model=SENSOR_MODEL):
    """Hitta Picamera2-index för given sensormodell. Faller tillbaka till 0
    (med varning) om modellen inte hittas, t.ex. om kameran kopplats loss."""
    for info in Picamera2.global_camera_info():
        if info.get("Model") == model:
            return info["Num"]
    print(f"[camera] VARNING: hittade ingen {model}-kamera, faller tillbaka till Num 0 "
          f"({[i.get('Model') for i in Picamera2.global_camera_info()]})")
    return 0


class Camera:
    """Alltid-på: kameran startas i konstruktorn och går sedan för appens hela
    livstid — precisionslandningsspårningen (precland.py) körs kontinuerligt, så
    det finns ingen anledning att stoppa/starta den längre (se modul-docstring)."""

    def __init__(self):
        self._picam2 = Picamera2(_find_camera_num())
        cfg = self._picam2.create_video_configuration(
            main={"size": CAPTURE_SIZE, "format": "YUV420"},
            controls={"FrameRate": FPS},
            buffer_count=1,   # låg buffert → capture_request ger färsk ruta (min latens), inte en
                              # kö av gamla. Vi kopierar+släpper direkt så 1 räcker (2 gav backlog).
        )
        self._picam2.configure(cfg)
        self._picam2.start()
        self._output = StreamingOutput()
        self._lock = threading.Lock()
        self._viewers = 0    # hur många webbläsare som just nu hämtar /video.mjpg

    # ---- webb-videoström (bara referensräkning, ingen hårdvarupåverkan) -----
    def frames(self):
        """Generator: JPEG-bytes för MJPEG-strömmen (precland-writer-tråden matar den
        via push_frame()). Räknar tittare så writer-tråden vet om det är värt att
        annotera/skala/koda (se PrecLandController._writer_loop)."""
        with self._lock:
            self._viewers += 1
        try:
            while True:
                with self._output.condition:
                    if not self._output.condition.wait(timeout=5):
                        continue
                    frame = self._output.frame
                if frame:
                    yield frame
        finally:
            with self._lock:
                self._viewers = max(0, self._viewers - 1)

    def capture_ts(self):
        """(YUV420-array, SensorTimestamp i ns) för latensmätning — via capture_request
        så vi får kamerans fångst-tidsstämpel. Fallback: (arr, None)."""
        try:
            req = self._picam2.capture_request()
        except Exception:
            return self._picam2.capture_array("main"), None
        try:
            arr = req.make_array("main")
            ts = req.get_metadata().get("SensorTimestamp")
        finally:
            req.release()
        return arr, ts

    def push_frame(self, jpeg_bytes):
        """Skriv en färdig JPEG till videoströmmen (precland-annoterad bild)."""
        with self._output.condition:
            self._output.frame = jpeg_bytes
            self._output.condition.notify_all()

    def set_cv_exposure(self, on, exposure_us=2000, gain=2.0):
        """Fast exponering mot motion blur (Fynd 1). Kort slutartid gör rutan skarp
        trots kameravibration. off = auto-exponering (AeEnable=True)."""
        if on:
            self._picam2.set_controls({
                "AeEnable": False,
                "ExposureTime": int(exposure_us),
                "AnalogueGain": float(gain),
            })
        else:
            self._picam2.set_controls({"AeEnable": True})

    @property
    def viewers(self):
        return self._viewers
