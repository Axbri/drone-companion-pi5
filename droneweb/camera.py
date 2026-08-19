"""IMX219-kamera via picamera2 för droneweb.

Äger kameran (libcamera tillåter bara en process). Två strömmar:
  - main  (4:3, webbvideo) → MJPEGEncoder (HW) → StreamingOutput → /video.mjpg
  - lores (640x480 YUV420) → Y-planet = gråskala för ArUco-CV (Steg 2)

On-demand via referensräkning:
  - "hold"   håller kameran igång (video-tittare + CV-hold i Steg 2)
  - "viewer" håller MJPEG-encodern igång (bara när någon tittar på videon)
Så när ingen tittar och CV är av → kameran stoppas helt (spar CPU/ström på Pi 3A+).
"""
import io
import threading

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

SENSOR_MODEL = "imx219"   # precland-kameran; Pi 5 har numera även en imx477 (12MP) på den
                          # andra CSI-porten — Picamera2() utan argument tar Num 0, vilket INTE
                          # längre är garanterat imx219, så vi slår upp rätt kamera via modellnamn.

MAIN_SIZE = (1024, 768)   # 4:3, full FOV (IMX219 är 4:3)
LORES_SIZE = (640, 480)   # gråskala-Y för CV
FPS = 40                  # Pi 5-tuning 2026-08-18: 25 var en Pi 3A+-gräns (CPU-bunden backlog vid
                          # buffer_count=2 + FrameRate 40 där). Roten var buffer_count, inte FPS —
                          # med buffer_count=1 (nedan) föll latensen på Pi 5 från ~23ms@25Hz till
                          # ~19-21ms@40Hz. >40 gav mer jitter (GIL-delning med webb-pollningen i
                          # app.js) utan lägre latens — 40 är den stabila sweetspoten. Se PI5-SETUP.md.
                          # (höjd FPS påverkar även FPV-videon → mer 4G-data när man tittar; ofarligt)


class StreamingOutput(io.BufferedIOBase):
    """Tar emot JPEG-frames från MJPEGEncoder och delar senaste till läsare."""

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
    def __init__(self):
        self._picam2 = Picamera2(_find_camera_num())
        cfg = self._picam2.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "YUV420"},
            lores={"size": LORES_SIZE, "format": "YUV420"},
            controls={"FrameRate": FPS},
            buffer_count=1,   # låg buffert → capture_request ger färsk ruta (min latens), inte en
                              # kö av gamla. Vi kopierar+släpper direkt så 1 räcker (2 gav backlog).
        )
        self._picam2.configure(cfg)
        self._output = StreamingOutput()
        self._lock = threading.Lock()
        self._holds = 0      # allt som behöver kameran igång
        self._viewers = 0    # MJPEG-tittare (behöver encodern)
        self._started = False
        self._encoding = False
        self._external = False   # precland matar videon (annoterade frames)

    # ---- referensräkning ------------------------------------------------
    def _acquire_hold(self):
        with self._lock:
            self._holds += 1
            if not self._started:
                self._picam2.start()
                self._started = True

    def _release_hold(self):
        with self._lock:
            self._holds = max(0, self._holds - 1)
            if self._holds == 0 and self._started:
                if self._encoding:
                    self._picam2.stop_encoder()
                    self._encoding = False
                self._picam2.stop()
                self._started = False

    def _acquire_viewer(self):
        self._acquire_hold()
        with self._lock:
            self._viewers += 1
            if not self._encoding and not self._external:
                self._picam2.start_encoder(
                    MJPEGEncoder(), FileOutput(self._output), name="main"
                )
                self._encoding = True

    def _release_viewer(self):
        with self._lock:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers == 0 and self._encoding:
                self._picam2.stop_encoder()
                self._encoding = False
        self._release_hold()

    # ---- publikt API ----------------------------------------------------
    def frames(self):
        """Generator: JPEG-bytes för MJPEG-strömmen. Startar/stoppar on-demand."""
        self._acquire_viewer()
        try:
            while True:
                with self._output.condition:
                    if not self._output.condition.wait(timeout=5):
                        continue
                    frame = self._output.frame
                if frame:
                    yield frame
        finally:
            self._release_viewer()

    def cv_hold(self):
        """Kontexthanterare som håller kameran igång för CV (Steg 2)."""
        cam = self

        class _Hold:
            def __enter__(self):
                cam._acquire_hold()
                return cam

            def __exit__(self, *a):
                cam._release_hold()

        return _Hold()

    def capture_gray(self):
        """Gråskalebild (Y-planet ur lores YUV420) för ArUco. Kräver aktiv hold."""
        yuv = self._picam2.capture_array("lores")
        h, w = LORES_SIZE[1], LORES_SIZE[0]
        return yuv[:h, :w]

    def capture_lores(self):
        """Rå lores YUV420-array (för CV: Y-plan = gråskala, cvtColor→BGR för färg).
        Kräver aktiv hold."""
        return self._picam2.capture_array("lores")

    def capture_lores_ts(self):
        """(lores YUV420-array, SensorTimestamp i ns) för latensmätning — via
        capture_request så vi får kamerans fångst-tidsstämpel. Fallback: (arr, None)."""
        try:
            req = self._picam2.capture_request()
        except Exception:
            return self.capture_lores(), None
        try:
            arr = req.make_array("lores")
            ts = req.get_metadata().get("SensorTimestamp")
        finally:
            req.release()
        return arr, ts

    def push_frame(self, jpeg_bytes):
        """Skriv en färdig JPEG till videoströmmen (precland-annoterad bild)."""
        with self._output.condition:
            self._output.frame = jpeg_bytes
            self._output.condition.notify_all()

    def set_external_source(self, on):
        """on: stäng HW-MJPEG-encodern och låt push_frame() mata videon (precland-läge).
        off: återuppta HW-MJPEG om det finns tittare."""
        with self._lock:
            self._external = bool(on)
            if on and self._encoding:
                self._picam2.stop_encoder()
                self._encoding = False
            elif not on and self._viewers > 0 and self._started and not self._encoding:
                self._picam2.start_encoder(
                    MJPEGEncoder(), FileOutput(self._output), name="main"
                )
                self._encoding = True

    def set_cv_exposure(self, on, exposure_us=2000, gain=2.0):
        """Steg 2: fast exponering under CV-loopen (fryser rörelse → skarp jämn
        detektering, mot motion blur på höjd — Fynd 1). Kort slutartid gör rutan
        skarp trots kameravibration. off = åter till auto-exponering (AeEnable=True)
        för normal FPV-video. Kräver att kameran är startad (aktiv hold)."""
        with self._lock:
            if not self._started:
                return
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
