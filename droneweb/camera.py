"""IMX219-kamera via picamera2 för droneweb — alltid-på spårningskamera (Steg 3).

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

SENSOR_MODEL = "imx219"   # precland-kameran; Pi 5 har numera även en imx477 (12MP) på den
                          # andra CSI-porten — Picamera2() utan argument tar Num 0, vilket INTE
                          # längre är garanterat imx219, så vi slår upp rätt kamera via modellnamn.

CAPTURE_SIZE = (1024, 768)   # 4:3, full FOV (IMX219 är 4:3). Höjd från 640x480 2026-08-21 för
                              # längre ArUco-räckvidd (Pi 5 har gott om marginal jämfört med Pi 3A+
                              # som satte den gamla gränsen) — se PRECISION-LANDING.md.
FPS = 40                      # Pi 5-tuning 2026-08-18, se PI5-SETUP.md. EJ omverifierat vid den
                              # högre upplösningen (2026-08-21) — kontrollera loop_hz i webben.


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
