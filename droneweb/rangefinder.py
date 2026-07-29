"""TF-Luna-lidar via I2C — AGL för precland-faslogik + DISTANCE_SENSOR-relä till FC.

TF-Luna på Pi:ns I2C (bus 1, addr 0x10). Måste vara i I2C-läge (default är UART).
Registren: 0x00=Dist_L, 0x01=Dist_H (cm). Graciöst om sensorn saknas/ej svarar →
agl() returnerar None (då kör precland detektionsbaserad fas, funkar för bänktest).
"""
import threading
import time

try:
    import smbus2
except ImportError:
    smbus2 = None

I2C_BUS = 1
TFLUNA_ADDR = 0x10
MIN_CM, MAX_CM = 10, 800   # TF-Luna räckvidd (~0,1–8 m)


class RangeFinder:
    def __init__(self, tel=None, relay_hz=15):
        self.tel = tel                 # MavlinkTelemetry för DISTANCE_SENSOR-relä
        self._relay_dt = 1.0 / relay_hz
        self._dist_m = None
        self._ok = False
        self._lock = threading.Lock()
        self._bus = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _read_cm(self):
        d = self._bus.read_i2c_block_data(TFLUNA_ADDR, 0x00, 2)
        return d[0] | (d[1] << 8)

    def _run(self):
        if smbus2 is None:
            return  # inget I2C-bibliotek → agl() förblir None
        last_relay = 0.0
        while True:
            try:
                if self._bus is None:
                    self._bus = smbus2.SMBus(I2C_BUS)
                cm = self._read_cm()
                valid = MIN_CM <= cm <= MAX_CM
                with self._lock:
                    self._dist_m = (cm / 100.0) if valid else None
                    self._ok = True
                now = time.time()
                if self.tel and valid and now - last_relay >= self._relay_dt:
                    self.tel.send_distance_sensor(cm, MIN_CM, MAX_CM)
                    last_relay = now
                time.sleep(0.05)
            except Exception:
                with self._lock:
                    self._ok = False
                    self._dist_m = None
                self._bus = None
                time.sleep(1.0)

    def agl(self):
        with self._lock:
            return self._dist_m

    def status(self):
        with self._lock:
            return {"ok": self._ok, "agl": self._dist_m}
