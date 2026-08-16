"""RTK-korrektionsstatus för droneweb.

Verifierar att hemma-basstationens NTRIP-caster (homebase:2101/LOCAL) är ansluten
och skickar RTCM, samt att injektorn (mavproxy-ntrip) kör. GPS-fix (RTK-läge) läses
separat ur telemetrin (app.py) och visar att FC:n faktiskt använder korrektionerna.

Bakgrundstråd gör en kort NTRIP-koll var PERIOD sekund (låg last: läser ~READ_S s
för att mäta RTCM-flödet, stänger sedan). Samma väg/caster som mavproxy använder, så
en lyckad koll = basen når drönaren och korrektionerna strömmar.
"""
import base64
import socket
import subprocess
import threading
import time

CASTER, PORT, MOUNT = "homebase", 2101, "LOCAL"   # matchar mavproxy-ntrip.service
USER, PASSWD = "anon", "anon"
INJECTOR_UNIT = "mavproxy-ntrip.service"
PERIOD = 10.0        # s mellan koll (kort → responsiv korrektionsålder; ~1 kB/koll)
READ_S = 1.2         # s att mäta RTCM-flöde


class RtkMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._st = {"base_ok": False, "base_bps": 0, "injector": False,
                    "mount": MOUNT, "caster": CASTER, "checked": None,
                    "last_rtcm": None, "err": None}
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            ok, bps, err = self._probe()
            inj = self._injector_active()
            now = time.time()
            upd = {"base_ok": ok, "base_bps": bps, "injector": inj,
                   "checked": round(now, 1), "err": err}
            if ok and bps > 0:                 # korrektioner strömmade → uppdatera åldern
                upd["last_rtcm"] = now
            with self._lock:
                self._st.update(upd)
            time.sleep(PERIOD)

    def _injector_active(self):
        try:
            r = subprocess.run(["systemctl", "is-active", INJECTOR_UNIT],
                               capture_output=True, timeout=4, text=True)
            return r.stdout.strip() == "active"
        except Exception:
            return False

    def _probe(self):
        """Return (got_200, rtcm_bytes_per_s, err). got_200 = caster+mount nås."""
        s = None
        try:
            s = socket.create_connection((CASTER, PORT), timeout=6)
            auth = base64.b64encode(("%s:%s" % (USER, PASSWD)).encode()).decode()
            req = ("GET /%s HTTP/1.1\r\nHost: %s:%d\r\n"
                   "Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP droneweb\r\n"
                   "Authorization: Basic %s\r\n\r\n" % (MOUNT, CASTER, PORT, auth))
            s.sendall(req.encode())
            s.settimeout(6)
            first = s.recv(256).split(b"\r\n", 1)[0]
            if b"200" not in first:            # ICY 200 OK / HTTP/1.1 200 OK
                return False, 0, first.decode(errors="replace")[:48] or "no response"
            s.settimeout(READ_S)
            n, t0 = 0, time.time()
            try:
                while time.time() - t0 < READ_S:
                    b = s.recv(4096)
                    if not b:
                        break
                    n += len(b)
            except socket.timeout:
                pass
            return True, int(n / max(0.1, time.time() - t0)), None
        except Exception as e:
            return False, 0, str(e)[:48]
        finally:
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass

    def status(self):
        with self._lock:
            s = dict(self._st)
        s["rtcm_age"] = round(time.time() - s["last_rtcm"], 1) if s["last_rtcm"] else None
        return s
