"""RTK-korrektionsstatus för droneweb.

Håller en IHÅLLANDE NTRIP-anslutning till hemma-basen (homebase:2101/LOCAL, samma
väg som mavproxy) och läser RTCM kontinuerligt → sann sub-sekund-ålder på senaste
korrektion + rullande byte-takt. Återansluter automatiskt om länken tappar.
En separat lätt tråd kollar att injektorn (mavproxy-ntrip) kör. GPS-fix läses ur
telemetrin i app.py (bevis att FC:n faktiskt använder korrektionerna).
"""
import base64
import socket
import subprocess
import threading
import time
from collections import deque

CASTER, PORT, MOUNT = "homebase", 2101, "LOCAL"   # matchar mavproxy-ntrip.service
USER, PASSWD = "anon", "anon"
INJECTOR_UNIT = "mavproxy-ntrip.service"
RATE_WIN = 5.0        # s fönster för byte-takt
STALE_S = 5.0         # ingen RTCM på så länge → betrakta länken som tappad, återanslut
RECONNECT_S = 5.0     # paus mellan återanslutningsförsök
INJ_PERIOD = 10.0     # s mellan injektor-koll


class RtkMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._st = {"base_ok": False, "base_bps": 0, "injector": False,
                    "mount": MOUNT, "caster": CASTER, "last_rtcm": None, "err": None}
        self._events = deque()      # (t, nbytes) — bara läs-tråden rör denna
        threading.Thread(target=self._run, daemon=True).start()
        threading.Thread(target=self._inj_loop, daemon=True).start()

    def _inj_loop(self):
        while True:
            try:
                r = subprocess.run(["systemctl", "is-active", INJECTOR_UNIT],
                                   capture_output=True, timeout=4, text=True)
                inj = r.stdout.strip() == "active"
            except Exception:
                inj = False
            with self._lock:
                self._st["injector"] = inj
            time.sleep(INJ_PERIOD)

    def _bps(self, now):
        while self._events and now - self._events[0][0] > RATE_WIN:
            self._events.popleft()
        return int(sum(n for _, n in self._events) / RATE_WIN)

    def _connect(self):
        s = socket.create_connection((CASTER, PORT), timeout=8)
        auth = base64.b64encode(("%s:%s" % (USER, PASSWD)).encode()).decode()
        req = ("GET /%s HTTP/1.1\r\nHost: %s:%d\r\n"
               "Ntrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIP droneweb\r\n"
               "Authorization: Basic %s\r\n\r\n" % (MOUNT, CASTER, PORT, auth))
        s.sendall(req.encode())
        s.settimeout(8)
        first = s.recv(256).split(b"\r\n", 1)[0]
        if b"200" not in first:               # ICY 200 OK / HTTP/1.1 200 OK
            s.close()
            raise IOError(first.decode(errors="replace")[:48] or "no 200")
        return s

    def _run(self):
        while True:
            s = None
            try:
                s = self._connect()
                with self._lock:
                    self._st["err"] = None
                s.settimeout(STALE_S)
                while True:
                    b = s.recv(4096)
                    if not b:
                        raise IOError("stängd av caster")
                    now = time.time()
                    self._events.append((now, len(b)))
                    with self._lock:
                        self._st["last_rtcm"] = now
                        self._st["base_ok"] = True
                        self._st["base_bps"] = self._bps(now)
            except Exception as e:
                self._events.clear()
                with self._lock:
                    self._st["base_ok"] = False
                    self._st["base_bps"] = 0
                    self._st["err"] = str(e)[:48]
            finally:
                if s is not None:
                    try:
                        s.close()
                    except Exception:
                        pass
            time.sleep(RECONNECT_S)

    def status(self):
        with self._lock:
            s = dict(self._st)
        s["rtcm_age"] = round(time.time() - s["last_rtcm"], 1) if s["last_rtcm"] else None
        return s
