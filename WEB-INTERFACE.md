# droneweb — companion-webbgränssnitt

Ett webbgränssnitt på `dronepi` som visar **kameravideo** (stor), **telemetri**
(batteri, roll, pitch, heading), **Pi-prestanda** (CPU, RAM, nät, temp) och en
**avstängningsknapp**. Nås över LAN/Tailscale — ingen inloggning (betrott nät).

Steg 2 (precisionslandning, ArUco) byggs in i samma app — se
[`PRECISION-LANDING.md`](PRECISION-LANDING.md).

## Åtkomst

`http://dronepi:8080` (eller `http://100.110.147.30:8080`) från en dator/telefon
på samma tailnet eller LAN.

## Arkitektur

En enda Python/Flask-app (`droneweb/`) **äger IMX219-kameran** via picamera2 (libcamera
tillåter bara en process → ersätter mediamtx/`dronecam.service`). Två kameraströmmar:
`main` 1024×768 (webbvideo, HW-MJPEG) och `lores` 640×480 (gråskala för CV i Steg 2).

```
IMX219 ─ picamera2 ─┬─ main  → MJPEGEncoder → /video.mjpg  (on-demand: encodar bara när någon tittar)
                    └─ lores → ArUco-CV (Steg 2)
mavproxy-ntrip (--out udpin:127.0.0.1:14551) → pymavlink-tråd → /api/telemetry
psutil / vcgencmd ───────────────────────────────────────────→ /api/stats
```

- `camera.py` — picamera2, on-demand via referensräkning (kameran stoppas helt när ingen
  tittar och CV är av → noll CPU/ström).
- `mavlink.py` — bakgrundstråd, läser ATTITUDE/VFR_HUD/SYS_STATUS/HEARTBEAT/GPS_RAW_INT.
- `app.py` — Flask: `/`, `/video.mjpg`, `/api/telemetry` (~5 Hz), `/api/stats` (~1 Hz),
  `/api/shutdown` (POST).
- `templates/` + `static/` — mörkt dashboard, video fyller sidan, panel med telemetri +
  Pi-stats, attityd-indikator (canvas), röd avstängningsknapp (bekräfta-dialog).

## Installation (på Pi:n)

```bash
# 1. Systempaket (delade libbar → snålt med RAM på 512 MB)
sudo apt install -y python3-picamera2 python3-opencv python3-numpy \
                    python3-flask python3-psutil python3-venv

# 2. Venv som ser systempaketen + pymavlink (finns ej i apt)
python3 -m venv --system-site-packages ~/droneweb-venv
~/droneweb-venv/bin/pip install pymavlink

# 3. Appfiler
mkdir -p ~/droneweb && cp -r droneweb/* ~/droneweb/

# 4. mavproxy: lägg till droneweb-telemetri-endpoint
sudo install -m 644 systemd/mavproxy-ntrip.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl restart mavproxy-ntrip

# 5. Kameran: droneweb äger den nu → stäng av mediamtx
sudo systemctl disable --now dronecam

# 6. Avstängningsknapp (endast /sbin/shutdown, inget annat root)
sudo install -m 440 -o root -g root droneweb/droneweb.sudoers /etc/sudoers.d/droneweb
sudo visudo -c

# 7. Autostart
sudo install -m 644 systemd/droneweb.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now droneweb
```

## Prestanda / RAM (Pi 3A+, 512 MB)

Snålt men fungerar. Steg 1 importerar **inte** OpenCV (spar ~80 MB) — det laddas först i
Steg 2. Kolla marginalen: `systemctl status droneweb` (RSS) och `free -h`. Vid minnestryck:
sänk `MAIN_SIZE`/`FPS` i `camera.py`. On-demand-encodern gör att video-CPU:n är noll när
ingen tittar.

## Felsökning

- **Ingen video** → `systemctl status droneweb`; kollidera inte med mediamtx (`dronecam`
  ska vara **disabled**, annars är kameran upptagen). En process i taget på kameran.
- **Ingen telemetri** ("MAVLink…") → kolla att `mavproxy-ntrip` kör med `--out
  udpin:127.0.0.1:14551` (`systemctl cat mavproxy-ntrip | grep 14551`).
- **Avstängning gör inget** → `sudo -l -U axel` ska visa `/sbin/shutdown`; kontrollera
  `/etc/sudoers.d/droneweb` + `visudo -c`.
- **Tungt/laggigt** → sänk upplösning/fps i `camera.py`; MJPEG är tyngre över 4G än H.264
  (Steg 1 är främst för LAN/tailnet).
