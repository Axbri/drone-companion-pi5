# Raspberry Pi 5 migration — install guide

Move the drone companion (droneweb + 4G + precland) from the old **Pi 3 Model A+**
to a **Raspberry Pi 5**. Reason: in flight the Pi 3A+ is CPU-starved — the CV loop
drops to 4–5 Hz and precland latency balloons to ~350 ms (bench showed 64 ms/24 Hz,
but flight adds 4G telemetry + web video + RTCM + writer contention). The Pi 5
(~5× CPU, 8–16× RAM) should hold ~15–25 ms even under full flight load, which is the
real fix for the precland tilt-coupling oscillation.

**Repo (source of truth) on the dev machine:** `D:\Elektroninkprojekt\RTK bas-station\drone-ntrip-pi`
Deploy files from there (pscp) or `git clone` the repo onto the Pi.

**SSH from Windows** uses PuTTY `plink`/`pscp`. First contact: harvest the host key with
`plink -batch -pw <pwd> axel@<ip> hostname` (prints the ED25519 fingerprint), then use
`-hostkey "SHA256:..."` on every call (the y/n prompt hangs under redirected stdin).
Prefix remote python one-liners with `MSYS2_ARG_CONV_EXCL='*' MSYS_NO_PATHCONV=1`.

---

## Part A — Axel (physical, before software)

1. **Flash** Raspberry Pi OS **Lite 64-bit** with Raspberry Pi Imager. In the settings (gear):
   - hostname `dronepi5` (avoids Tailscale clash with the old `dronepi`; rename later)
   - user `axel` + password; **enable SSH**; WiFi = `<home-wifi>` (home, for setup + apt)
   - locale/timezone
2. Boot, confirm it joins WiFi, note its IP (or try `dronepi5.local`).
3. **Hardware move (Pi 5 differences!):**
   - **Camera:** IMX219 needs a **Pi 5 camera cable (22-pin → 15-pin)** — old one won't fit.
     (2026-09-14: precland camera is now an **IMX296** Global Shutter mono on the same port; imx477 = FPV.)
   - **Power:** Pi 5 draws up to ~5 V/5 A under load — **verify the BEC can supply it** (else brownout in flight).
   - **TF-Luna:** same I2C — SDA→pin3 (GPIO2), SCL→pin5 (GPIO3), 5V, GND. Sensor pin5→GND = I2C mode.
   - **4G dongle (SIM7600E-H):** any USB port.
   - **Serial to Cube:** GPIO14 (TX, pin8) → Cube RX, GPIO15 (RX, pin10) ← Cube TX, GND. Cube side = the
     GPS2/SERIAL4 or whichever port the FC expects (`SERIALx_PROTOCOL=2 @921600`). **Watch TX/RX crossing.**
4. Keep the **old Pi 3A+ untouched** as fallback until the Pi 5 is verified in flight.

---

## Part B — Software setup (remote, from repo)

Run as `axel`. `sudo` is passwordless once configured; use `echo <pwd> | sudo -S` if it prompts.

### 1. System packages + groups
```bash
sudo apt-get update
sudo apt-get install -y python3-picamera2 python3-opencv python3-libcamera python3-numpy \
    libqmi-utils modemmanager git i2c-tools python3-venv
sudo usermod -aG dialout,i2c,gpio,video,render,spi axel   # re-login for group changes
```
`python3-opencv` must include `cv2.aruco` (Debian's does). ModemManager is installed only to satisfy deps
of libqmi, then **masked** (step 5).

### 2. config.txt / boot  (`/boot/firmware/config.txt`)
```
dtparam=i2c_arm=on
camera_auto_detect=1
enable_uart=1
```
- **UART on Pi 5 differs from Pi 3** — do NOT use `dtoverlay=disable-bt` / `dwc2` / `nospi10` (those were
  Pi-3-specific). On Pi 5, `enable_uart=1` gives the GPIO UART; **verify** `/dev/serial0` maps to it
  (`ls -l /dev/serial0`) and that it is `ttyAMA0` (or the RP1 UART). If `/dev/serial0` is missing/wrong,
  the Pi 5 primary PL011 may be `ttyAMA0`; point mavproxy at whatever is the GPIO UART.
- **Disable serial console** so the FC link isn't stolen: `sudo raspi-config nonint do_serial_hw 0` +
  `do_serial_cons 1`, or remove `console=serial0,...` from `/boot/firmware/cmdline.txt`. Verify no
  `console=serial` remains.
- **I2C module:** `echo i2c-dev | sudo tee /etc/modules-load.d/i2c.conf`
- Reboot. Verify: `ls /dev/i2c-1`, `ls /dev/serial0`, `rpicam-hello --list-cameras` shows imx296 (precland) and imx477 (FPV).

### 3. droneweb app + venv
```bash
# copy the repo's droneweb/ to /home/axel/droneweb  (pscp or git clone)
python3 -m venv --system-site-packages ~/droneweb-venv
~/droneweb-venv/bin/pip install flask pymavlink psutil smbus2
mkdir -p ~/recordings
sudo install -m440 -o root -g root ~/droneweb/droneweb.sudoers /etc/sudoers.d/droneweb
sudo visudo -c
sudo cp systemd/droneweb.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now droneweb.service
```
Web UI at `http://dronepi5.local:8080` (LAN/tailnet, no auth).

### 4. mavproxy-ntrip (RTCM from home base → Cube)
```bash
python3 -m venv ~/mavproxy-venv
~/mavproxy-venv/bin/pip install mavproxy future     # 'future' needed on py3.13
sudo cp systemd/mavproxy-ntrip.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now mavproxy-ntrip.service
```
Serves MAVLink locally: `--out udpin:127.0.0.1:14551` (droneweb), `--out tcpin:0.0.0.0:5760` (MP/tools).
NTRIP: caster `homebase:2101`, mount `LOCAL`, anon/anon (over Tailscale). Master = `/dev/serial0 @921600`
(adjust to the Pi 5 UART device from step 2).

### 5. 4G (SIM7600E-H, direct QMI — NOT ModemManager)
```bash
sudo systemctl mask --now ModemManager           # its AT/PPP path is flaky; we use QMI
sudo install -m755 4g/4g-modem.sh /usr/local/sbin/4g-modem.sh
sudo install -m600 4g/qmi-network.conf /etc/qmi-network.conf
sudo install -m644 4g/99-unmanage-wwan0.conf /etc/NetworkManager/conf.d/99-unmanage-wwan0.conf
sudo cp systemd/4g-modem.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now 4g-modem.service
```
APN `internet.telenor.se`. Route metric 700 (wlan0=600 wins at home; 4G used when no WiFi).
Verify: `mmcli`-free; `ping -I wwan0 8.8.8.8`; `ip route` shows wwan0 default metric 700.
**Remove the portable-router WiFi profile** so the drone can't auto-join it in the field:
`sudo nmcli connection delete mobile` (SSID "<phone-hotspot>") — keep only `<home-wifi>`.

### 6. Tailscale
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up        # prints an auth URL — Axel must open it and approve the node
```
Confirm the drone reaches `homebase:2101` over the tailnet.

---

## Part C — Verify (before flight)
- `i2cdetect -y 1` → TF-Luna at **0x10**; droneweb "Lidar OK", AGL sane.
- Camera: web UI shows video; precland "● Spela in" runs the CV loop.
- MAVLink: mavproxy heartbeat from Cube; FC `RANGEFINDER` shows TF-Luna distance (needs `RNGFND1_TYPE=10`
  + FC reboot); `PLND_TYPE=1`.
- 4G: force it via the web-UI "Tvinga 4G" button, reach the drone over Tailscale.
- RTK card: base connected + RTCM flowing + GPS RTK.
- **Latency readout** (precland card "Latens (PI)"): should be **~15–25 ms** at high loop-Hz even with
  4G video + telemetry running — the whole point of the upgrade. If still high, check camera FrameRate
  (`FPS` in `camera.py`, now 40) ≈ the achieved loop rate, `buffer_count=1` (2 caused backlog on Pi 5
  at 40Hz — see camera.py comments). Measured on the bench 2026-08-18: ~19-21ms @ ~40Hz, stable.

## Systemklocka — ingen RTC-batteri (2026-08-25)

Pi 5:ns inbyggda RTC (`/dev/rtc0`) har **ingen batteribackup installerad** på denna enhet — vid
varje kallstart nollställs den till Unix-epok (`setting system clock to 1970-01-01T00:00:15`).
Klockan förlitar sig alltså helt på NTP efter varje boot. Uppmätt en gång: NTP tog **20+ timmar**
att lyckas synka (Pi:n var på hemma-WiFi hela tiden, orsaken ospårad — trolig långsam
retry-backoff/opålitlig pool-server, inte fel nätverk). Under tiden är alla tidsstämplar
(loggar, inspelningsfilnamn) opålitliga.

**Fix:** `mavlink.py` sätter systemklockan från FC:ns GPS-tid (`SYSTEM_TIME.time_unix_usec`,
ArduPilot fyller i den från GPS vid fix) som fallback — kräver 3D-fix + ett rimlighetstest, och
rör ALDRIG klockan om NTP redan har synkat (`/run/systemd/timesync/synchronized` finns). GPS-tid
är alltid tillgänglig i fält, oberoende av server/nätverk. Kräver `sudo date -s @*` i
`droneweb.sudoers` (installerad). Permanent fix vore en RTC-batteri (liten knappcell på Pi 5:ans
RTC-kontakt) — inte gjort än, GPS-fallbacken täcker behovet under flygning.

## FC params (Cube — for reference; verify live, this file is a snapshot)
`PLND_ENABLED=1, PLND_TYPE=1, PLND_EST_TYPE=1` (Kalman, switched from raw 2026-08-18),
`PLND_LAG=0.025` (25ms — lowered from 0.25 to match the Pi 5's measured ~20ms pipeline latency
at 40Hz; was set high for the old Pi 3A+), `RNGFND1_TYPE=10/ORIENT=25`,
GPS1_TYPE=25 (UM982), `SERIALx` for the Pi link @921600. Backup: `cube-full-params-*.param` (stale —
predates the Pi 5 migration params above; re-export before trusting it).
Precland tuning: **resolved 2026-08-20** — root cause was a double camera→body rotation (our own
transform duplicating one ArduPilot already does internally for BODY_FRD `LANDING_TARGET`), not
latency. Fixed in `precland.py` (see `PRECISION-LANDING.md` Flygtest #3/#4). Verified over many
LAND and RTL landings at full styrskala=1.0 — fast, precise correction, no overshoot.

## Finish
**Done (2026-08-20):** flight-verified (many LAND/RTL landings, see precland section above) — the
**old Pi 3A+ is retired**, powered off, fully replaced by the Pi 5. Hostname/Tailscale rename
(`dronepi5` → `dronepi`) not done yet — still running as `dronepi5`.
Deploy path on the Pi: `/home/axel/droneweb/`. Restart after edits: `sudo systemctl restart droneweb`.
