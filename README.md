# drone-ntrip-pi — onboard RTK-companion (Pi 3A+ → NTRIP → MAVLink)

En Raspberry Pi ombord på drönaren som drar RTK-korrektioner (RTCM) över 4G
och injicerar dem till ArduPilot (Cube) över en **serial MAVLink-länk** — så
att laptop + SiK-radio inte längre behövs för RTK i fält.

Konsument av hemmabasen ([`../home-base/`](../home-base/)). Första byggstenen
mot en fullare följeslagardator (MAVLink/video/styrning över 4G senare).

## Kedjan

```
Hemmabas (F9P) → hemma-internet → Tailscale → 4G (mobilrouter) → Pi 3A+ (WiFi)
   → MAVProxy ntrip → serial 921600 → Cube GPS2 → GPS_RTCM_DATA → UM982 → RTK Fix
```

UM982-kompassen sitter på **GPS1 / SERIAL3** (orörd); companion-Pi:n på
**GPS2 / SERIAL4**. (TELEM1 = ELRS, TELEM2 = SiK.)

## Hårdvara (PoC)

* **Raspberry Pi 3 Model A+** — 512 MB, inbyggd WiFi, GPIO-UART. Räcker gott.
* **Mobil 4G-router** med WiFi (batteridriven, monteras på drönaren). Pi:n
  ansluter via WiFi. Redundant men fungerar; framtida uppgradering = USB-4G-
  dongle/HAT direkt på Pi:n.
* **5 V/≥3 A BEC/UBEC** från drönarbatteriet för Pi:ns ström (**inte** Cube:ns
  telem-5V).
* 3 jumperkablar (TX/RX/GND) mellan Cube GPS2 och Pi GPIO.

## Varför den här vägen

* **Ingen SiK-flaskhals** — serial @921600 klarar full MSM7 utan bantning.
* **Ingen laptop i fält** — Pi:n är självständig, startar korrektionerna vid boot.
* **Tailscale** → når din egen bas oberoende av RTK2Go:s upptid, anonymt.

## Repo-layout

```
drone-ntrip-pi/
├── README.md                     — denna fil
├── SETUP.md                      — bygg-/konfig-guide (flash, wiring, UART, MAVProxy, Cube-params)
├── UM982-GPS-CONFIG.md           — GNSS-kompass (UM982) + ArduPilot-params, heading, lärdomar
├── mediamtx.yml                  — kamera-/RTSP-konfig (läggs på Pi:n som ~/mediamtx.yml)
└── systemd/
    ├── mavproxy-ntrip.service    — autostart av MAVProxy + ntrip vid boot
    └── dronecam.service          — autostart av mediamtx (kamera-RTSP)
```

GNSS-kompassen (UM982) och dess ArduPilot-konfig dokumenteras separat i
[`UM982-GPS-CONFIG.md`](UM982-GPS-CONFIG.md) — **läs den innan du rör GPS-/heading-inställningar.**

## Snabbstart

Full guide i [`SETUP.md`](SETUP.md). Kort:

1. Flasha Pi 3A+ (Pi OS Lite, WiFi = mobilrouter, SSH på).
2. Tailscale på Pi:n → samma tailnet som basen.
3. Wire Cube GPS2 ↔ Pi GPIO (TX/RX/GND), driv Pi:n från BEC.
4. Frigör Pi-UART:en (`disable-bt`, stäng serie-konsol).
5. Installera MAVProxy, testa `ntrip`-modulen mot `homebase/LOCAL`.
6. Cube: `SERIAL4_PROTOCOL=2`, `SERIAL4_BAUD=921` (GPS2-porten).
7. När allt funkar: aktivera `systemd/mavproxy-ntrip.service`.

## Telemetri över 4G (Mission Planner via Tailscale)

Samma MAVProxy-instans som injicerar RTK serverar även **MAVLink-telemetri över
nätverket** (`--out udpin:...14550` + `--out tcpin:...5760`, `--streamrate 10`). Så du
kan ansluta Mission Planner över internet istället för en SiK-radio på COM-port:

* **Rekommenderat (responsivast):** MP-anslutningstyp **`UDPCl`** → host `dronepi`
  (eller `100.110.147.30`), port **`14550`**. UDP undviker TCP:ns head-of-line-blocking
  → snabbare HUD och param-nedladdning över 4G.
* **Fallback:** anslutningstyp **TCP** → host `dronepi`, port **`5760`**.
* Laptopen måste vara på samma tailnet (`axel.brinkeby@`).
* Ligger **ovanpå** SiK:n (egen länk på TELEM2) — du kan ha båda samtidigt (redundans).
* Behåll `FS_GCS_ENABLE=0` så en 4G-glitch inte triggar GCS-failsafe. Använd inte
  4G-länken för realtids-joystick (lagg); RC går via ELRS.
* Param-nedladdning är seg *första* gången per fordon; MP cachar sedan params så
  följande anslutningar går snabbt.

## Kamera-video → Mission Planner HUD

En CSI-kamera (OV5647) streamas som HUD-bakgrund via **mediamtx** (RTSP), helt
fristående från MAVProxy. Filer: [`mediamtx.yml`](mediamtx.yml) +
[`systemd/dronecam.service`](systemd/dronecam.service).

* **På Pi:n:** mediamtx serverar `rtsp://dronepi:8554/cam` — **1280×960 (4:3)**,
  **10 fps**, full-sensor-läge (full FOV), hårdvaru-H.264, **fast/manuell exponering**
  (`rpiCameraShutter`+`rpiCameraGain` → ingen AGC-oscillation), IDR var ~1 s, **on-demand**
  (kameran/encodern kör bara när MP är ansluten → noll CPU annars). Config: [`mediamtx.yml`](mediamtx.yml).
  Belastning på Pi 3A+: ~0,5 kärna av 4 (video HW-kodad, mavproxy är den tyngsta processen);
  ~0,8 GB/h 4G-data per ansluten MP (nästan allt video).
* **I MP:** installera GStreamer-runtime på Windows (MP uppmanar till rätt version
  första gången). HUD → högerklick → **Video → Set GStreamer Source** → klistra in:
  ```
  rtspsrc location=rtsp://dronepi:8554/cam latency=100 protocols=tcp ! application/x-rtp ! decodebin3 ! queue max-size-buffers=1 leaky=2 ! videoconvert ! video/x-raw,format=BGRA ! appsink name=outsink sync=false
  ```
  (`protocols=tcp` = robust över 4G; ta bort för lägre latens över UDP.)
* ~1–1.5 Mbps → gott om marginal bredvid telemetrin.

**Felsökning (kamera):**
- **"Kan inte ansluta" fast `dronecam` är active + `:8554` lyssnar** → sitter på MP-sidan:
  stäng videon helt i MP och sätt GStreamer-källan på nytt (en drönar-reboot uppdaterar
  inte MP:s videostate), och kolla att laptopens Tailscale är ansluten. Verifiera Pi-sidan
  med `systemctl is-active dronecam`, `ss -tlnp | grep 8554`, och från en tailnet-dator
  `Test-NetConnection dronepi -Port 8554`.
- **Bilden "utsmetad/pixlig" i ~10 s efter störning** → för lång IDR-period. `rpiCameraIDRPeriod`
  ska vara ≈ fps (10 vid 10 fps ≈ 1 s); default 60 = 6 s vid 10 fps → långsam återhämtning.
- **Bilden blinkar helvitt ~1 gång/s (särskilt utomhus)** → auto-exponeringen (AGC) oscillerar i
  starkt ljus, förvärrat av att pinnad fps klampar max slutartid. Fix: **fast exponering** i
  `mediamtx.yml` — `rpiCameraShutter` (µs, **HÖGRE = ljusare**; ~2000–5000 för dagsljus) +
  `rpiCameraGain: 1.0`. Ingen AGC-loop → inget blink. Sänk shutter mot 500–1000 om det klipper
  vitt, höj mot 4000–8000 i skugga/moln.
- **Bilden ihoptryckt/fel proportioner** → MP **sträcker** videon till HUD-panelens form
  (letterboxar inte). Kameran matar redan 4:3; matcha HUD-panelens form till 4:3 (dra i
  avdelaren mot kartan), eller sätt kamerans `rpiCameraWidth/Height` till HUD-panelens
  exakta proportion. (OV5647:s 16:9-lägen är dessutom centrumbeskurna → använd 4:3 för full FOV.)

## Relaterat

* [`../home-base/`](../home-base/) — basen som matar korrektionerna
* [MAVProxy NTRIP-modul](https://ardupilot.org/mavproxy/docs/modules/ntrip.html)
* [ArduPilot: Raspberry Pi via MAVLink](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)
