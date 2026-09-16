# roverweb — webbinterface för ArduRover companion computer (V2)

Webbaserat kontroll- och övervakningsgränssnitt som kör på companion-datorn
(**roverpi**, Raspberry Pi 5) och nås över Tailscale. Byggt efter konceptskissen
[`../web interface concept scetch.png`](../web%20interface%20concept%20scetch.png).

**Åtkomst:** <http://roverpi:8080>

Kompanjon till [`../Ardurover companion computer plan.md`](../Ardurover%20companion%20computer%20plan.md)
(V1: RTK, video, MAVLink-routing) och [`../home-base/`](../home-base/) (RTK-basstationen).

## Arkitektur

```
Pixhawk 4 ──TELEM1 921600── /dev/serial0 ── mavproxy.service
                                              │  fan-out (udpin)
                                              ├─ :14550  Mission Planner (UDPCl)
                                              ├─ :14551  rover-status (CLI)
                                              └─ :14552  roverweb  ◄── denna app
                                                            │
  webbläsare ──WebSocket /ws──► roverweb (FastAPI+uvicorn, :8080)
             ──WebRTC/WHEP────► mediamtx (:8889/rover/whep)
             ──kartrutor──────► OSM / Esri / OpenSeaMap  (direkt från internet)
```

* **Telemetri** läses från mavproxys dedikerade endpoint `udp:127.0.0.1:14552`
  och pushas till webbläsaren som JSON över WebSocket (10 Hz).
* **Styrning** (arm, mode, körning) skickas tillbaka över samma WebSocket.
* **Video** går *inte* genom roverweb — webbläsaren hämtar den direkt från
  mediamtx via WebRTC (låg latens).
* **Kartrutor** hämtas av webbläsaren från internet → **kostar ingen 4G-data på rovern**.

## Funktioner

| Panel | Innehåll |
|---|---|
| **Fordonstelemetri** | mode, armering, batteri (V/A/%), fart, heading, GPS-fix, satelliter, HDOP, koordinater |
| **Styrning** | armera/disarmera, mode-val, **AUTO-uppdragsstyrning**, D-pad + tangentbord, effekt-slider, systemmeddelanden |
| **Raspberry Pi** | CPU-temp/last/load, RAM, nät ↑/↓, **Data/h-estimat** (4G-förbrukning), **avstängningsknapp** |
| **Home base** | om RTK-basens caster (`homebase:2101`) svarar |
| **Kamera** | WebRTC-ström + upplösnings- och FPS-meny |
| **Karta** | rover, spår, mission, fence, rally, avståndsringar, flera kartlager |

### Körning (manuell)

* Aktiv **endast** i `MANUAL`/`STEERING` **och** armerad (annars gråas D-paden).
* **Tangentbord:** piltangenter eller WASD. Flera riktningar samtidigt — håll
  framåt och dutta höger/vänster för kurskorrigering.
* **Effekt-slider** skalar både gas och styrning (10–100 %).
* **Dödmansgrepp:** klienten skickar 10 Hz medan knapp/tangent hålls; servern
  har en 20 Hz watchdog som neutraliserar om kommando saknas > 0,6 s, eller om
  fönstret tappar fokus / körläget lämnas.

### AUTO (uppdrag)

I Styrning-panelen finns en AUTO-ruta som visar **vilken waypoint rovern kör mot**
(från `MISSION_CURRENT`, samma nummer som den blå ringen på kartan) och låter dig
styra uppdraget:

| Kontroll | Funktion |
|---|---|
| **Kör mot waypoint** | Live-nummer på aktuell målwaypoint |
| **WP-nr + "Sätt WP"** | Hoppar uppdraget till angivet nummer (Enter fungerar också) |
| **"↻ Börja om"** | Snabbknapp: sätter aktuell waypoint till **1** = starta om uppdraget |

Skickas som `MISSION_SET_CURRENT` (samma sak som Mission Planners "Set WP").
Fungerar oavsett läge men får effekt när farkosten kör i **AUTO** — bekräftelsen
syns direkt genom att numret och den blå ringen flyttar sig.

> ⚠️ Är rovern armerad i AUTO börjar den köra mot den nya waypointen **direkt**.

### Karta

| Lager | Not |
|---|---|
| Underlag | Karta (OSM) · Satellit (Esri) · Topo (OpenTopoMap) · Stigar (CyclOSM) |
| Sjömärken | OpenSeaMap-överlägg, oberoende av underlag |
| Relief | Esri Hillshade, halvtransparent |
| Mission | Home (grön), waypoints (blå, numrerade), ruttlinjer, **DO_JUMP**-slinga |
| Fence | **lila** streckad polygon (tillåtet) · **röd diagonalstreckad** yta (förbjudet) |
| Rally | orange punkter |
| Ringar | 50/100/200 m från Home |
| Mål-WP | blå ring runt aktuell waypoint (`MISSION_CURRENT`) |

"Läs mission" hämtar **mission → fence → rally** i en kedja (tre MAVLink
mission-typer). "Rensa karta" tömmer allt ritat (påverkar inte farkosten).

### Nätverk (WiFi) — lägg till/välj nät från webben (2026-07-31)

**Varför:** Pi:n nådde inte en mobil 4G-router på annan plats — NetworkManager
autoansluter bara till **sparade** nät, och bara hemnätet var sparat. Moment 22:
gick inte att nå Pi:n för att lägga till nätet. Panelen löser det: lägg till 4G-
nätet **hemma** (på tailnet) innan resan, så autoansluter Pi:n på plats. Pi:ns
tailnet-IP är stabilt oavsett underliggande nät, så förbindelsen består.

Panelen "Nätverk (WiFi)" i vänsterkolumnen visar:
- **Nuvarande nät** (SSID + signal, live via telemetrin `state["net"]["ssid"]`) + IP.
- **Sparade nät** med *Anslut* och *Radera* (✕).
- **+ Lägg till nät** — välj ur scan eller skriv SSID + lösenord.
- **↻ Uppdatera/scanna**.

Backend = NetworkManager via `nmcli`. Skrivning kräver `sudo` (polkit nekar
tjänsten annars, trots `netdev`-grupp) → samma sudo-mönster som avstängning.
Endpoints (FastAPI, före StaticFiles-mounten):

| Endpoint | Gör |
|---|---|
| `GET /wifi/list` | nuvarande + sparade + scannade nät |
| `POST /wifi/add` `{ssid,password}` | skapar sparad profil (autoconnect, priority 0) — **kopplar INTE om** |
| `POST /wifi/forget` `{name}` | raderar sparat nät — **vägrar det aktiva** |
| `POST /wifi/connect` `{name}` | byter nät med **auto-återgång** (verifierar internet ≤25 s, annars tillbaka) |

**Säkerhet (allt curl-testat):** "lägg till" kan inte fälla förbindelsen (skapar
bara profil, byter inte); "radera" skyddar det aktiva nätet; "byt" kan inte låsa
ut dig (auto-återgång i egen tråd). Nätverksfakta: stack = NetworkManager +
wpa_supplicant-backend, hemnätet heter `netplan-wlan0-<home-wifi>` (netplan-
genererat), `iw reg`=SE. `axel` har NOPASSWD:ALL sudo.

## Filer

```
roverweb/
├── server.py            — FastAPI-backend (telemetri, styrning, mission, video-config, WiFi/nätverk)
├── static/
│   ├── index.html       — layout
│   ├── app.js           — WebSocket, styrning, WebRTC, Leaflet-karta
│   └── style.css        — mörkt tema
└── deploy/
    ├── roverweb.service — systemd-unit för webbappen
    ├── mavproxy.service — MAVLink-routing + NTRIP RTK (V1)
    ├── mediamtx.service — videoserver
    ├── mediamtx.yml     — kamerakonfig (skrivs om av roverweb vid kvalitetsbyte)
    └── rover-status     — CLI-hälsokoll (se plan-dokumentet)
```

## Installation / driftsättning

Beroenden i companion-datorns venv:

```bash
~/mav-venv/bin/pip install fastapi "uvicorn[standard]"
```

Lägg koden i `/home/axel/roverweb/`, installera unit-filerna och starta:

```bash
sudo install -m0644 deploy/roverweb.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now roverweb
systemctl is-active roverweb          # -> active
curl -s -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8080/   # -> 200
```

> **mediamtx-configen ligger i `/home/axel/roverweb/mediamtx.yml`** (inte
> `/usr/local/etc/`) så att roverweb kan skriva om den utan sudo. `mediamtx.service`
> pekar dit. mediamtx **hot-reloadar** vid filändring — därför byter videokvaliteten
> utan omstart (~3 s avbrott).

## Portar

| Port | Tjänst |
|---|---|
| 8080 | roverweb (HTTP + WebSocket) |
| 8554 / 8889 / 9997 | mediamtx: RTSP / WebRTC / REST-API |
| 14550 / 14551 / 14552 | mavproxy fan-out: Mission Planner / rover-status / roverweb |

Allt exponeras bara på tailnet — inga portar öppnade mot internet.

## Fällor vi gick på (läs innan du felsöker)

1. **`SYSID_MYGCS` styr RC-override.** ArduPilot accepterar
   `RC_CHANNELS_OVERRIDE` bara från GCS-sysid som matchar `SYSID_MYGCS` (default
   **255**). Med sysid 250 fungerade arm och mode-byte men **körningen gjorde
   ingenting**. Servern använder därför `source_system=255`.

2. **Släpp aldrig overriden helt.** Att skicka nollor (release) ser ut som
   RC-bortfall för ArduPilot när ingen mottagare är inkopplad → failsafe → läget
   lämnar MANUAL och knapparna gråas. Watchdogen skickar **neutral (1500)** i
   stället; rovern stannar ändå.

3. **Varje tittare = en egen kopia av videoströmmen.** Kvarhängande
   WebRTC-sessioner staplades till 12 st (~36 Mbit/s). Klienten stänger nu gamla
   sessioner och stänger vid `pagehide`. **Håll bara en flik öppen.**

4. **Bitrate-taket är en riktlinje.** Pi 5 saknar hårdvaru-encoder; OpenH264
   håller inte taket vid rörelse/detaljer (uppmätt 17,9 Mbit/s mot 2,9 i mål).
   **Upplösning och FPS är de pålitliga spakarna** — uppmätt encoder-output:
   640×480@5 → 0,1 · 640×480@15 → 1,1 · 1280×960@30 → 3,0 Mbit/s.

5. **Fence-kommandon:** `5000` return point, `5001/5002` polygon-hörn
   (inkl/exkl, `param1` = antal hörn), `5003/5004` cirkel (`param1` = radie),
   `5100` rally. (Lätt att förskjuta med ett steg.)

6. **AF-inställningar är Cam 3-specifika.** `rpiCameraAfMode`/`LensPosition`
   får libcamera att vägra starta en fast-fokus-sensor (Cam v2/IMX219) → svart bild.

## Drift

```bash
systemctl is-active roverweb mavproxy mediamtx
journalctl -u roverweb -n 40 --no-pager
rover-status                 # snabb hälsokoll (tjänster, RTK, video, batteri)
curl -s http://127.0.0.1:9997/v3/paths/list   # mediamtx: antal läsare, faktisk bitrate
```

Efter ändring i `static/` räcker **hård-uppdatering** i webbläsaren (Ctrl+Shift+R);
ändringar i `server.py` kräver `sudo systemctl restart roverweb`.

### CV-visualisering — två detektorer i samma vy (V3.7 + V4.3)

Vyn visar **antingen** V3 (roverav, färgbaserad) **eller** V4 (roverstereo), och
säger vilken i en egen etikett: `V3 färg` (gul) eller `V4 stereo` (grön).

De kan inte köra samtidigt — båda vill ha cam0 — så servern läser båda filerna
i `/dev/shm` och **väljer den som är färskast**, inte den som har företräde.
Fälten byts med källan i stället för att fyllas med platshållare: `Underlag`
finns bara i V3, `Markplan` och `Sikt` bara i stereo. Att blanda ihop dem vore
värre än att inte visa dem — talen betyder olika saker.

**Stereons egen bild ersätter videon.** När stereotjänsten kör är `mediamtx`
stoppad, alltså finns ingen RTSP-ström och videopanelen är svart. Tjänsten
skriver därför en egen bild till `/dev/shm/roverstereo_live.jpg` i 2 Hz som
servern serverar på **`/stereo.jpg`**, och klienten lägger den ovanpå
videoelementet så länge stereo är källan. Bilden ritas ur **exakt de arrayer
hinderbeslutet bygger på** (`roverstereo/stereo_draw.py`, delad med
felsökningsverktyget) — inte ur en egen kamerakedja som skulle kunna visa något
annat än det tjänsten räknar på.

Den går som vanlig HTTP, inte över websocketen: ~30 kB per bild i 2 Hz hade
dränkt telemetrikanalen, och webbläsaren hämtar om den själv. Klienten startar
nästa hämtning först när den förra laddat klart — annars köar de på sig på en
långsam länk och bilderna blir gamla utan att någon märker det.

Stereopanelen visar också:

| Fält | Varför det står där |
|---|---|
| `Markplan` 20,5° · 27 cm | hela den geometriska kedjans hälsa i två tal. Stämmer de inte med riggen (20°, 27 cm) är höjdskalan fel — och med den hinderhöjder och markkontakt. Gulnar när marklinjen inte är färsk. |
| `Sikt` 2,8 m | så långt bort marken syns vid analysfönstrets överkant. Hinder längre bort har foten utanför bilden och känns inte igen. Det är den verkliga räckvidden, inte `PRX1_MAX`. |
| radarns röda innerzon | **närgränsen** (~0,33 m). Närmare än så försvinner hinder i stället för att larma. En blind fläck ska ritas ut, inte utelämnas. |
| diagnostikraden | giltiga pixlar, närblind, förkastade klumpar, dt, gir, hinderhöjdsgränser, sändtakt |

⚠️ **Kryssrutan heter numera `Hinderdata` och styr BÅDA detektorerna.** Är den
urkryssad syns ingenting — det är avsikten (bandbredd), inte ett fel.

🚨 **`/dev/shm` töms när sista SSH-sessionen stängs.** `RemoveIPC=yes` är
default i `logind.conf`, tjänsterna kör som `User=axel`, och när användarens
sista session tagit slut (plus `UserStopDelaySec`, ~10 s) raderar systemd-logind
**alla axel-ägda filer i `/dev/shm` — även för en tjänst som kör**. Verifierat
2026-07-24 med en markörfil: kvar efter 8 s, borta efter 40 s.
Det är ofarligt här, och det är designen som gör det ofarligt: båda
detektorerna skriver om filen 5 ggr/s med skriv-och-byt, och läsaren öppnar via
sökväg varje gång. Effekten blir ett glapp på ~0,2 s. **Lägg däremot aldrig
något i `/dev/shm` som bara skrivs en gång** — det försvinner tyst.

Två vyer som visar vad hinderdetekteringen gör, båda under kamerabilden:

* **Överlägg på videon** — fri yta skuggas grönt, hinderkonturen ritas gul (utanför
  `AVOID_MARGIN`) eller röd (innanför), plus streckade referenslinjer på 0,8 / 1,5 / 3,0 m.
* **Radarvy** — de 72 sektorerna som faktiskt skickas som `OBSTACLE_DISTANCE`, med
  avståndsringar och marginalen som röd streckad ring.

**Radarn visar vad ArduPilot får, inte vad detektorn tyckte.** Skiljer de sig är det just
skillnaden man vill kunna se — bekräftelsefiltret och höjdfiltret sitter mellan dem.

Designval värda att känna till:

1. **Överlägget ritas i webbläsaren, inte i videoströmmen.** mediamtx slipper koda om, och
   överlägget kan slås av utan att röra den fältverifierade videokedjan.
2. **Koordinaterna är normaliserade (0..1)**, inte pixlar. Servern behöver inte veta något
   om videoupplösning och överlägget sitter rätt även om kvaliteten byts under drift.
3. **`object-fit: contain` brevlådar videon.** Ritar man rakt på elementets yta hamnar
   överlägget fel så fort bildens proportioner skiljer sig från panelens — `videoRect()`
   räknar därför ut den faktiskt ritade rutan.
4. **Data går via `/dev/shm/roverav_live.json`** respektive
   **`/dev/shm/roverstereo_live.json`** (tmpfs, **inte** SD-kortet — filerna skrivs
   5 ggr/s och skulle annars nöta kortet). Detektorn skriver atomiskt, roverweb läser
   passivt; en död detektor syns som gammal data i stället för att blockera
   webbgränssnittet. Data äldre än 3 s kastas — en detektor som slutat skriva ska
   inte lämna en frusen hinderbild kvar på skärmen.
5. **Skickas bara när rutan är ikryssad**, i 5 Hz i stället för telemetrins 10. Alltid-på
   hade kostat ~70 MB/h extra över 4G. Verifierat: 0 meddelanden av, exakt 20 på 4 s när på.

### Avstängningsknappen (Pi-panelen)

Finns för att ett hårt strömavbrott på en körande Pi är den SD-korruption som står som
felmod i plandokumentet — och sedan V3 skriver `roverav` dessutom kontinuerligt till
journalen. Kör `sudo shutdown -h now` (fungerar utan lösenord, `axel` har NOPASSWD-sudo).

Tre spärrar, för att en fjärrknapp som släcker companion-datorn ska vara svår att träffa
av misstag:

1. **Vägras när fordonet är ARMERAT** — kontrolleras i `server.py`, inte bara i webbläsaren.
   Går Pi:n ned tar den med sig mavproxy och därmed video, telemetri *och*
   RC-override-vägen som den manuella körningen använder. Att kunna göra det mitt under
   körning vore att bygga in en fjärrstyrd förlust av kontrollen.
2. **WebSocket-kommandot kräver en uttrycklig nyckel** (`confirm: "SHUTDOWN"`), så ett
   tappat eller upprepat meddelande inte kan släcka Pi:n.
3. **Dialogruta i UI:t**, och knappen är dämpad tills muspekaren är på den — den ska inte
   konkurrera visuellt med DISARMERA, som är knappen man vill träffa när det bränner.

**Vänta tills lysdioden slocknat innan strömmen bryts.**

## Nästa steg (V3)

Obstacle avoidance — kamera → `OBSTACLE_DISTANCE`/`DISTANCE_SENSOR` → ArduPilots
inbyggda undvikande. Se V3-avsnittet i plan-dokumentet.
