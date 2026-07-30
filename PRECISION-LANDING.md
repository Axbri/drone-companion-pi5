# Precisionslandning (Steg 2)

Tvåfas-visuell precisionslandning inbyggd i **droneweb**: den nedåtriktade IMX219-kameran ser
landningsplattan, Pi:n skickar `LANDING_TARGET` till ArduPilot, och bildanalys-grafiken visas i
webbgränssnittet. Bygger på Steg 1 ([`WEB-INTERFACE.md`](WEB-INTERFACE.md)).

> **Status:** mjukvaran är byggd och rök-testad på Pi:n (detektering, faslogik, `LANDING_TARGET`,
> annoterad video, web-panel). **Kvar innan flygtest:** skriv ut plattan, koppla TF-Luna (I2C),
> montera kameran nedåt, sätt `PLND_*`-params, och kör test-stegen nedan.

## Faser (färg → ArUco → RTK)

| Fas | Villkor | Mål-detektor | Skickar |
|---|---|---|---|
| **COLOR** | AGL ≳ 3,5 m *eller* ArUco ej sedd | orange cirkel (HSV-centroid) | `LANDING_TARGET` |
| **ARUCO** | ArUco sedd *och* AGL < 3,5 m | ArUco `DICT_4X4_50` ID 0 | `LANDING_TARGET` (exaktare) |
| **RTK-HÅLL** | AGL < 0,5 m *eller* mål tappat lågt | — | inget → RTK håller x/y + landar |

Utan TF-Luna (AGL = None, t.ex. bänk) körs **detektionsbaserad** fas: ArUco om den syns, annars
färg. Trösklarna `ARUCO_MAX_AGL` / `RTK_AGL` finns i [`droneweb/precland.py`](droneweb/precland.py).

## Plattan

Se [`landing-marker/`](landing-marker/): `marker_40cm.png` (skriv ut i **40 cm** — markören blir
30 cm, vit kant = quiet-zone) centrerad i en **Ø 62 cm matt safety-orange cirkel**. Markör =
`DICT_4X4_50` **ID 0** (måste matcha `ARUCO_ID` i precland.py). Räckvidd @ 640×480: färg ~8 m,
ArUco tillförlitligt ~3 m (rök-testad detektering ned till ~34 px markör).

## TF-Luna (AGL via I2C) — verifierad setup

1. **I2C-läge på sensorn:** TF-Lunas pin5→GND sätter I2C-läge (default är UART). Verifiera
   gärna mot en Arduino först. Wire: sensor SDA→Pi **pin 3 (GPIO2)**, SCL→**pin 5 (GPIO3)**,
   5V, GND. (Sensorns I/O är 3,3 V — OK mot Pi:ns I2C.)
2. **Aktivera I2C på Pi:n (TVÅ delar — bara dtparam räcker INTE):**
   - `dtparam=i2c_arm=on` i `/boot/firmware/config.txt` (instansierar i2c-1-styrenheten).
   - Ladda **`i2c-dev`-modulen** (skapar `/dev/i2c-1`): `echo i2c-dev | sudo tee
     /etc/modules-load.d/i2c.conf` (består vid boot). `raspi-config nonint do_i2c 0` gör båda.
   - **Reboot.** Verifiera: `ls /dev/i2c-1` finns; enheten svarar på **0x10** (bus 1).
   - `axel` måste vara i **`i2c`-gruppen** (udev sätter `/dev/i2c-1` till `root:i2c` 660) —
     annars kan droneweb (körs som axel) inte läsa. `sudo usermod -aG i2c axel` vid behov.
   - Obs: **IMX219-kameran ligger också på 0x10 men på bus i2c-10** (separat) → ingen krock.
3. `smbus2` finns i `droneweb-venv`. [`droneweb/rangefinder.py`](droneweb/rangefinder.py) läser
   dist+amp (reg 0x00, 6 byte) och **reläar `DISTANCE_SENSOR`** till FC. Ogiltig läsning
   (amp<100 för svag, **amp=0xFFFF mättad**, eller dist utanför 0,1–8 m) → `agl()`=None (graciöst;
   precland kör då detektionsbaserad fas). Verifierat: läser 0,70 m mot golv på 72 cm.

**Felsökning (TF-Luna):**
- **dist=0, amp=0xFFFF (mättad) på ett rimligt avstånd** → **optisk blockering framför linsen**
  (skyddsfilm, kåpa, glas) eller mål i dödzonen (<0,2 m). Ta bort filmen/kåpan. (Detta var
  fältfelet — inte kod/kabling.)
- **`i2cdetect`/scan hittar 0x10 men dist=0** → ofta ovanstående; kontrollera även att I2C är
  aktiverat (`ls /dev/i2c-1`) och att `i2c-dev` laddas vid boot.
- **`agl` blinkar null fast direkt läsning funkar** → **buss-krock**: kör inte manuella i2c-läsningar
  samtidigt som droneweb (dess rangefinder-tråd läser 0x10 kontinuerligt). Stoppa droneweb för
  manuella sensortester.

## ArduPilot-params (sätts i Mission Planner)

```
PLND_ENABLED    = 1
PLND_TYPE       = 1     # MAVLink / companion (LANDING_TARGET)
PLND_EST_TYPE   = 0     # 0=raw sensor, 1=kalman (prova båda)
PLND_YAW_ALIGN  = 0     # kamerans monterings-yaw (0 om kamera-x = nos)
PLND_ALT_MIN    = 0.5   # sluta jaga målet under 0,5 m → matchar RTK-håll-fasen
PLND_STRICT     = 1
# TF-Luna som MAVLink-rangefinder (reläas av rangefinder.py):
RNGFND1_TYPE    = 10    # MAVLink
RNGFND1_ORIENT  = 25    # nedåt
RNGFND1_MIN_CM  = 10
RNGFND1_MAX_CM  = 800
```

Läggs i param-backupen (`cube-full-params-*.param`) efter att de satts.

> **Två gotchas (verifierade 2026-07-30):**
> - **`RNGFND1_TYPE=10` kräver en FC-reboot** för att MAVLink-rangefinder-backend ska
>   initieras — annars syns inget `RANGEFINDER` och `DISTANCE_SENSOR` ignoreras.
> - **`PLND_TYPE` måste vara `1`** (MAVLink/companion), inte 0 (=ingen källa).
> - Verifiera från Pi:n: `RANGEFINDER` från FC ska visa TF-Lunas avstånd (t.ex. 0,69 m).
>   droneweb reläar `DISTANCE_SENSOR` **alltid** (från boot, oberoende av precland-arm).

## Användning

1. Öppna webben, flyg (piloten) drönaren över plattan.
2. Tryck **Aktivera** i precland-panelen (bekräfta-dialog). Pi:n skickar `LANDING_TARGET` när
   ett mål syns; videon växlar till **annoterad** vy (cirkel/markör-box, offset, fas, AGL).
3. Piloten går till **LAND** — ArduPilot korrigerar x/y mot målet och sjunker; under ~0,5 m
   släpper Pi:n och RTK håller in touchdown.
4. **Avaktivera** när som helst; pilotens läges-byte/RC vinner alltid. **Pi:n armar/flyger aldrig.**

## Inspelning (för fältanalys i efterhand)

Tryck **● Spela in** i precland-panelen. Fungerar **med eller utan** precland armerat → du kan spela
in hela flygningen. Sparar synkat i `/home/axel/recordings/`:

- **Video** `.avi` (MJPEG, 640×480, **med CV-overlay** — markör/cirkel, fas, AGL, offset).
- **Data** `.csv` — **en rad per videoruta** (ruta N ↔ rad N): `t` (epok-tid, exakt), mode, armed,
  roll, pitch, yaw, heading, alt, **agl** (TF-Luna), batteri (V/%), GPS-fix/sats, fas, mål-källa,
  offset ox/oy, `angle ax/ay` (grader), `sent` (skickades LANDING_TARGET). CSV:ns `t` är den exakta
  tidsstämpeln per ruta; video-fps är nominell (~8 Hz på Pi 3A+).

Ladda ner via **Inspelningar**-panelen (video- + data-länkar) eller `http://dronepi:8080/recordings/<namn>.avi`.
Radera via ✕ i listan. **Analystips:** plotta CSV (agl vs t = nedstigningsprofil, ox/oy vs t =
inriktnings-konvergens, fas-övergångar, `sent`) och titta på videon för visuell kontext.

## Att verifiera / tuna

- **Kamera→kropp-mappning** i `image_to_body()` (precland.py): flytta målet mot nosen →
  `angle_x` ska bli positiv. Rotera mappningen om kameran är monterad annorlunda. **Kritiskt.**
- **HSV-färg** (`ORANGE_H_LO/HI`, `ORANGE_S_MIN`, `ORANGE_V_MIN`): kalibrerat i sol 2026-07-30 —
  plattan avbildas som **mättad röd (H≈0, S≈240, V≈214)**, inte orange, så trösklarna använder
  **hue-wrap** (H ≤ 12 ELLER ≥ 165) + hög S/V → separerar rent mot grönt gräs (H~40-80).
  **Omkalibrera** vid annat ljus med `droneweb/tune_color.py` (samplar plattan via ArUco → HSV).
  Fast exponering/AWB (picamera2-controls) ger stabilast färg om ljuset varierar mycket.
- **Kamera-kalibrering** (schackbräde) för exakta vinklar; nu används FOV-approx.

## RAM (Pi 3A+)

cv2 laddas **lazy** vid första arm → Steg 1-RAM oförändrad. Armerat: droneweb-RSS ~**187 MB**,
~105 MB fritt + swap. Fungerar men snålt. Vid tryck: sänk upplösning/`RATE_HZ`, eller Pi 4/5.

## Test-stege (säkerhet först)

1. **TF-Luna:** `i2cdetect` = 0x10; rimlig AGL; `DISTANCE_SENSOR` syns i MP.
2. **Bänk (inga propellrar):** kamera på stativ över plattan → webben visar overlay + offset följer
   när du flyttar plattan; bekräfta `LANDING_TARGET` i MP (PrecLand-status). Mät RSS.
3. **Kalibrering** + verifiera vinkeltecken.
4. **Hovring ~3 m, armerat, INGEN LAND:** logga att korrektionerna pekar rätt.
5. **Bevakad LAND** från låg höjd → sedan full 8 m → touchdown; jämför mot ren RTK-LAND.

## Uppskjutet

- **Yaw-inriktning** (vrida drönaren mot markören) via RC-yaw-override — egen testiteration.
- **Nästlad liten markör** för spårning ännu lägre än RTK-håll.
