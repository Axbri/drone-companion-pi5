# Precisionslandning (Steg 2)

Tvåfas-visuell precisionslandning inbyggd i **droneweb**: den nedåtriktade IMX219-kameran ser
landningsplattan, Pi:n skickar `LANDING_TARGET` till ArduPilot, och bildanalys-grafiken visas i
webbgränssnittet. Bygger på Steg 1 ([`WEB-INTERFACE.md`](WEB-INTERFACE.md)).

> **Status (2026-08-20):** flygverifierad, fungerar bra. Snabb och exakt korrektion så fort
> drönaren kommer ner på rätt höjd och ser plattan, ingen översläng, styrskala=1.0. Verifierat
> med många landningar i både LAND- och RTL-läge. Se Flygtest #3 nedan för grundorsak/fix och
> Flygtest #4 för verifieringsflygningen.
>
> **2026-08-21:** färgläge borttaget, upplösning höjd till 1024×768, spårningen kör nu alltid
> (ingen Arm-knapp). Se Steg 3-avsnittet nedan — **INTE flygtestad än vid denna upplösning/
> alltid-på-form.**

## Faser (WAIT → ArUco → RTK)

Färgläget (orange cirkel, för mål på hög höjd) togs bort 2026-08-21 — se Steg 3-avsnittet
nedan för varför.

| Fas | Villkor | Mål-detektor | Skickar |
|---|---|---|---|
| **WAIT** | AGL > `aruco_start_agl` | ArUco (diagnostik, se nedan) | inget |
| **ARUCO** | `rtk_agl` ≤ AGL ≤ `aruco_start_agl` (eller AGL okänd) | ArUco `DICT_4X4_50` ID 0 | `LANDING_TARGET` |
| **RTK-HÅLL** | AGL < `rtk_agl` | ArUco (diagnostik, se nedan) | inget → RTK håller x/y + landar |

**Detektering körs alltid** (2026-08-21), oavsett fas — även i WAIT/RTK-HOLD, för att kunna testa
räckvidden bortom de aktiva trösklarna (syns markören på högre/lägre höjd än den skulle agera på?).
Sändning (`LANDING_TARGET`/`CONDITION_YAW`) är strikt grindad på `phase == ARUCO`, oavsett om ett
mål hittas. Overlayn i videon är **grön** i ARUCO-fasen, **röd** i WAIT/RTK-HOLD — röd = "ser den,
men skickar inget".

Alla tre trösklar (`aruco_start_agl`, `rtk_agl`, plus `yaw_start_agl` för girinriktningen — se
nedan) är **live-justerbara i webben** (kortet "Altitude thresholds") för experiment under
flygning. Default/gränser: `aruco_start_agl` 7 m (1–10), `rtk_agl` 0,3 m (0,1–2), `yaw_start_agl`
5 m (0,5–10).

Tröskeln `RTK_AGL` finns i [`droneweb/precland.py`](droneweb/precland.py).

## Plattan

Se [`landing-marker/`](landing-marker/): `marker_40cm.png` (skriv ut i **40 cm** — markören blir
30 cm, vit kant = quiet-zone) centrerad i en **Ø 62 cm matt safety-orange cirkel**. Markör =
`DICT_4X4_50` **ID 0** (måste matcha `ARUCO_ID` i precland.py). Räckvidd @ 1024×768 (sedan
2026-08-21, se Steg 3 nedan): ArUco ensam täcker hela intervallet ner till RTK-hold.

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

Sedan Steg 3 (2026-08-21) körs spårningen **alltid**, ingen Aktivera-knapp längre — se
Steg 3-avsnittet nedan.

1. Öppna webben, flyg (piloten) drönaren över plattan. Videon i precland-fliken visar redan
   den annoterade spårnings-vyn (markör-box, riktningspil, offset, fas, AGL).
2. Piloten går till **LAND** (eller **RTL**) — ArduPilot korrigerar x/y mot målet och sjunker;
   under 0,5 m släpper Pi:n och RTK håller in touchdown.
3. Pilotens läges-byte/RC vinner alltid. **Pi:n armar/flyger aldrig.**

## Inspelning (för fältanalys i efterhand)

Tryck **● Spela in** i precland-panelen. Fungerar **med eller utan** precland armerat → du kan spela
in hela flygningen. Sparar synkat i `/home/axel/recordings/`:

- **Video** `.avi` (MJPEG, 640×480, **med CV-overlay** — markör/cirkel, fas, AGL, offset).
- **Data** `.csv` — **en rad per videoruta** (ruta N ↔ rad N): `t` (epok-tid, exakt), mode, armed,
  roll, pitch, yaw, heading, alt, **agl** (TF-Luna), batteri (V/%), GPS-fix/sats, fas, mål-källa,
  offset ox/oy, `angle ax/ay` (grader), `sent` (skickades LANDING_TARGET), `scale`, samt
  `yaw_err_deg`/`yaw_cmd_deg`/`yaw_align` (girinriktning: markörens vinkelfel, den beräknade
  mål-headingen (grader, oavsett om den faktiskt skickades) och om girinriktning var på för den
  raden — se avsnittet om girinriktning nedan). CSV:ns `t` är den exakta tidsstämpeln per ruta;
  video-fps är nominell (~8 Hz på Pi 3A+).

Ladda ner via **Inspelningar**-panelen (video- + data-länkar) eller `http://dronepi:8080/recordings/<namn>.avi`.
Radera via ✕ i listan. **Analystips:** plotta CSV (agl vs t = nedstigningsprofil, ox/oy vs t =
inriktnings-konvergens, fas-övergångar, `sent`) och titta på videon för visuell kontext.

## Att verifiera / tuna

- **Kamera→kropp-mappning** i `image_to_body()` (precland.py): flytta målet mot nosen →
  `angle_x` ska bli positiv. Rotera mappningen om kameran är monterad annorlunda. **Kritiskt.**
- **Kamera-kalibrering** (schackbräde) för exakta vinklar; nu används FOV-approx.
- **Loop-takt vid 1024×768** (2026-08-21): `RATE_HZ=40` är omverifierat på Pi 5 vid 640×480 men
  INTE vid den nya upplösningen — kolla `loop_hz` i webben efter uppgradering, sänk om den inte
  håller 40.

## Test-stege (säkerhet först)

1. **TF-Luna:** `i2cdetect` = 0x10; rimlig AGL; `DISTANCE_SENSOR` syns i MP.
2. **Bänk (inga propellrar):** kamera på stativ över plattan → webben visar overlay + offset följer
   när du flyttar plattan; bekräfta `LANDING_TARGET` i MP (PrecLand-status). Mät RSS.
3. **Kalibrering** + verifiera vinkeltecken.
4. **Hovring ~3 m, armerat, INGEN LAND:** logga att korrektionerna pekar rätt.
5. **Bevakad LAND** från låg höjd → sedan full 8 m → touchdown; jämför mot ren RTK-LAND.

## Flygtest-resultat & öppna tuning-punkter (2026-07-30)

Första riktiga testflygningar gjorda (manuell nedstigning i **Loiter** + **Precision-Loiter** ~3 m
över plattan). Inspelning analyserad (`rec_20260730_162014`, 56 s: COLOR 293 rutor / ARUCO 48 / RTK-HOLD 83).

**Fynd 1 — rörelseoskärpa (vibration) → detektionsbortfall på höjd.** Skärpan (Laplacian-varians)
pendlar 70→1752 över klippet = intermittent motion blur. **COLOR-fasen: bara 22 % detekterade;
ARUCO-fasen (nära): 100 %.** Attityd-estimatet hölls dock stabilt (roll/pitch std ~1°), så
vibrationen sitter i kameran (hög frekvens). Orsak: **auto-exponering** väljer ibland lång slutartid
→ vibrationen smetar ut rutan. Effektiv färg-räckvidd blev ~4–5 m (inte 8 m) p.g.a. liten platta +
blur. *(Obs: att röda plattan renderas "blå" på låg höjd är en CV-markering, INTE vitbalansfel.)*

**Fynd 2 — Precision-Loiter överkompenserar / oscillerar** vid ~3 m (kör förbi flera gånger).
- **Uteslutet: vinkelskala.** FOV-kollen: kameran läser hela sensorn (1640×1232-läge, ScalerCrop
  100 %) → verklig HFOV = 62,2° = exakt vad koden antar. Vinklarna är alltså rätt skalade.
- **Tecknet är rätt** (den pendlar *runt* målet, divergerar inte bort) — `image_to_body` verifierad.
- **Trolig orsak: latens.** CV ~8 Hz + kedjan kamera→CV(Python)→mavproxy→FC ~150–250 ms → precland
  styr mot där plattan *var* → översläng → oscillation. Klassiskt companion-precland-problem.

**Applicerad fix:** `LANDING_TARGET time_usec=0` (commit `31a89ca`) — vi skickade epok-µs, fel
tidsbas kan förvirra precland-Kalmans latenskompensering. 0 = använd mottagningstid.

**Öppna punkter (prova nästa flygning, i ordning):**
1. **Testa med time_usec-fixen** (klar) — se om överslängen minskar.
2. **`PLND_EST_TYPE` 0 (raw) vs 1 (Kalman)** — om Kalman översvänger p.g.a. latens kan raw vara lugnare.
3. **Fast exponering under CV** (KLAR, ej flygtestad) — `camera.set_cv_exposure()`: picamera2
   `AeEnable=False` + kort `ExposureTime` + gain, sätts när CV-loopen startar och återställs till
   auto när precland avaktiveras. Tunas via `CV_EXPOSURE_US`/`CV_GAIN` i `precland.py` (default
   2000 µs / gain 2,0; tuna 1500–2500 µs efter ljus). Fryser rörelsen → skarp jämn detektering →
   mjukare data. Störst effekt mot Fynd 1. **Verifiera på bänk** att overlay-videon inte blir för
   mörk/ljus vid rådande ljus innan flygtest.
4. **Höj takten / kapa latensen** (KLAR — se Flygtest #2 nedan) — JPEG-encode + inspelning körs nu
   i en writer-tråd, kontroll-loopen skickar LANDING_TARGET vid ~8–12 Hz.
5. **Minska vibration mekaniskt** — balansera propellrar (störst), kolla motorer/prop-skick,
   mjukmontera FC. Kolla ArduPilots **VIBE**-logg (VibeX/Y/Z <30, clipping=0).
6. Kamera-kalibrering (schackbräde) — valfritt, FOV är rätt; bara distorsion kvar.

**Analysmetod:** spela in nästa försök → plotta CSV: `ox/oy` vs `t` (oscillationens amplitud +
frekvens → latensbidrag), `agl` vs `t`, fas-övergångar, `sent`. Skala/period bekräftar om fixarna hjälpte.

## Flygtest #2 — grundorsak: ÖVERSLÄNG, inte detektering (2026-08-16)

4 inspelningar analyserade. Fast exponering ~fördubblade COLOR-detekteringen (22 %→30–46 %), men
**det verkliga felet är översläng.** RTL-autolandning (`rec_20260816_074011`): precland korrigerar,
**översvänger direkt förbi mitten så markören åker ut ur den fasta kamerans smala synfält** → tappas →
ArduPilot klättrar och gör om (`PLND_RET_MAX`=4) 3–4 ggr → landar flera meter bredvid. Data bekräftar:
översvänger även från centrerat läge (centrerad vid 9 s → bildkant vid 13 s); markören ligger alltid vid
|offset| 0,8–0,98 precis före tappet; tydlig klättra/gör-om-sågtand vid ~27/42/57/71 s. Den låga
"COLOR-%" var alltså mest markör-utanför-bild, inte oskärpa. **Orsak = latensdriven översläng** (loopen
gick bara ~5 Hz + pipeline-lag) med fast kamera som inte kan hämta hem översvängen när målet lämnat bild.

**Åtgärd A — ArduPilot-params (sätts i MP; testa på bevakad LAND från ~3 m RAKT över plattan, INTE RTL):**
```
PLND_EST_TYPE = 1      # Kalman (krävs för lag-kompensering)
PLND_LAG      = 0.25   # kompensera vår sensor-latens (max; börja högt)
WPNAV_SPEED   = 200    # cm/s — mjukare horisontell omställning (från 1000)
LAND_SPEED    = 30     # cm/s — långsammare nedstigning = mer tid att stabilisera (från 50)
```
Behåll `PLND_STRICT`/`PLND_RET_MAX` tills vidare; om den fortfarande gör om är överslängen ej tämjd.

**Åtgärd B — kod (KLAR, commit `d2fe524`):** kontroll-loopen gör nu bara detektera + skicka
LANDING_TARGET (`RATE_HZ`=15, verkligt ~8–12 Hz), medan JPEG-encode + AVI/CSV-inspelning körs i en
separat **writer-tråd** via en bounded drop-oldest-kö (`_Q_MAX`=8, RAM-säkert på Pi 3A+). BGR-konvertering
hoppas över i ARUCO/RTK-fas när ingen tittar/spelar in. **OBS inspelnings-semantik ändrad:** CSV/video
samplas nu i writer-takt (kan tappa rutor vid last), INTE en rad per skick; sann skick-räkning i status
`tx`. Rollback: `precland.py.prethread.bak` på Pi:n.

## Flygtest #3 — Pi 5, 40Hz: dubbel kamera→kropp-rotation (2026-08-18/19)

Efter Pi 5-migreringen (40Hz, ~20ms latens, `PLND_EST_TYPE=1` Kalman, `PLND_LAG=0.025`) flögs
Precision-Loiter igen med mycket låg styrskala för att se korrektionsriktningen tydligt. **Samma
oscillation som förut, men nu tydligt orsakad av fel riktning, inte latens:** mål fram om nosen fick
drönaren att styra höger, mål bak → vänster, mål till vänster → framåt, mål till höger → bakåt
(fyra videobilder med inritad styrriktning, 2026-08-18) → cirklade runt plattan i stället för att
flyga in.

**Felsökning (2026-08-19):** Ett bänktest (markör fysiskt fram/bak/vänster om drönaren, avläst i
webbens Offset-fält, ingen FC inblandad) visade att **bildgeometrin var korrekt** — kameran är
monterad exakt som antaget (bild-topp = nos, ingen fysisk vridning). Det uteslöt en felmonterad
kamera. Verklig grundorsak hittad i ArduPilots källkod
(`libraries/AC_PrecLand/AC_PrecLand_MAVLink.cpp`, `handle_msg()`):
```cpp
_los_meas.vec_unit = Vector3f{-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f};  // (forward, right, down)
```
ArduPilot **roterar redan själv** från kamerans bildvinklar till BODY_FRD — den förväntar sig råa
bildvinklar (angle_x/angle_y = kamerans egna horisontell/vertikal-vinkel), inte en vinkel vi redan
roterat till kroppsram. Vår gamla `image_to_body()` (`body_x=-ay_img, body_y=ax_img`) gjorde SAMMA
rotation en gång till innan skick → nettoeffekt 90°. Matchar exakt alla fyra videobilderna (testat
i efterhand mot denna hypotes).

**Fix** (`precland.py`, `_control_tick`): skicka råa `ax, ay` direkt till `send_landing_target()`,
ingen egen kropps-transform. `image_to_body()` borttagen (ersatt med förklarande kommentar).
`PLND_YAW_ALIGN` lämnas på 0 (bänktestat: ingen fysisk vridning att kompensera för).

## Flygtest #4 — verifiering av dubbel-rotations-fixen: fungerar (2026-08-20)

Många landningar, både **LAND**- och **RTL**-läge, styrskala=1,0 (full korrektion, ingen mildring).
Snabb och exakt korrektion så fort drönaren kommer ner på rätt höjd och ser plattan — ingen
översläng alls.

Två inspelningar analyserade i detalj:
- **RTL** (`rec_20260820_002545.csv`): nedstigning 7,83 m → 0,47 m på ~13 s, ARUCO-fasens
  offset-magnitud max **0,22** (medel 0,10). Ren övergång till RTK-HOLD exakt vid 0,47 m AGL,
  ingen enda gör-om-cykel.
- **LAND** (`rec_20260820_001008.csv`): nedstigning 3,48 m → 0,52 m på ~9 s, offset max **0,62**
  (medel 0,17) — lite mer rörelse men fortfarande långt under den gamla måltapp-zonen. Viss
  fas-flimmer ARUCO↔RTK-HOLD precis vid 0,5 m-tröskeln under sista inbromsningen (normalt,
  inte samma sak som `PLND_RET_MAX`-gör-om).

Jämför med Flygtest #2 (2026-08-16) där offset regelbundet låg **0,8–0,98** precis innan målet
tappades ur bild och drönaren klättrade för ett nytt försök. Ingen sådan cykel syns i något av
dagens klipp — dubbel-rotations-fixen (Flygtest #3) var grundorsaken, inte bara latensen.

## Girinriktning mot markören — bänktest (2026-08-20)

Ny funktion: drönaren vrider sig (yaw) under nedstigningen så nosen hamnar likadant mot markören
varje gång (pilen i videon pekar rakt upp), samtidigt som den sjunker — inte stanna-och-vrida. Av/på
+ vridhastighet (grader/s) i webben (precland-panelen).

**Metod:** `MAV_CMD_CONDITION_YAW` (COMMAND_LONG), inte RC-yaw-override. Källkontroll (`mode.cpp`,
`autoyaw.cpp`) visar att ArduPilots LAND-läge (inkl. precision landing) alltid styr gir via
`auto_yaw.get_heading()`, oberoende av piloten och av position/höjd-styrningen — CONDITION_YAW sätter
`auto_yaw` i FIXED-läge mot en absolut heading med given grader/s, och FC:n rampar dit själv
kontinuerligt. RC-override hade låst ut pilotens gir-spak helt och krävt egen omsändnings-logik;
CONDITION_YAW gör varken. **OBS:** pilotens gir-spak har ingen effekt alls i LAND-läge (varken av
eller på för denna funktion) — nödstopp för fel gir-beteende är **lägesbyte**, inte spaken.

**Bänktest (utan props/arm):** samma sorts risk som dubbel-rotations-buggen (Flygtest #3) — kamerans
bildrotation mot kroppens girriktning var overifierad. Testat i två steg:
1. Markören roterad för hand under kameran (drönaren still): beräknat `yaw_err_deg` följde
   markörens rotation korrekt (0°→-1°, +90°→89°, -90°→-93°).
2. Drönaren roterad för hand ovanför en fast markör (props av, EJ armerad): jämförde beräknad
   mål-heading (`yaw` + `YAW_ERR_SIGN`×`yaw_err_deg`) före/efter en 90° handvridning — i princip
   oförändrad (296,8° → 295,8°), vilket bekräftar tecknet (fel tecken hade gett en fördubblad,
   motsatt förskjutning). `YAW_ERR_SIGN = 1` bekräftat rätt för denna kameramontering.

`yaw_err_deg` och beräknad `yaw_cmd_deg` loggas i CSV:n (full 40Hz-upplösning, oavsett om funktionen
är på) för efteranalys.

## Flygtest #5 — girinriktning i luften: fungerar (2026-08-20)

Fyra fulla landningar (LAND/LOITER och LOITER/RTL), `yaw_align` på hela flygningen, 8 m → touchdown.
Alla fyra konvergerade från stort initialt vinkelfel vid ArUco-lock (godtycklig inflygningsriktning)
till nära noll vid touchdown, utan att nedstigningen pausade:

| Inspelning | Fel vid ArUco-lock | Fel vid touchdown |
|---|---|---|
| `rec_20260820_231310` | -91,6° | 0,2° |
| `rec_20260820_232124` | 113,2° | 0,3° |
| `rec_20260820_232309` | 147,3° | 0,5° |
| `rec_20260820_232450` | 180,0° (nosen rakt bakvänd) | -0,5° |

T.ex. `_232124`: AGL 7,91→3,83→0,66→0,14 m medan `yaw` samtidigt svängde 213°→306,5°→309,3° och la sig
— konvergerar på någon sekund, nedstigningen fortsätter oavbrutet genom hela korrigeringen.
`YAW_ERR_SIGN=1` bekräftat rätt i verklig flygning, inte bara på bänken.

## Steg 3 — gråskale-spårning, alltid på, färgläge borttaget (2026-08-21)

Efter bänktest med en 1:4-skalad markör (se ovan-diskussionen i sessionen som ledde hit): ArUco-
räckvidden, inte upplösningen, var den faktiska begränsningen — färgläget (orange cirkel) gav ingen
extra räckvidd i praktiken jämfört med vad ArUco redan klarade. Ändringar:

- **Färgläge borttaget helt** — ingen HSV-detektering, ingen `COLOR`-fas. `Faser`-tabellen ovan
  uppdaterad.
- **Upplösning 640×480 → 1024×768** — fler px/grad ger ArUco längre räckvidd (samma geometri som
  markör-skalnings-testet, omvänt). `FX`/`FY`/`CX`/`CY` skalar automatiskt.
- **Gråskala** — kameran (`camera.py`) körs nu med en enda ström (ingen lores/main-uppdelning);
  detekteringen använder bara Y-planet. Overlay-färg (grön) läggs på via en billig
  `GRAY2BGR`-replikering, inte en riktig färgkonvertering.
- **Alltid på** — `PrecLandController` startar sin loop direkt vid appstart, ingen Arm/Disarm
  längre. Detta är säkert för `LANDING_TARGET` (AC_PrecLand agerar redan bara i relevanta lägen/
  faser) men INTE lika självklart för `CONDITION_YAW` (auto_yaw-målet är "sticky" och dess
  beteende vid lägesbyte är overifierat) — därför skickas `CONDITION_YAW` numera bara när
  flygläget är **LAND** eller **RTL** (`YAW_MODES` i precland.py), oavsett `yaw_align`-inställning.
- **Rå-FPV-läget borttaget** för den nedåtriktade kameran (`camera.py`) — den var bara till för
  tidig kameraexperimentering, används inte i normal flygning. All video från den kameran är nu
  spårningens egen annoterade vy.
- **Live-förhandsvisning halverad** — 512×384 @ 20 FPS (från 1024×768 @ upp till 40 FPS) för att
  spara bandbredd; bara aktiv när precland-fliken faktiskt är öppen. Inspelning (.avi) är
  opåverkad — full upplösning, oberoende takt.
- **RTK-hold-övergången återställd** vid 0,5 m (tidigare temporärt bortkopplad för bänktestet av
  markör-skalningen).

**EJ flygtestad ännu** vid denna upplösning/alltid-på-form — `RATE_HZ=40` är inte omverifierat vid
1024×768 (kolla `loop_hz` i webben efter uppgradering), och CONDITION_YAW-läges-grinden (LAND/RTL)
är ny och overifierad i luften.

## Höjdtrösklar live-justerbara i webben (2026-08-21)

De tre höjdgränserna (`aruco_start_agl`, `rtk_agl`, `yaw_start_agl`) är inte längre hårdkodade
konstanter — de går att ändra live i webben (kortet "Altitude thresholds", precland-fliken) för
att experimentera under en flygning, utan omdeploy. `PrecLandController.set_thresholds()` klämmer
var och en oberoende (ingen inbördes ordning tvingas). Se `Faser`-tabellen ovan för vad varje
tröskel styr.

| Tröskel | Default | Min | Max |
|---|---|---|---|
| `aruco_start_agl` | 7 m | 1 m | 10 m |
| `yaw_start_agl` | 5 m | 0,5 m | 10 m |
| `rtk_agl` | 0,3 m | 0,1 m | 2 m |

(`aruco_start_agl`-defaulten höjd 5→7 m och `rtk_agl` sänkt 0,5→0,3 m, 2026-08-21 efter Flygtest
#6 — matchar vad som faktiskt flögs med. `yaw_start_agl` oförändrad.)

(Justerat 2026-08-21 från de initiala default/gränserna 15 m / 3,5 m / 0,5 m, max 30/30/3 m —
snävare intervall bättre anpassade för faktisk flygtestning.)

## Flygtest #6 — alltid-på gråskale-spårning + girinriktning, full flygenvelop (2026-08-21)

Tre landningar, 8 m → touchdown, `yaw_align` på (45°/s), `aruco_start_agl` manuellt höjd till 7 m
under flygningen (rummet vid bänktestet tidigare samma dag var bara ~5,5 m, så tröskeln höjdes för
att få marginal — se defaultändringen ovan), `rtk_agl` sänkt till 0,3 m (bekräftat i efterhand
av alla tre CSV:ers WAIT→RTK-HOLD-övergång, ~0,3 m i samtliga — ursprungligen misstolkat som brus
runt 0,5 m innan piloten bekräftade det verkliga värdet).

| Inspelning | Dur | Detektion* | Längsta äkta lucka* | Offset konv. | Gir konv. |
|---|---|---|---|---|---|
| `rec_20260821_205405` | 113,8 s (LAND+LOITER) | 84,6 % | 62 rutor (~1,5 s) | 0,46→0,14 | 147°→0° |
| `rec_20260821_205628` | 49,0 s (LAND) | 93,8 % | 6 rutor | 0,28→0,12 | -175°→1° |
| `rec_20260821_205858` | 27,7 s (LAND+LOITER) | 93,8 % | 12 rutor | 0,42→0,19 | -69°→1° |

*begränsat till rutor där TF-Luna faktiskt gav ett giltigt AGL-värde — se nedan för varför.

Bekräftar: 85–94 % detektionsgrad inom den avsedda höjdbanden, korta luckor (under ~1,5 s), och
både positionsoffset och gir-fel konvergerar rent till nära noll i alla tre — även flygningen som
startade 175° feljusterad. Matchar pilotens intryck: tillförlitlig, tappar spårningen enstaka
rutor ibland, styr ändå in mot målet utan problem.

**Upptäckt i efteranalysen:** en stor andel av ARUCO-fasens rutor (upp till ~70 % i den längsta
flygningen) hade **inget giltigt AGL-värde alls** från TF-Luna — de hamnade i `_decide_phase()`s
"AGL okänt → detektera ändå"-fallback. Utan att filtrera bort dessa ser detektionsgraden mycket
sämre ut (28–65 %, "luckor" på 11–26 s) — men de långa luckorna sammanföll nästan uteslutande med
AGL=None-perioderna, inte med genuina missar inom en känd höjd. Höjdgrindningen (WAIT/RTK-HOLD)
var alltså inte aktiv under en stor del av dessa flygningar. Orsak ej utredd (TF-Luna räckvidd?
attityd-beroende? specifikt för LOITER-omplacering?) — kvarstår som öppen punkt.

## Kamera-offset från CG (2026-08-21)

Kameran sitter inte i drönarens rotationscentrum (CG) utan **16,5 cm bakom** (uppmätt). Pilotens
egna försök att kompensera via ArduPilots `PLND_CAM_POS_*`-parametrar gav ingen synlig effekt —
okänt om `AC_PrecLand_MAVLink`-backend'en (som denna LANDING_TARGET-baserade lösning använder)
stödjer den parametern alls. Kompenseras istället i `precland.py` (`_apply_cam_offset()`): målets
siktlinjevinkel, uppmätt från kameran, räknas om geometriskt till motsvarande vinkel som skulle
setts från CG innan den skickas — annars ger en ren gir en falsk skenbar målrörelse, och
positionskorrektionen blir systematiskt fel med avståndet offset/AGL.

Live-justerbar i webben (kortet "Camera calibration", "Camera offset from center"), ±30 cm,
default -16,5 cm (kroppens X-axel, framåt positivt — kameran sitter alltså på minus). Uppdatera
om kameran flyttas fysiskt.

**Flygverifierad 2026-08-23** — tecknet stämde, ingen justering behövdes (-16,5 cm rätt från start).

### FOV-konflikt nära marken (upptäckt + åtgärdad 2026-08-23)

Pilotens observation: nära marken slutade markören synas, eftersom den (korrekt!) inte längre
ska vara bildcentrerad utan förskjuten mot bildens ovankant (nos-riktning) när CG är centrerad
över målet — kameran sitter ju bakom CG. Bekräftat i flygdata: 0 % ArUco-detektion i 0,3–0,6 m
AGL-bandet (mot 74–100 % innan offset-korrigeringen fanns), 31–67 % i 0,6–1,5 m.

Orsak: vid perfekt CG-centrering måste kameran se målet vid vinkeln atan(0,165/AGL) från
bildcentrum — det når redan halva VFOV (24,4°) vid AGL ≈ 0,36 m. Under den höjden är det
geometriskt omöjligt att ha både centrerad CG och målet inom bild. Fix (`_apply_cam_offset()`,
`CAM_OFFSET_MAX_FOV_FRAC = 0,6`): klipper offseten (inte hela vinkeln — verkligt spårningsfel
klipps inte bort) till max 60 % av halva VFOV vid given AGL, så korrigeringen tonas ned mjukt
istället för att tvinga målet ur bild. Full korrektion ovanför ~0,7 m, ner till ~48 % av full
korrektion vid RTK-hold-gränsen (0,3 m) — resten av avvikelsen tar RTK-hold hand om ändå.

**EJ omflygtestad efter denna fix** — bygger på samma geometri som redan flygverifierats, men
själva FOV-klippningen är ny.

## Uppskjutet

- **Nästlad liten markör** för spårning ännu lägre än RTK-håll.
- **TF-Luna AGL=None en stor del av flygningen** (se Flygtest #6) — varför, och går det åtgärda?
  Höjdgrindningen (WAIT/RTK-HOLD) är inaktiv (faller till "detektera ändå") när det händer.
