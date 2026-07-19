# UM982 GNSS-kompass → ArduPilot — konfig & lärdomar

Konfiguration av ArduSimple **simpleRTK3B Compass** (Unicore **UM982**, dubbelantenn:
position + GNSS-heading i en enhet, en serieport) mot en Cube som flyger ArduPilot.
Detta dokument täcker GNSS-mottagaren och Cube-parametrarna. RTCM-korrektionerna kommer
från companion-Pi:n (se [`README.md`](README.md)) eller SiK.

> **Läs detta först om något strular med GPS/heading — särskilt innan du handpillar
> mottagaren i Uprecise.** Nästan alla problem vi haft kom från manuell om-konfig.

## Aktuell fungerande konfig (verifierad 2026-07-19)

**Wiring:** UM982 → Cubens **GPS1-port = SERIAL3**. Companion-Pi:n (RTCM/telemetri) →
Cubens **GPS2-port = SERIAL4** @921600. (Obs: äldre text i README/SETUP nämner TELEM1/TELEM2 —
den fysiska kopplingen hamnade slutligen på GPS1/GPS2. Det är GPS1/SERIAL3 + GPS2/SERIAL4 som
gäller.)

**Cube-parametrar (Mission Planner → Full Parameter List):**

| Parameter | Värde | Not |
|---|---|---|
| `GPS_AUTO_CONFIG` | **1** | **Låt ArduPilot konfigurera UM982:an. Rör INTE detta.** (se lärdom nedan) |
| `GPS1_TYPE` | 25 | UnicoreMovingBaselineNMEA (auto-AGRICA, heading ur AGRICA) |
| `SERIAL3_PROTOCOL` | 5 | GPS på GPS1-porten |
| `SERIAL3_BAUD` | 230 | 230400 |
| `GPS_PRIMARY` | 0 | FirstGPS = instans 0 (UM982 sitter på instans 0 = GPS1) |
| `GPS1_MB_TYPE` | 1 | Moving-baseline heading på för instans 0 |
| `GPS1_MB_OFS_X` | 0.45 | Baslinjevektor bas→heading-antenn, X=framåt (m) → orienterar yaw |
| `GPS1_POS_X` | 0 | Antennens CG-hävarm (skild från MB_OFS) |
| `GPS2_TYPE` | 0 | Instans 2 är companion-Pi:n (MAVLink), inte GPS |
| `SERIAL4_PROTOCOL / BAUD` | 2 / 921 | Companion-Pi (RTCM-injektion + 4G-telemetri) |
| `EK3_SRC1_YAW` | 3 | GPS + kompass-fallback |
| `AHRS_EKF_TYPE` | 3 | EK3 |

> **Firmware använder 4.5+-namn** (`GPS1_TYPE`, `GPS1_MB_OFS_X`, `GPS1_POS_X`) — de gamla
> namnen (`GPS_TYPE`, `GPS_MB1_*`, `GPS_POS1_*`) finns INTE.

**Resultat:** RTK-Fixed, korrekt GNSS-yaw, ~5 Hz, och ingen "Unhealthy GPS Signal" i HUD:en.

## Läxa 1 — LÅT `GPS_AUTO_CONFIG=1` STÅ, handpilla inte AGRICA

Med auto-config **på** sätter ArduPilot varje boot upp UM982:an med en *effektiv* meddelande-
uppsättning i 5 Hz som mottagaren klarar **med bibehållen RTK-Fixed + heading**. Allt strul vi
hade (Float, tappad yaw, ohälsovarning) kom från att stänga av auto-config och handtrimma
meddelande-takter i Uprecise:

- **Höjde AGRICA manuellt till 5–10 Hz** → den tunga AGRICA-strömmen översteg vad mottagarens
  RTK-/heading-motorer klarade → **RTK föll till Float och yaw försvann**. (Auto-configs 5 Hz
  funkar; en handsatt tung 5 Hz gör det inte.)
- **Stängde av auto-config** → ArduPilot slutade sätta upp dubbelantenn-**heading-läget** →
  yaw försvann helt tills auto-config sattes tillbaka.
- Ändringar i Uprecise landade ofta på **fel COM-port** (Uprecise pratar över USB, en annan
  port än den som går till Cuben) → ingen effekt på det ArduPilot läser.

**Slutsats:** behöver du ändra något GPS-relaterat, gör det med `GPS_AUTO_CONFIG=1` och verifiera
efteråt. Undvik att SAVECONFIG:a egna meddelande-takter i UM982:an.

## Läxa 2 — moving-baseline-offset är PER INSTANS (180°-fällan)

När kompassen flyttades från GPS2-porten till GPS1-porten blev headingen 180° fel, trots orörda
antenner. Orsak: `GPS1_MB_*` och `GPS2_MB_*` är **separata** parametrar. Den gamla GPS2-setupen
hade `GPS2_MB_TYPE=1` + `GPS2_MB_OFS_X=0.45`; på GPS1 låg 0.45 av misstag i `GPS1_POS_X`
(CG-hävarm, INTE heading) medan `GPS1_MB_OFS_X` var 0. `MB_OFS` = vektorn bas→heading-antenn
och är det som orienterar yaw:en; utan den blir headingen fel/180°. Fix: `GPS1_MB_TYPE=1`,
`GPS1_MB_OFS_X=0.45`, `GPS1_POS_X=0`. (`POS` ≠ `MB_OFS` — blanda inte ihop dem.)

## "Unhealthy GPS Signal" (röd HUD-text) — vad den betyder

MP målar texten när `SYS_STATUS`-GPS-hälsobiten är false, dvs ArduPilots `is_healthy()` för
GPS:en. Den kräver att **positionsuppdateringen kommer ≥ ~2,5 Hz** (tröskel = 2×`GPS1_RATE_MS`,
default 200 ms). Om UM982:an bara matar 1 Hz blir biten false → röd text, **men EKF:n fuserar
1 Hz utan problem** → armar och flyger ändå (kosmetiskt). Med `GPS_AUTO_CONFIG=1` kör den ~5 Hz
→ hälsobiten TRUE → texten borta. (Kör den av någon anledning 1 Hz är texten ofarlig.)

## Diagnostik — läsa GPS-status utan Mission Planner

Companion-Pi:ns mavproxy exponerar Cuben på `tcp:127.0.0.1:5760`. Från Pi:n (pymavlink i
`~/mavproxy-venv`) går det att läsa params och GPS-state:

- **Sann GPS-takt:** räkna *distinkta* `GPS_RAW_INT.time_usec` (inte antal meddelanden — MP
  repeterar dem i stream-takt). 1 distinkt/s = 1 Hz-mottagare.
- **Yaw:** `GPS_RAW_INT.yaw` — 65535 = ingen giltig heading; annars heading×100 (cdeg).
- **Fix:** `GPS_RAW_INT.fix_type` (6 = RTK-Fixed, 5 = Float).
- **Hälsobit:** `SYS_STATUS.onboard_control_sensors_health & 32`.
- **Params:** per-namn `param_request_read` funkar (men är lite lossy → gör 6 försök).
  `PARAM_REQUEST_LIST` forwardas INTE av mavproxy till en andra klient.

> **Viktigt:** `/dev/serial0` på Pi:n är **MAVLink-länken till Cuben** (@921600), INTE GPS:en.
> UM982:an sitter på Cubens GPS1-port — Pi:n har ingen ledning dit. Enda vägarna att inspektera
> mottagaren är (a) ArduPilot via MAVLink (ovan) eller (b) Uprecise över USB direkt till UM982:an.

## Att göra någon gång (ingen brådska)

- Stäng av de interna kompasserna när GNSS-yaw är fullt betrodd — `EK3_SRC1_YAW=3` faller annars
  tyst tillbaka på magnetometer-heading om GNSS-yaw tappas.

## Referenser

- ardupilot.org/copter/docs/common-ardusimple-rtk-gps-simplertk3b-compass.html
- ardusimple.com — simpleRTK3B Compass (UM982) med ArduPilot
