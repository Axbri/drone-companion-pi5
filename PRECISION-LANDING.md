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

## TF-Luna (AGL via I2C)

1. Ställ TF-Luna i **I2C-läge** (default UART) med Benewakes verktyg, spara.
2. Wire: SDA→Pi pin 3, SCL→pin 5, 5V, GND.
3. På Pi:n: `dtparam=i2c_arm=on` i `/boot/firmware/config.txt`, reboot; `i2cdetect -y 1` → **0x10**.
4. `smbus2` finns redan i `droneweb-venv`. [`droneweb/rangefinder.py`](droneweb/rangefinder.py)
   läser 0x10 och **reläar `DISTANCE_SENSOR`** till FC. Saknas sensorn → `agl()` = None (graciöst).

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

## Användning

1. Öppna webben, flyg (piloten) drönaren över plattan.
2. Tryck **Aktivera** i precland-panelen (bekräfta-dialog). Pi:n skickar `LANDING_TARGET` när
   ett mål syns; videon växlar till **annoterad** vy (cirkel/markör-box, offset, fas, AGL).
3. Piloten går till **LAND** — ArduPilot korrigerar x/y mot målet och sjunker; under ~0,5 m
   släpper Pi:n och RTK håller in touchdown.
4. **Avaktivera** när som helst; pilotens läges-byte/RC vinner alltid. **Pi:n armar/flyger aldrig.**

## Att verifiera / tuna

- **Kamera→kropp-mappning** i `image_to_body()` (precland.py): flytta målet mot nosen →
  `angle_x` ska bli positiv. Rotera mappningen om kameran är monterad annorlunda. **Kritiskt.**
- **HSV-orange** (`ORANGE_LO/HI`) + `COLOR_MIN_AREA`: tuna mot din platta i verkligt ljus så att
  bara plattan triggar (inte annat orange). Fast exponering/AWB ger stabilast färg.
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
