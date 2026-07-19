# Onboard RTK-companion — bygg- och konfigguide

Steg-för-steg för att sätta upp en Raspberry Pi 3A+ ombord som drar RTK-
korrektioner över 4G och injicerar dem till en Cube via serial (MAVLink),
med MAVProxy:s `ntrip`-modul. Ersätter laptop + SiK-radio för RTK i fält.

Kompanjon till [`README.md`](README.md). Matar från hemmabasen
([`../home-base/`](../home-base/)).

> Ersätt `<user>` nedan med Pi:ns Linux-användarnamn och `dronepi` med det
> hostname du väljer.

## 0. Hårdvara

| Del | Not |
|---|---|
| Raspberry Pi 3 Model A+ | 512 MB, WiFi, GPIO-UART — räcker gott |
| Mobil 4G-router (WiFi, batteri) | Pi:n ansluter via WiFi; monteras på drönaren |
| 5 V/≥3 A BEC/UBEC | Pi:ns ström från drönarbatteriet — **inte** Cube:ns telem-5V |
| 3× jumperkablar | TX/RX/GND mellan Cube TELEM2 och Pi GPIO |

## 1. Flasha Pi:n

Raspberry Pi OS **Lite** via Raspberry Pi Imager. I OS-inställningarna:
- Hostname `dronepi`, aktivera **SSH**
- **WiFi:** mobilrouterns SSID + lösen, Wireless country `SE`
- Användarnamn/lösen, tidszon `Europe/Stockholm`

Boota, och verifiera att den kommer upp på routerns nät (`ssh <user>@dronepi.local`
eller via routerns klientlista). Uppdatera: `sudo apt update && sudo apt full-upgrade -y`.

## 2. Tailscale (så Pi:n når basen)

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up          # logga in på SAMMA tailnet som basen (axel.brinkeby@)
tailscale status           # ska visa 'homebase'
ping homebase              # ska svara → MagicDNS funkar
```

Nu når Pi:n `homebase:2101/LOCAL` var den än är (över 4G).

## 3. Serial-wiring: Cube TELEM2 → Pi GPIO

Cube TELEM2 är JST-GH 6-pin. Koppla **korsat** TX↔RX + gemensam jord:

| Cube TELEM2 | → | Pi (Model A+) |
|---|---|---|
| pin 2 (TX) | → | **RXD** GPIO15 — phys pin 10 |
| pin 3 (RX) | → | **TXD** GPIO14 — phys pin 8 |
| pin 6 (GND) | → | GND — phys pin 6 |
| pin 1 (5V), CTS, RTS | | **lämna okopplade** |

Båda sidor är 3.3 V TTL → säkert utan nivåomvandlare. Driv Pi:n från BEC:en, inte
från Cube:ns 5V.

## 4. Frigör Pi:ns UART

På Pi 3 sitter den stabila PL011-UART:en (`ttyAMA0`) på Bluetooth som standard, och
mini-UART:en (ostabil vid hög baud) hamnar på GPIO14/15. Vi flyttar PL011 till
GPIO-pinnarna och stänger serie-konsolen.

Redigera `/boot/firmware/config.txt`, lägg till:
```
enable_uart=1
dtoverlay=disable-bt
```
Stäng serie-konsolen och Bluetooth-UART-tjänsten:
```bash
sudo raspi-config       # Interface Options → Serial:
                        #   login shell over serial?  -> No
                        #   serial port hardware?     -> Yes
sudo systemctl disable hciuart
sudo reboot
```
Efter reboot pekar `/dev/serial0` på `/dev/ttyAMA0` (PL011) → stabilt @921600.
Kontroll: `ls -l /dev/serial0` ska länka till `ttyAMA0`.

## 5. MAVProxy

Trixie blockerar rå `pip` (PEP 668) — installera i en venv (eller `pipx`):
```bash
sudo apt install -y python3-venv python3-pip python3-dev
python3 -m venv ~/mavproxy-venv
~/mavproxy-venv/bin/pip install --upgrade pip MAVProxy future
```
> **Obs:** `future` är ett beroende MAVProxy inte drar in automatiskt på Python
> 3.13 — utan det kraschar den med `ModuleNotFoundError: No module named 'future'`.
> Lägg även `axel` (din user) i `dialout`-gruppen för serieåtkomst:
> `sudo usermod -aG dialout $USER` (logga ut/in för att det ska gälla).
Testkör och konfigurera `ntrip` interaktivt:
```bash
~/mavproxy-venv/bin/mavproxy.py --master=/dev/serial0 --baudrate 921600
```
I MAVProxy-konsolen:
```
module load ntrip
ntrip set caster homebase
ntrip set port 2101
ntrip set mountpoint LOCAL
ntrip set username anon        # LOCAL är anonym → dummy funkar
ntrip set password anon
ntrip start
ntrip status                   # ska visa ansluten + injektion
```
Du ska också se heartbeats från Cube:n i konsolen (serial-länken frisk).

> **Viktigt beteende:** ntrip-modulen visar `Start delayed pending position` tills
> autopiloten skickar en GPS-position över MAVLink. Det är **normalt** — modulen
> börjar dra/injicera korrektioner först när Cube:n är ansluten och har en 3D-fix.
> Utan Cube (t.ex. torrkörning) förblir den "pending position". Så full ntrip-test
> kräver att Cube:n är inkopplad.

## 6. Cube-parametrar (Mission Planner)

På TELEM2 = SERIAL2:
```
SERIAL2_PROTOCOL = 2      # MAVLink2
SERIAL2_BAUD     = 921    # 921600
```
UM982-kompassen på TELEM1/SERIAL1 lämnas orörd.

**Felsökning** om ingen data/flaky länk:
- Stäng av flödeskontroll på TELEM2 (bara TX/RX/GND draget): sätt porten till
  `BRD_SERx_RTSCTS = 0` (rätt index för TELEM2 på din Cube).
- Sänk baud: `SERIAL2_BAUD = 115` (115200) och `--baudrate 115200` i MAVProxy.
- Dubbelkolla att TX/RX inte är omkastade.

## 7. Autostart vid boot

När PoC:n fungerar, installera systemd-tjänsten så korrektionerna startar av sig
själva vid påslag (justera sökväg till mavproxy + user i unit-filen först):
```bash
sudo install -m 644 systemd/mavproxy-ntrip.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now mavproxy-ntrip
journalctl -u mavproxy-ntrip -f
```

## Verifiering

1. **På Pi:n:** `ntrip status` = ansluten + injektion; heartbeats från Cube:n.
2. **RTK utan laptop/SiK:** koppla bort SiK-injiceringen i Mission Planner — GPS:en
   ska nå **RTK Fixed** med enbart onboard-Pi:n som korrektionskälla.
3. **Fälttest:** upprepa RTL/landnings-testet med bara Pi:n — samma dm-precision.

## PoC-varningar

- **2.4 GHz WiFi** kan störa en 2.4 GHz RC-länk och bör hållas borta från GNSS-
  antennerna — montera genomtänkt, använd 5 GHz om routern kan.
- Beror på att **hemmabasen är online** + Tailscale (RTK2Go som ev. fallback:
  byt `ntrip set caster rtk2go.com` / `mountpoint Pryssgarden` / `username <din-epost>`).
- MAVProxy ntrip injicerar ~1–2 Hz — fullgott för RTK.
- **Ström:** underdimensionerad matning → brownout. Använd en riktig 5 V/≥3 A BEC.

## Felsökning (hårt lärda läxor)

**Ingen kontakt / 0 byte på serieporten** — nästan alltid **TX/RX korsat fel**.
Diagnos-stegen som funkar (kör på Pi:n):
1. `stty -F /dev/serial0 921600 raw -echo; timeout 4 cat /dev/serial0 | wc -c` — 0 byte
   betyder ingen signal in (inte fel baud; fel baud ger skräptecken).
2. **Pin-loopback:** bygla Pi pin 8↔pin 10 → `looptest.py` → bekräftar att Pi:ns UART är OK.
3. **Kabel-loopback:** kortslut de två datatrådarna i *Cube-änden* → `looptest.py`.
   Bekräftar kabel + Pi-pinnar — men **avslöjar INTE korsningen** (kortslutning
   loopar oavsett).
4. Om 1–3 är OK men fortfarande 0 byte: **korsningen är fel** → byt plats på de två
   datatrådarna vid Pi-headern (pin 8 ↔ pin 10). Cube-TX (GPS2 pin 2) *måste* nå
   Pi-RX (GPIO15/pin 10).

`fuser -k /dev/ttyAMA0` frigör porten säkert (device-baserat) — använd **inte**
`pkill -f mavproxy.py` i ett skript, det matchar och dödar skalets egen kommandorad.

**systemd-tjänsten failar direkt** med `Permission denied: 'mav.tlog'` → MAVProxy
skriver sin logg i arbetskatalogen; sätt `WorkingDirectory=/home/<user>` i unit-filen.

**RTK når bara DGPS på bänken** — normalt. Korrektionerna appliceras (3D→DGPS bevisar
det), men RTK Float→Fixed kräver fri himmel. Testa utomhus.

## Relaterat

- [MAVProxy NTRIP-modul](https://ardupilot.org/mavproxy/docs/modules/ntrip.html)
- [ArduPilot: Raspberry Pi via MAVLink](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html)
- [ArduSimple: NTRIP till ArduPilot med MAVProxy](https://www.ardusimple.com/send-ntrip-corrections-to-ardupilot-with-missionplanner-qgroundcontrol-and-mavproxy/)
