# Landing on a moving platform (investigation 2026-09-15, Pi side built 2026-09-16)

Goal: land reliably on a marker pad carried by a rover/car moving 1–2 m/s in a straight line.
Constraints: marker + camera only, no electronics on the pad. Builds on
[`PRECISION-LANDING.md`](PRECISION-LANDING.md).

## Conclusion

ArduPilot already supports moving landing targets natively: with `PLND_EST_TYPE=1` (Kalman, already
set) the precland EKF estimates target **velocity** as well as position, and `PLND_OPTIONS` bit 0
("Moving Landing Target") feeds that velocity forward into the LAND-mode position controller
(`ArduCopter/mode.cpp`, `land_run_horizontal_control` → `input_pos_vel_accel_NE`). No Pi-side
velocity estimation or GUIDED controller is needed. The work is: FC parameters, a small
"moving target" mode in `precland.py`, a bigger pad, and — the real problem — keeping the marker
in sight until touchdown, because the FC's target-lost timeout is a **hard-coded 2 s**
(`LANDING_TARGET_TIMEOUT_MS = 2000` in `AC_PrecLand.cpp`, not `PLND_TIMEOUT`).

## What ArduPilot does (verified in master source, 2026-09-15)

| Mechanism | Behaviour | Consequence for a moving pad |
|---|---|---|
| EKF (`PLND_EST_TYPE=1`) | 2-state (pos, vel) per NE axis; vehicle IMU delta-velocity is the process input, our LOS measurement the update. Init assumes target static (rel. vel = −vehicle vel). | Velocity converges in a few seconds after first lock. `PLND_ACC_P_NSE` (default 2.5) = how fast it re-learns velocity. |
| `PLND_OPTIONS` bit 0 | `get_target_velocity_ms()` returns rel. vel + vehicle vel; without the bit it returns 0. | **Must be set.** Otherwise the drone always steers to where the pad *is*, lagging a moving pad by ~vel × controller time constant. |
| Dead reckoning | `run_output_prediction()` propagates target pos/vel with IMU between measurements. `target_acquired()` stays true until 2 s after the last *fused* measurement. | Gaps < 2 s are fine — the drone keeps flying at the target's velocity. Gap ≥ 2 s → `prec_land_active=false` → horizontal target velocity 0 → **drone brakes** while the rover drives on. |
| NIS outlier rejection | Measurement rejected if NIS ≥ 3, unless 3 rejections in a row. | A sudden rover speed change is accepted after ≤3 frames (~0.2 s). Not a problem. |
| `PLND_OPTIONS` bit 2 | Disables the 0.35–2 m "slow to 10 cm/s if XY error > 3 cm" rule in `land_run_vertical_control`. | **Must be set.** With a moving pad there is always some error → 10 cm/s → 0.6 m of blind descent would take 6 s ≫ 2 s. |
| `PLND_OPTIONS` bit 1 | Precland resumes after the pilot nudges the sticks. Without it one stick input disables precland for the rest of the landing. | Set it: the pilot nudging to help catch up must not turn the landing blind. |
| `PLND_STRICT` / `PLND_RET_MAX` / `PLND_RET_BEHAVE` | On loss (above `PLND_ALT_MIN`): wait `PLND_TIMEOUT`, then retry by climbing to where the target *was* (static position). Strict=2: hold position when retries are exhausted. | A retry to a stale position is useless for a moving pad; hovering is the safe failure. Suggest `PLND_STRICT=2`, `PLND_RET_MAX=0`. |
| `PLND_ALT_MIN` (0.2) | Below it (per FC rangefinder) a lost target is "out of range": keep landing vertically, no retry. | Fine as is. |
| Horizontal speed limit in LAND | `ModeLand::init`: `WPNAV_SPEED` / `WPNAV_ACCEL`. | Must comfortably exceed pad speed. **Check the FC** — the August note suggested lowering `WPNAV_SPEED` to 200 (= 2 m/s, zero margin). |
| Land detector | Needs throttle at minimum, filtered accel < 1 m/s², |vz| < 1 m/s, rangefinder < 2 m, for 1 s. **No horizontal-velocity check.** LAND disarms as soon as `land_complete` + ground idle. | Landing while moving with the pad works. The rover must not brake/accelerate until the drone has disarmed (~2–4 s after touchdown). |
| Pilot abort | Stick input (`LAND_REPOSITION=1`) or mode switch overrides precland. | Unchanged. Mode switch to LOITER is the abort. |

## The critical number: the blind window

Flight data with the static pad: ArUco detection is 0–6 % below 0.6 m AGL (the 30 cm marker fills
the frame and the CG-offset compensation pushes it to the image edge; see the FOV-conflict section
in `PRECISION-LANDING.md`). So tracking effectively ends at ~0.6 m and the FC dead-reckons the rest.

| Final descent rate | Blind time for 0.6 m | Verdict |
|---|---|---|
| 10 cm/s (precland slow-down rule) | 6 s | target lost → brake → lands 1–2 m behind the pad |
| 25 cm/s (`LAND_SPEED` before 2026-09-16) | 2.4 s | lost just before touchdown |
| 30 cm/s (**current**, since 2026-09-18) | 1.8 s | inside the FC's 2 s, margin from the Pi coast |
| 40 cm/s (flown 2026-09-17) | 1.5 s | OK with margin, but hard touchdown |
| 50 cm/s | 1.2 s | OK |

Braking at `WPNAV_ACCEL=150` from 2 m/s takes 1.3 s / 1.3 m, so a timeout at touchdown means
landing well behind the pad or being dragged off it. Decision (2026-09-16): keep the single
30 cm marker (with the wide lens it stays in view until the gear is ~20 cm above the pad, so
the blind window is short), descend faster near the ground (`PLND_OPTIONS` bit 2 +
`LAND_SPEED` 30–40), and have the **Pi coast** — track the pad's velocity and, once the marker
leaves the frame, keep feeding the FC an extrapolated target at constant velocity. The FC's own
2 s dead reckoning is then a backup behind the Pi's, not the only thing holding the landing.

## Changes

### FC parameters (set via MAVLink 2026-09-16, Copter 4.6.3)

```
PLND_OPTIONS   = 7      # bit0 moving target + bit1 resume after reposition + bit2 fast final descent
PLND_STRICT    = 0      # land vertically if the target is lost. Was 2 (hold) 2026-09-16..21.
                        # NOTE: this was NOT the cause of the armed-after-touchdown problem
                        # (see flight test #3) — either value leaves a stale position target.
                        # 0 is kept because an in-air loss then lands vertically where it is
                        # rather than hovering; pilot aborts with the mode switch.
PLND_RET_MAX   = 0      # retries go to a stale (static) position — pointless for a moving pad
LAND_SPEED     = 25     # cm/s. 40 (09-17) flew fine but landed hard → 30 (09-18) → read back
                        # as 25 on 09-24, origin unclear. In practice fine: sightings now reach
                        # 0.3-0.8 m and touchdown follows within 0.3-1.7 s.
WPNAV_SPEED    = 500    # already 500 (the August "200" was never applied); ≥ 2× pad speed
WPNAV_ACCEL    = 150    # left as is
PLND_LAG       = 0.05   # keep; Pi latency measured 44 ms median
LOG_DISARMED   = 1      # only during bench step 2, set back to 0 afterwards (done)
```
Set from the Pi with `tools/fc_params.py` (`get`/`set` through mavproxy's tcp:5760).
Other FC values read the same day: `LAND_ALT_LOW 130`, `LAND_SPEED_HIGH 100` (so `LAND_SPEED`
only governs the last 1.3 m), `PLND_TIMEOUT 4`, `PLND_ALT_MIN 0.2`, `PLND_ALT_MAX 20`.

Keep `PLND_EST_TYPE=1`, `PLND_ALT_MIN=0.2`, `PLND_XY_DIST_MAX=5`. Static-pad landings must still
work with these settings (bit 0 estimates ~0 velocity for a static pad) — fly one regression
landing before any moving test. Add the values to the param backup once flown.

### Pi (`droneweb/precland.py`) — implemented 2026-09-16; flown 2026-09-17 with the mode off, coast path still unflown

A **Target mode** card in the precland tab: *Moving pad* checkbox (off = static pad = the
flight-proven behaviour, byte-for-byte the same sending logic; **on by default** since
2026-09-17) and a *Coast after marker lost* slider (0–8 s, default 3). The card turns amber
while moving mode is on.

Moving mode changes three things:

1. **No RTK-hold cut-off** — `LANDING_TARGET` is sent whenever the marker is seen, down to
   touchdown. Every sighting in the last metre resets the FC's 2 s timer.
2. **Marker range instead of lidar** — `distance` = marker line-of-sight range
   (`Target.range_m()`, scale-corrected), effective AGL = marker-derived. The pad is raised;
   the TF-Luna reads the ground beside it until the drone is right over it, and lidar AGL as
   `distance` would overstate the lateral error by (ground AGL / pad AGL) — an unintended
   extra gain. Lidar stays the fallback.
3. **Coasting** (`PadTracker`) — each sighting is converted to a pad position in the FC's
   local NED frame: drone position (`LOCAL_POSITION_NED`, now requested at 20 Hz, velocity-
   extrapolated to the frame's exposure time) + `R_bn` (ATTITUDE) × line-of-sight vector
   (raw camera angles, along-axis distance, camera offset added back — true geometry, not the
   FOV-tapered angles that are sent). Velocity = least-squares line through a 1.5 s window
   (needs ≥6 samples over ≥0.5 s). When the marker is lost, the pad is extrapolated at that
   velocity and a synthetic `LANDING_TARGET` (angles + distance, CG-referenced) is sent each
   tick for up to `coast_s` after the last sighting — only if the last sighting was itself
   actively sent (phase ARUCO), the NED data is < 0.5 s old and the prediction is below the
   drone. Target shows **coast** in the UI/CSV, a yellow circle marks the predicted pad in
   the video. The tracker runs in both modes (pad speed/heading shown in the card); only 1–3
   are gated on the switch.

Offline checks (stubbed camera/FC, `scratchpad/test_padtracker.py`, `test_tick.py`): pad at
1.5 m/s with 0.3° angle / 2 % range noise at 6 m → fitted velocity within 3 cm/s, 7 cm
prediction error after 1.5 s of coasting, synthetic angles within 1° of a real sighting;
static pad → 0.02 m/s. Tick test: 30 sightings, marker lost, 40 coast ticks over 2.0 s then
silence; static mode sends nothing after loss.

CSV gains `moving, gs, dn, de, dd, pn, pe, pd, pvn, pve, marker_cm` (mode flag, FC groundspeed,
drone NED, pad NED per sighting / prediction, fitted pad velocity, marker size) and
`source = coast`.

**Marker size slider** (calibration card, 5–100 cm, default 30): the marker-derived range/AGL,
the calibration card and hence moving-mode pad speed scale with it — set the bench marker's
true size (14.5 cm) indoors, else they read off by the size ratio.

## Bench results (2026-09-16)

Drone on a table, disarmed, 14.5 cm marker on the floor 0.70 m below (≈ the 30 cm pad from
1.5 m). Indoors the FC's EKF has no horizontal position (const-pos mode, GPS h_acc ~270 m)
and sends no `LOCAL_POSITION_NED` — the tracker's disarmed "bench: drone static" fallback
covers this; in the air the card must show `fit` without that suffix.

**Step 1 — Pi tracker** (`rec_20260916_212758`): static 0.00 m/s; slides tracked with the right
compass heading (nose = yaw); coast engaged on the first missed frame, sent for 3.0 s, `pn/pe`
continued on the fitted line (worst step 2 cm), also bridged single-frame dropouts. Dark room →
auto exposure → ~11 Hz loop (exposure-bound; 40 Hz with a lamp).

**Step 2 — FC side** (FC log 45 after a reboot, `LOG_DISARMED=1`; Pi watch t = FC t − 100.4 s):
`PL` units are cm / cm/s. Static: vX/vY within ±0.5. Slide toward the nose: FC vX +12…+18,
Pi 0.13–0.19 m/s @ 358–6°. Backward: FC (−18.5, −3.1) = 0.19 m/s @ 189.5°, Pi 0.18 m/s @ 189°.
Coast: the FC fused the Pi's synthetic targets as ordinary measurements (`mX` continued along
the same −18 cm/s line, `TAcq=1`, no `EKFOutl`), then dead-reckoned 2.0 s more and reported
`Target Lost` — confirming the hard-coded 2 s. Re-acquisition: `Target Found` → `Init Complete`
takes **2 s** before `TAcq=1` again, so a gap > 2 s in flight costs ~4 s without corrections.
`PLND_OPTIONS` bit 0 confirmed active (velocity estimated and carried through the coast).

### Pad and vehicle

- **Bigger pad.** Static-pad error is 0.1–4 cm; expect 10–30 cm with a moving pad (velocity
  transients, wind). Pad ≥ 1 × 1 m, flat, level, high-friction surface (rubber mat), a 2–3 cm rim.
  Matte print (already noted in flight test #7).
- **Pad height** as low as practical for the first tests (cart/trailer 0.3–0.5 m) so the lidar
  jump is small.
- **Vehicle**: straight, constant speed, no braking until the drone has disarmed. Path length:
  a LAND from 8 m takes ~15 s (1.2 m/s above 2.5 m, then 40 cm/s) → 30 m at 2 m/s. From 3–4 m:
  ~10 s → 20 m. Start low.
- The existing ArduRover (0.2 m/s) is fine for the first moving test — the mechanism is the same,
  only the required margins scale with speed.

## Test ladder

1. **DONE 2026-09-16** — Bench, FC powered, not armed, `LOG_DISARMED=1`: slide the pad under the drone at ~1 m/s.
   Dataflash `PL.vX/vY` should follow the pad velocity within a couple of seconds, `PL.TAcq=1`
   throughout, and the web card's pad speed/heading should agree with it and with the tape
   measure/stopwatch. Then cover the marker: target shows *coast*, the yellow circle keeps
   moving with the (hidden) pad, sending stops after the coast time. Validates
   `PLND_OPTIONS`, EKF velocity, the marker-range `distance` and the Pi tracker without
   flying. Also confirms the Pi's loop rate is unaffected (`loop_hz`).
2. **DONE 2026-09-17** — Static pad, new params: one LAND regression. Check the final-metre descent rate (CSV `agl`
   vs `t`) — this is the blind-window number.
3. **DONE 2026-09-17/21** (pad dragged on a rope, 0.9–1.9 m/s) — Slow pad: pad on a cart pulled by a ≥ 10 m rope at 0.5 m/s. First Precision-Loiter at
   3–4 m (tracking only, no descent), then LAND from 3 m. Analyse: `PL.vX/vY` vs actual speed,
   CSV `ox/oy` during descent, last-detection AGL, touchdown offset on the pad.
4. **1–2 m/s done 2026-09-21 (dragged pad, from ~10 m)**; rover/car and a proper cart still to do.
5. In the CSVs: last real sighting AGL, coast duration, `pn/pe` continuity across the
   real→coast transition, and touchdown offset on the pad. Tune `coast_s`/`LAND_SPEED` from that.

Abort at every step: mode switch to LOITER. Keep the rover's path clear of people; the drone
descends along it.

## Flight test #1 — pad dragged on a rope (2026-09-17)

Eight recordings (`rec_20260917_175409` … `180452`): 2 static-pad landings, one 152 s
Precision-Loiter follow at ~4.4 m, 5 LAND on the dragged pad (0.9–2.0 m/s), one aborted LAND
(pad never in view). Pilot's verdict: hits within 30–40 cm every time, tracks slow walk and
faster pulls, overshoots when the pad stops abruptly, works hard on yaw when the pad twists on
the grass.

**Moving mode was OFF in every recording** (`moving=0`; the Pi booted in static mode and the
switch is not persistent — default changed to ON afterwards, see Status). Everything below is therefore ArduPilot's own velocity feed-forward
(`PLND_OPTIONS` bit 0) plus the new FC params — the Pi's coasting / marker-range path is still
unflown. Yaw-align was on.

- **Tracker in flight:** `LOCAL_POSITION_NED` flows (EKF has GPS). Hovering over the static pad:
  pad speed 0.01–0.03 m/s. During a dynamic descent over the static pad: 0.05–0.2 m/s (attitude/
  position timing noise) — good enough for a 1–2 s coast at 1–2 m/s. Pulls read 0.9, 1.3,
  1.5 → 3.9 m/s with the direction of the rope.
- **Follow (LOITER):** matched 0.9–1.5 m/s with the marker within ±0.3 of frame centre. Stop
  from 1 m/s → 0.85 m overshoot; stop from 3+ m/s → 2.3 m. At 3.9 m/s the drone lagged 1.5 m,
  the marker left the bottom of the frame, **lost for 7 s**, FC braked to a stop after 2 s —
  reacquired only because the pilot stopped the pad inside the 4.3 m footprint. 2 m/s is the
  realistic ceiling with the current controller dynamics.
- **Landings:** last sighting always at **0.50–0.68 m AGL** with `oy` −0.55…−0.63 (marker
  parked at the top of the frame exactly where the CG-offset geometry puts it, then its quiet
  zone leaves the frame). Touchdown **0.9–1.7 s later** — inside the FC's 2 s dead-reckoning, so
  the target was never lost before touchdown. Lateral offset at the last sighting `ox`
  −0.06…−0.46 (≈ 3–30 cm at that height): most of the 30–40 cm is the **chase oscillation**
  during the descent (ox swinging ±0.3–0.4 with ~2.5 s period, roll/pitch ±6–10° at 2 m/s),
  not the blind phase. After touchdown at 2 m/s the pitch read −27…−29° for a second (drone on
  the pad being dragged over grass).
- Detection 3–20 m: 70–100 %, max gap 1.6 s (close to the 2 s limit — coasting matters here too).
- Yaw: converged 178° → 2° during the descent in LAND even with the pad twisting; in LOITER the
  drone does not yaw (CONDITION_YAW is LAND/RTL-only by design), `yaw_err` just tracks the twist.

Next (done 2026-09-21, flight test #2 below): fly the same with **Moving pad ON**. Still open:
the chase oscillation (`PLND_LAG`, `PLND_ACC_P_NSE`, `PSC_VELXY_*`) — that, not the blind
window, is what limits the 30–40 cm.

## Flight test #2 — moving mode on (2026-09-21)

Nine recordings (`rec_20260921_171056` … `172315`): static pad, a 125 s follow, six dragged-pad
landings at 1.1–1.9 m/s, `moving=1` throughout. Pilot: works well, coast works; but after
landing on the moving pad the drone stayed armed, sometimes rocking/almost tipping, sometimes
just idling the props.

- **Coast:** engaged in every landing, bridged mid-descent gaps, ran 3.0 s after the last
  sighting. Sightings now reach 0.36–0.67 m AGL (no RTK-hold cut-off).
- **Disarm delay after touchdown:** static pad 3.5 s; moving pad 2.7 / 4.9 / 7.6 / 8.8 /
  10.9 / **16.6 s**. In the worst case the drone sat perfectly still (pitch −2°, gs 0) for
  13 s before disarming.
- **Root cause (FC log 48, `tools/fc_log_landings.py` + `fc_log_window.py`):** throttle was at
  0 the whole time (`motor_at_lower_limit` OK), but the **pitch request sat at −30°
  (`ANGLE_MAX`)** from touchdown until 0.5 s before disarm: the position controller held a
  target 1.2 m away and demanded 1.2 m/s toward it (`PSCN` target-pos −10.97 vs pos −9.76,
  target-vel −1.21 = `PSC_POSXY_P` × error). A lean request > 15° is a land-detector veto
  (`large_angle_request`). The target came from ArduPilot's lost-target handling under
  `PLND_STRICT=2`: after `Target Lost` (Pi coast 3 s + FC 2 s after touchdown) it waits
  `PLND_TIMEOUT`, then retry/failsafe steer toward the pad's *last dead-reckoned position* —
  1.2 m ahead once the pad had been stopped. The static pad escapes because `LAND_COMPLETE`
  fires before the 2 s target timeout. The rocking = the same lean request while the throttle
  was still spooling down.
- **Fixes:** `PLND_STRICT=0` (lost → land vertically → horizontal controller relaxes →
  landed within ~1 s). Pi: **touchdown ends the coast** — in moving mode, in LAND, with the
  last sighting < 1 m, once the FC's EKF vertical speed has been < 0.15 m/s for 0.5 s no more
  coast targets are sent (`PadTracker` status `touchdown`, CSV `td`). Decided from
  `LOCAL_POSITION_NED` vz, not the lidar, and it disarms nothing: a false trigger only ends
  the coast early and the FC lands vertically. Both deployed 2026-09-21, not yet flown.
- Still open for the rover: a landing-gear contact switch → Pi force-disarm, gated on switch
  closed ≥ 100 ms AND LAND AND fresh valid lidar < 0.3 m AND vz ≈ 0 — a hung lidar (stale)
  or a reflection (switch open) each veto; both failing at once bounds the drop to ~30 cm.
  Speed-up only; not needed for the fix above.

## Flight test #3 — the disarm deadlock, root cause and real fix (2026-09-24)

Six recordings (`rec_20260924_182437` … `183919`), moving mode on, pad dragged at 0.6–1.8 m/s,
plus FC log 50. Landings themselves were good (last sighting 0.32–0.83 m AGL, touchdown
0.2–1.7 s later, coast working). **The disarm problem was not fixed** by `PLND_STRICT=0` +
"touchdown ends coast": the pilot had to disarm manually on most landings, and in several the
drone tipped or nearly tipped while still armed.

**Measured (FC log 50, the landing in `rec_20260924_182437`):** touchdown ≈ 91 s, throttle
`ThO` 0.00 from 91.0 s on, drone physically still (roll/pitch −3.8/−2.9 constant, EKF velocity
0.01 m/s, lidar 0.12 m) — and **`DesPitch` pinned at −29° for 21 s**, until the pilot disarmed
at 112.6 s. `PSCN` shows why: position target 23.11 m vs position 22.34 m = **0.77 m frozen
error**, target velocity 0.76 m/s (= error × `PSC_POSXY_P` 1.0).

**Mechanism (Copter 4.6 `Mode::land_run_horizontal_control`, read in full):**
1. `prec_land_active = !land_repo_active && precland.target_acquired()`.
2. The Pi stops sending at touchdown → `target_acquired()` expires 2 s later → Copter takes the
   **non-precland** branch: `input_vel_accel_NE_m(0, 0)`.
3. That integrates velocity into the position target, so with zero velocity input the position
   target **freezes where it was** — ~0.8 m ahead, where the pad was heading at touchdown. It
   is never reset to the drone's position.
4. Position P demands 0.77 m/s, the velocity PID winds its I-term to `ANGLE_MAX` → lean request
   29° → land detector's `large_angle_request` (> 15°) vetoes `land_complete`.
5. The one thing that would relax the target, `NE_soften_for_landing()`, is called **only while
   `land_complete_maybe`** — which the same veto clears 0.2 s after touchdown.
   Self-sustaining deadlock; it never resolves on its own.

So `PLND_STRICT` was never the real cause (it only chose *which* stale target was used), and
"stop sending at touchdown" actively caused this variant by letting `target_acquired()` expire.
Note for reading older logs: `DISARMED` + `LAND_COMPLETE` + `LAND_COMPLETE_MAYBE` at the *same*
timestamp = a manual disarm; a genuine land-detector disarm shows `LAND_COMPLETE_MAYBE`, then
`LAND_COMPLETE`, then `DISARMED` ~0.5 s later. By that test the only genuine auto-disarms in
this whole campaign were on the static pad.

**Fix (2026-09-24, deployed, not yet flown): touchdown sends a LEVEL target instead of going
silent.** Same trigger as before (moving mode, LAND, last sighting < 1 m, EKF vz < 0.15 m/s for
0.5 s, armed), but instead of stopping, the Pi sends `LANDING_TARGET` with `angle_x = angle_y =
0` and `distance` = lidar AGL, every tick. `target_acquired()` stays true → Copter keeps the
precland branch → `input_pos_vel_accel_NE_m(target_pos = our own position, vel = 0)` pulls the
position target onto the drone → lean request collapses → the land detector can finish.
Source/CSV shows `level`. Still no disarm authority on the Pi, still not lidar-gated: a false
trigger in the air only says "the pad is straight below", i.e. descend vertically — which is
what a lost target does anyway.

**Next flight:** on the moving pad, `DesPitch` must stay small after touchdown and
`LAND_COMPLETE_MAYBE → LAND_COMPLETE → DISARMED` must appear in that order, ~1–2 s apart, with
no pilot action. If the lean still builds, the remaining options are a landing-gear contact
switch → gated Pi force-disarm (see below), or `ANGLE_MAX`-independent tricks on the FC side.

**Also found:** `LAND_SPEED` is back to **25** (it was set to 30 on 2026-09-18 and verified
after a reboot on 09-21). Not changed back — check whether this was deliberate.

## Status

Bench steps 1–2 done 2026-09-16; flight #1 09-17 (mode off), #2 09-21 (mode on, coast works),
#3 09-24 (disarm deadlock root-caused, level-target fix deployed — **not yet flown**).
Landing accuracy 30–40 cm is limited by chase oscillation, not by the blind window. FC is in flight configuration (`LOG_DISARMED=0`),
Pi defaults to **moving mode ON** (since 2026-09-17, so it is not forgotten; harmless on a
static pad) and 30 cm marker at every restart. Untick it for a static-pad comparison landing.
Log tools on the Pi (`/home/axel/tools/`, repo `tools/`): `fc_params.py`, `fc_logs.py`
(list/download dataflash logs to `/home/axel/fclogs/`), `fc_log_landings.py` (events, TAcq,
throttle before each disarm), `fc_log_window.py` (attitude request/actual, EKF velocity,
position-controller targets in a time window).

## Open points

- ~~FC firmware version~~ — Copter 4.6.3, `PLND_OPTIONS` present and confirmed working (bench step 2).
- ~~Current `WPNAV_SPEED`/`WPNAV_ACCEL`/`LAND_SPEED`~~ — read and set 2026-09-16, see above.
  Refresh the param backup in the repo once flown.
- Does the 0.35–2 m slow-down rule actually engage today? The static-pad CSVs show ~33 cm/s
  average in the last 3 m, so maybe not on this firmware — check `agl` vs `t` in the last 2 m.
- Lidar mounting position vs pad: at touchdown the TF-Luna must see the pad, not the ground
  beside it (land detector's rangefinder < 2 m check is fine either way, but AGL logging isn't).
- Wind: a 5 m/s headwind on the rover's path means the drone flies pitched — more time with the
  marker near the top image edge (the 4 m dip in flight test #7).
