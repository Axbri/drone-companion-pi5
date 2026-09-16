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
| 25 cm/s (current `LAND_SPEED`) | 2.4 s | lost just before touchdown |
| 40 cm/s | 1.5 s | OK with margin |
| 50 cm/s | 1.2 s | OK |

Braking at `WPNAV_ACCEL=150` from 2 m/s takes 1.3 s / 1.3 m, so a timeout at touchdown means
landing well behind the pad or being dragged off it. Decision (2026-09-16): keep the single
30 cm marker (with the wide lens it stays in view until the gear is ~20 cm above the pad, so
the blind window is short), descend faster near the ground (`PLND_OPTIONS` bit 2 +
`LAND_SPEED` 40), and have the **Pi coast** — track the pad's velocity and, once the marker
leaves the frame, keep feeding the FC an extrapolated target at constant velocity. The FC's own
2 s dead reckoning is then a backup behind the Pi's, not the only thing holding the landing.

## Changes

### FC parameters (Mission Planner / MAVLink)

```
PLND_OPTIONS   = 7      # bit0 moving target + bit1 resume after reposition + bit2 fast final descent
PLND_STRICT    = 2      # hold position instead of landing blind if the target is lost
PLND_RET_MAX   = 0      # retries go to a stale (static) position — pointless for a moving pad
LAND_SPEED     = 40     # cm/s (from 25) — shrink the blind window; see table above
WPNAV_SPEED    = 500    # cm/s — verify current value first; ≥ 2× pad speed
WPNAV_ACCEL    = 200    # cm/s² — catch-up authority; 150 is probably fine too
PLND_LAG       = 0.05   # keep; Pi latency measured 44 ms median
LOG_DISARMED   = 1      # temporarily, for the bench test below (PL log message)
```

Keep `PLND_EST_TYPE=1`, `PLND_ALT_MIN=0.2`, `PLND_XY_DIST_MAX=5`. Static-pad landings must still
work with these settings (bit 0 estimates ~0 velocity for a static pad) — fly one regression
landing before any moving test. Add the values to the param backup once flown.

### Pi (`droneweb/precland.py`) — implemented 2026-09-16, not yet flight-tested

A **Target mode** card in the precland tab: *Moving pad* checkbox (off = static pad = the
flight-proven behaviour, byte-for-byte the same sending logic) and a *Coast after marker lost*
slider (0–8 s, default 3). The card turns amber while moving mode is on.

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

CSV gains `moving, gs, dn, de, dd, pn, pe, pd, pvn, pve` (mode flag, FC groundspeed, drone
NED, pad NED per sighting / prediction, fitted pad velocity) and `source = coast`.

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

1. **Bench, FC powered, not armed**, `LOG_DISARMED=1`: slide the pad under the drone at ~1 m/s.
   Dataflash `PL.vX/vY` should follow the pad velocity within a couple of seconds, `PL.TAcq=1`
   throughout, and the web card's pad speed/heading should agree with it and with the tape
   measure/stopwatch. Then cover the marker: target shows *coast*, the yellow circle keeps
   moving with the (hidden) pad, sending stops after the coast time. Validates
   `PLND_OPTIONS`, EKF velocity, the marker-range `distance` and the Pi tracker without
   flying. Also confirms the Pi's loop rate is unaffected (`loop_hz`).
2. **Static pad, new params**: one LAND regression. Check the final-metre descent rate (CSV `agl`
   vs `t`) — this is the blind-window number.
3. **Slow pad**: pad on a cart pulled by a ≥ 10 m rope at 0.5 m/s. First Precision-Loiter at
   3–4 m (tracking only, no descent), then LAND from 3 m. Analyse: `PL.vX/vY` vs actual speed,
   CSV `ox/oy` during descent, last-detection AGL, touchdown offset on the pad.
4. **1 m/s, then 2 m/s** (rover or car). Then from 8 m.
5. In the CSVs: last real sighting AGL, coast duration, `pn/pe` continuity across the
   real→coast transition, and touchdown offset on the pad. Tune `coast_s`/`LAND_SPEED` from that.

Abort at every step: mode switch to LOITER. Keep the rover's path clear of people; the drone
descends along it.

## Open points

- Exact FC firmware version — function names in this note are from master; semantics are the
  same since Copter 4.2 but confirm `PLND_OPTIONS` exists in the installed version.
- Current `WPNAV_SPEED`/`WPNAV_ACCEL`/`LAND_SPEED` on the FC (backup is from July, before PLND).
- Does the 0.35–2 m slow-down rule actually engage today? The static-pad CSVs show ~33 cm/s
  average in the last 3 m, so maybe not on this firmware — check `agl` vs `t` in the last 2 m.
- Lidar mounting position vs pad: at touchdown the TF-Luna must see the pad, not the ground
  beside it (land detector's rangefinder < 2 m check is fine either way, but AGL logging isn't).
- Wind: a 5 m/s headwind on the rover's path means the drone flies pitched — more time with the
  marker near the top image edge (the 4 m dip in flight test #7).
