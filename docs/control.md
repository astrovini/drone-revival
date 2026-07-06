# Control (the synthesis: stabilization → hover → routes)

Where sensors + motors come together. Reads the fused attitude from [sensors.md](sensors.md),
drives the motors from [motors.md](motors.md), and closes the loop. This is the endgame.

## Status
- 🟡 **Unblocked and in planning (2026-07-04).** Prereqs now exist: our own motor driver works
  (`scripts/motors/motorspin.c` — props-off constant PWM, all 4) and the navboard gyro/accel is
  decoded + scale-validated (`docs/sensors.md`). What's missing is a **real-time C fast loop** and
  the control law. User goal set: **acro first, then the documented auto-level/hover control.**
- 📌 Stock firmware already *has* a PID + attitude stabilization; the goal is to
  **replace/retune** it with our own, not add one where none existed.

## Why the stock drone flew poorly (3 separate problems)
1. **Leveling felt off** → tuning/mechanical: stale calibration, worn props/motors, and it
   was set to **outdoor mode** (aggressive tilt/yaw) while flown indoors. *Cheapest wins here.*
2. **Drifts around the room (indoors)** → limited by **optical-flow position sensing** (down
   cam + sonar), fragile on plain/shiny/dim floors. A *sensing* limit, not a tuning one.
3. **Can't hold a spot / fly routes (outdoors)** → needs **GNSS** (uBlox over USB). GPS does
   ~nothing indoors and nothing for leveling — it's an outdoor position/route aid.

So: steady indoor hover = tuning + calibration + mechanical health. Outdoor routes = add GNSS.

## Architecture
- **Fast loop on the drone:** sensor fusion + PID reading `/dev/ttyO1`, driving `/dev/ttyO0`.
- **Slow loop on the Mac:** logging, tuning/optimization, parameter updates pushed over WiFi.
- Both halves already exist in primitive form (sensor streaming + the network link).

## Acro-first plan (acro and full control are the same controller)
Both endgame goals are **one onboard fast loop**; they differ only in how many loops are closed:
- **Acro = inner loop only.** Read gyro body rates → **rate PID** against stick setpoints → motor mix.
  No attitude estimate needed. Self-level OFF.
- **Auto-level / hover = inner loop + outer loop(s).** An **angle PID** (from fused attitude) feeds rate
  setpoints into the same inner loop; altitude (sonar/baro) adds a throttle loop. So **building acro
  builds the core of the documented control** — acro-first is the correct order.

Neither can use stock firmware: we `kill program.elf`, then read `/dev/ttyO1` (navboard) and drive
`/dev/ttyO0` (motors) ourselves in one process — exactly like `motorspin`.

### Reference code that's actually useful (verified 2026-07-04)
- **Hugo `ardrone/ardrone` `fly/controlthread.c` + `pid.c` — most reusable; it targets THIS board.**
  Exact X-quad **motor mixer** (verbatim, modulo our slot→corner check):
  `m0 = T +roll −pitch +yaw · m1 = T −roll −pitch −yaw · m2 = T −roll +pitch +yaw · m3 = T +roll +pitch −yaw`,
  plus a tiny PID lib with **D-on-gyro**. It's angle-mode/P-only (Kp roll/pitch 0.5, yaw 1.0, tilt cap
  12°) — flip to acro by feeding **rate error** instead of angle error; the mixer is identical.
- **Paparazzi `firmwares/rotorcraft/stabilization/stabilization_rate.c` — the acro algorithm.** Real rate
  controller: `err = stick_rate_sp − gyro_body_rate`, `cmd = P·err + I·∫err` (windup-clamped, no D). Copy
  this law for our inner loop. Paparazzi also has the 2.0 gyro + actuator mixing in the same tree.
- **Betaflight/Cleanflight — gold standard for acro *feel*/tuning** (RC rates/expo, iterm-relax, D-term
  filtering). Not portable to this hardware and overkill to port, but the reference for flying rate mode
  nicely.
- **Synthesis:** Hugo's mixer (this airframe) + Paparazzi's rate-PI law (acro) + our `navread` gyro +
  our motor driver = the acro controller.

### Safety (non-negotiable — we lose the ~3 s cutout net)
Once our loop holds the motors continuously, the props-off no-load self-cut no longer saves us. Before
anything spins with intent: **single-axis test rig** (drone on a rod, one axis only), **hard abort limits**
(cut motors if rate/tilt exceeds a threshold, or on comms/setpoint loss), and props off until the rig
stage. Airframe caveat ([radio.md](radio.md)): thrust-to-weight ~1.5 → manual **rate** flight only, no flips.

## Next steps (staged, safest first)
- [x] Motor driver — props-off constant PWM, all 4. → [motors.md](motors.md)
- [x] Navboard gyro/accel decoded + scale-validated (raw batch logger). → [sensors.md](sensors.md)
- [x] **`navread.c`** — port `log_idle.sh`'s decode to real-time C, physical units (°/s, g), fresh gyro
      bias measured at startup. (Sensor half of the fast loop; safe, no motors.) **DONE + on-drone
      validated 2026-07-06** — still→0, hand-rotation lights the right axis, accel tilt agrees with the
      rate integral, 0 dropped frames under motion. → [sensors.md](sensors.md). **Fast-loop input ready.**
- [x] **Body-axis sign map from `navread`** (2026-07-06 front/back/right/left×2 test) — the mixer's rate-sign
      reference: **pitch = `gy`/`ax`** (nose-down = −ax/−gy), **roll = `gx`/`ay`** (right = −ay/+gx),
      **yaw = `gz`**. Full table in [sensors.md](sensors.md). Still need slot→corner (below) to close the loop.
- [x] **`navread` frame-validation / lock-on gate** — DONE + on-drone verified 2026-07-06. Startup lock-on
      + seq-continuity + 12-bit sanity + self-heal; a gated `--csv` capture had 0 impossible frames / 0 seq
      discontinuities (was 1/5379). `navread` is now safe to feed a PID. → [sensors.md](sensors.md).
- [~] **Print-only rate-loop bench tool — BUILT (2026-07-06), pending on-drone run.**
      `scripts/control/ratebench.c`: the full inner acro chain (navboard gyro → rate PID → Hugo X-quad
      mixer) with motor drive **disabled — prints the 4 motor commands only**. Reads `/dev/ttyO1` ONLY;
      never opens `/dev/ttyO0`/GPIO/​/dev/mem — as safe as navread, props off. Setpoint=0 ⇒ rate damper:
      still ⇒ all four ≈ throttle; rotate an axis ⇒ opposing motors split (hand-verified logic: +100°/s roll
      ⇒ m0,m3 low / m1,m2 high; pitch & yaw split their own pairs). Confirms the chain + relative signs with
      **zero risk** before any closed-loop spin. Full physical which-corner check needs slot→corner (below).
      Also introduced **`scripts/sensors/navboard.h`** — header-only shared IMU reader (frame protocol +
      factory cal + fresh bias + validation gate) so navread and the control tools share ONE copy; gate
      unit test still passes against it. Has a live in-place gauge + `--csv` capture mode.
      **On-drone verified 2026-07-06 (pitch + roll):** hand-rotation split the correct motor pairs —
      pitch ⇒ {m0,m1} vs {m2,m3}, roll ⇒ {m0,m3} vs {m1,m2}, scaling with rate, returning to throttle at
      rest; gate stayed clean (0 dropped/rejected/relocks over 3426 frames). ⬜ yaw grouping ({m0,m2} vs
      {m1,m3}) still to exercise; physical which-corner/spin-direction check needs slot→corner (below).
- [~] **Motor mixer + geometry map** — slot→corner + CW/CCW. **Open-loop bench tool built (2026-07-04):**
      `scripts/motors/manualmix.c` (onboard: UDP sticks → Hugo mixer → `/dev/ttyO0`; arm-to-idle, hi-Z
      select; **NO stabilization — props-off bench only**, comms-timeout + emergency + throttle-low-arm
      failsafes) + `scripts/control/tango_mix.py` (Mac: Tango/keyboard → UDP :5560). Arms all four to idle,
      then sticks modulate the four motors — also the way to *read off* which motor is which corner.
      ⬜ run props-off, confirm stick→motor response, record slot→corner + spin directions.
- [ ] **Single-axis test rig + hard abort limits** before any intentional spin under closed loop.
- [ ] **Rate PID = ACRO** on the rig — one axis, then all three; tune P then I (Paparazzi rate law).
- [ ] **Free win, no code:** flip `outdoor`→indoor ([system.md](system.md)) and re-fly the stock baseline.
- [ ] **Sensor fusion** (complementary → optional Kalman) for attitude. → [sensors.md](sensors.md)
- [ ] **Angle PID = auto-level** wrapping the rate loop; then **altitude hold** (sonar/baro) = **hover**.
- [ ] **Routes** — outdoor waypoints need GNSS; ILC improves repeated paths each pass.
- [ ] **Command source:** onboard CRSF RX → our loop ([radio.md](radio.md) Path C), or keep tethered
      keyboard/Tango setpoints streamed to the loop.

## Can it auto-tune / "learn"? (spectrum, matched to the problem)
- **Drifts right → corrects:** mostly the **integral term** of a PID (eliminates steady-state
  offset within a flight) + **flat-trim** (`AT*FTRIM` on a flat surface). Try these before any ML.
- **Trim learning:** save the integrator's settled offset as a persistent trim → next flight
  starts pre-corrected. ~20 lines, real cross-flight learning.
- **Offline gain auto-tune (best bang for buck):** CMA-ES / Bayesian optimization over the
  ~6–12 PID gains, learning loop on the Mac, converges in tens of trials. "It tunes itself."
- **Adaptive control (online):** MRAC / L1 / extremum-seeking adjust gains mid-flight; real
  aerospace techniques, run fine on the Cortex-A8.
- **ILC:** perfect for repeated routes — gets better at *your* specific path each pass.
- **RL / neural policy:** possible but overkill for stabilization; needs many crashes/sim-to-real.
  A "because it's cool" project, not the path to a stable hover.

## Gotchas
- Garbage attitude estimate → nothing tunes; it learns to track a lie. Fusion first.
- Learning by flying = crashing by flying → use the single-axis rig + hard abort limits
  (e.g. cut motors if tilt exceeds X°).
- Position hold can't beat the sensing — no PID gain fixes fragile optical flow.
