# Control (the synthesis: stabilization → hover → routes)

Where sensors + motors come together. Reads the fused attitude from [sensors.md](sensors.md),
drives the motors from [motors.md](motors.md), and closes the loop. This is the endgame.

## Status
- ⬜ Not started — depends on solid sensor fusion AND a working motor driver first.
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

## Next steps (staged, safest first)
- [ ] **Free win, no code:** flip `outdoor`→indoor (AT command, [system.md](system.md)) and
      re-fly to feel the baseline improvement.
- [ ] **Sensor fusion solid** (complementary → Kalman) before any controller. → [sensors.md](sensors.md)
- [ ] **Single-axis test rig** — mount the drone on a rod/gimbal so it can only rotate about
      one axis. Tune safely (oscillate, overshoot, recover) without it flying away.
- [ ] **Attitude-hold PID on one axis** → extend to full **auto-level**.
- [ ] **Altitude hold** using sonar/baro.
- [ ] **Free hover.**
- [ ] **Routes** — outdoor waypoints need GNSS; for repeated paths, Iterative Learning Control
      (ILC) improves tracking every pass.

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
