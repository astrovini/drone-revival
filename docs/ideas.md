# Ideas backlog

A fast-capture inbox for raw, speculative, or cross-cutting ideas — so nothing gets lost.
This is **not** a task list. The committed, ordered work lives in each area doc's
**Next steps**. Lifecycle:

```
capture here (raw)  →  explore  →  graduate into an area's Next steps  →  strike from here
                                 ↘  or park / reject (keep the reason)
```

**Format:** one bullet per idea, tagged with the area(s) it touches and a status.
Tags: `[system] [toolchain] [sensors] [vision] [motors] [control] [cross]`
Status: `raw` · `exploring` · `promoted→<doc>` · `parked` · `rejected`

---

## Inbox (unsorted / raw)
- `[cross] raw` — **Mac web dashboard**: one page showing live sensor readouts + video feed +
  (later) control state, instead of separate terminals. Nice once fusion exists.
- `[sensors] raw` — Build a **shareable characterization dataset** of this drone's IMU
  (noise, drift, temperature effects) — useful for tuning and as a public reference.
- `[control] raw` — **Simulator first**: model the drone so the controller/auto-tuner can be
  developed and crash safely in sim before touching hardware (sim-to-real).
- `[system] raw` — **Serial console solder** (3.3 V FTDI on the debug pads): a safety net that
  keeps a shell even if WiFi breaks. Only worth it before/if we ever flash a custom kernel.

## Exploring
- _(none yet)_

## Parked (revisit later)
- `[control] parked` — **RL / neural-net stabilization**: real but overkill for a stable
  hover; needs many crashes or a good sim. A "because it's cool" project, not the main path.
- `[cross] parked` — **Drone-as-parts / ground-robot brain**: if flight proves not worth the
  hassle, the mainboard + IMU + cameras make a great rover brain. Fallback direction.

## Promoted (now live as Next steps elsewhere)
- `[control] promoted→control.md` — uBlox GPS for outdoor waypoint routes.
- `[control] promoted→control.md` — CMA-ES/Bayesian offline gain auto-tune on a single-axis rig.
- `[control] promoted→radio.md` — Fly it with a real radio (TBS Tango 2): joystick→AT bridge (Path A) now; onboard-RX acro (Path C) later.
- `[vision] promoted→vision.md` — optical-flow experiment on the bottom camera.
- `[cross] promoted→vision.md+control.md` — vision-guided flight (line-follow, hover-over-marker).

## Rejected (with reason — so we don't re-litigate)
- `[motors] rejected` — Reusing the factory FVT motor script directly: it's a PC-side tester
  tool that *reflashes* the BLCs, not a drone shell command. See docs/motors.md Findings.
- `[cross] rejected` — Bolting on an external Arduino/STM32 for control: throws away the
  onboard Linux computer + cameras for no benefit. The whole project premise is reusing it.
