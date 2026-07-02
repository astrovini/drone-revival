# Radio control (manual flight with a real transmitter)

Fly the drone with a physical RC transmitter (**TBS Tango 2**) instead of the phone app
or a hard-coded program. Two layers: a Mac-side **joystick→AT bridge** over the *stock*
firmware (Path A — happening now), and eventually an **onboard receiver** feeding our own
rate controller for true acro (Path C). Graduated from [ideas.md](ideas.md).

## Status
- ✅ Tango 2 confirmed as a USB-HID joystick on the Mac (EdgeTX/FreedomTX joystick mode).
- ✅ Stick + switch map characterized and baked into `scripts/control/tango_read.py`.
- ✅ Safe rest states verified — A & D rest at −1 (disarmed / not-triggered) and latch (stay put).
- ✅ Mac→drone **AT send loop** built + verified (`scripts/control/at_link.py` + `fake_drone.py`).
- ✅ Joystick→AT **bridge** (`scripts/control/tango_fly.py`): built, sim-verified, and **flown on the
  real drone (props off)** — arm / takeoff / land / emergency all respond and the drone obeys the sticks.
  **Path A fundamentally works.**
- ✅ Command logging: `tango_fly --log` → `data/radio/flight_*.csv` (raw sticks + exactly what was sent;
  confirmed the right stick *was* read — 384 move frames carried roll/pitch input).
- ✅ **navdata bootstrap — BEATEN & LIVE-CONFIRMED (2026-07).** `wake_navdata()` registers the multiconfig
  session then sends `navdata_demo=TRUE` + ACK; on the real drone it printed `bootstrap cleared — attitude
  live (tags [0,16])` and streamed roll/pitch/yaw. (Earlier failure was a rewrite that wrongly dropped the
  session registration → CONFIG ACKed-but-dropped → stuck; restored.)
- ✅ **Motor PWM (tag 9) — LIVE-CONFIRMED via `navdata_options=513`.** Additive route wins: staying in demo
  mode + `navdata_options=513` gives `tags [0, 9, 16]`, and PWM decodes (`M[0 0 0 0]` while landed — motors
  off, as expected). `navdata_demo=FALSE` fallback wasn't needed.
- ✅ **Telemetry folded into `tango_fly --telemetry`** — reads attitude + motor PWM over the **same** AT
  link (one sequence stream) and logs it beside the sticks. Shares the now-proven `wake_navdata()`;
  offline-verified (mock-drone handshake + drain/parse/sim). ⬜ *props-off takeoff run to log non-zero PWM.*
- ⬜ Confirm stick **directions** on a real hover (flip `REVERSE_*` in `tango_fly.py` if wrong).
- ⬜ Path C (onboard CRSF RX → our rate controller → acro) — depends on [control.md](control.md).

## How it works — the drone has no RC receiver
A normal FPV quad chains TX → receiver → flight controller → motors. The AR.Drone has
**none of that**: it takes commands only via **AT/WiFi** (stock `program.elf`) or our own
motor-bus binary. So the radio can't talk to it directly — it becomes an **input device
feeding a translator**. Three paths, easiest first:

- **Path A (now):** Tango 2 in USB-HID joystick mode → Mac reads axes → emits `AT*PCMD`/`AT*REF`
  over WiFi to the stock firmware. Zero extra hardware; tethered (Mac is the WiFi link).
- **Path B (optional):** same, but via a wireless trainer dongle so the Tango isn't USB-tethered.
- **Path C (later):** onboard Crossfire/Tracer **Nano RX** wired to a drone UART → parse CRSF →
  feed *our* control loop directly. True standalone RC; only viable once the inner-loop rate
  controller exists ([control.md](control.md)), since CRSF only gives stick positions.

## The control map — TBS Tango 2 (Mode 2, TAER channel order)
Switches enumerate as **axes**, not buttons (EdgeTX joystick mode): 2-pos = −1/+1,
3-pos = −1/0/+1. This table is authoritative; `scripts/control/tango_read.py` encodes it.

| Axis | Source | Function | Notes |
|---|---|---|---|
| 0 | left stick V | **throttle** | |
| 1 | right stick H | **roll** (aileron) | |
| 2 | right stick V | **pitch** (elevator) | |
| 3 | left stick H | **yaw** (rudder) | |
| 4 | switch **A** (2-pos) | **arm** | safe rest = −1 (disarmed) |
| 5 | switch **B** (3-pos) | **takeoff / land** | up = takeoff, center = hold, down = land |
| 6 | switch **C** (3-pos) | **flight mode** | reserved for Path C (indoor / level / acro) |
| 7 | switch **D** (2-pos) | **EMERGENCY cut** | safe rest = −1 (not triggered) |

## "Acro mode" reality
Acro is **not a stock switch** — stock firmware always self-levels. It emerges from Path C's
rate controller with the self-level outer loop turned off. Airframe caveat: heavy foam quad,
thrust-to-weight ~1.5 → manual rate flight is fine, real freestyle/flips are not.

## Telemetry debugging (navdata) — SOLVED & live-confirmed (attitude + motor PWM)
**Goal:** read the drone's attitude estimate + per-motor PWM to answer *"is a motor misbehaving, or is
the stabiliser fighting a phantom tilt?"* — the "motors sounded like they were compensating while it sat
still" symptom from the first real flight.

**Root cause (proven with `--probe`, three live runs 2026-07):** two requirements, both client-side.
1. **The multiconfig session must be registered first.** On 2.4.x, an `AT*CONFIG` that names an
   *unregistered* session (via `CONFIG_IDS`) is **ACKed but silently dropped**. `--probe` registers it
   (`custom:session_id/profile_id/application_id`) *before* `navdata_demo`, so its config actually applies
   and bootstrap clears. My first "clean" rewrite **dropped** that registration on a wrong hunch that the
   "burst" was the bug — result: `navdata_demo=TRUE` was ACKed (COMMAND_ACK toggled) but never applied, so
   it sat in bootstrap forever. **The burst was never the problem; the missing session registration was.**
2. **Only `navdata_demo=TRUE` exits bootstrap.** `=FALSE` sent from bootstrap does nothing (the `--full`
   build that pointed straight at `FALSE` sat in bootstrap ~50 s). So break bootstrap with `TRUE`, *then*
   add PWM once the stream is live.

Breakout is unmistakable in `--probe`: packets jump `len 24, tags [65535], …BOOTSTRAP…` →
`len 500, tags [0,16,65535]` with **no** BOOTSTRAP bit and tag 0 (attitude) present.

Also fixed: **PWM is tag 9** (`NAVDATA_PWM_TAG`), not tag 4 (`GYROS_OFFSETS`).

**Fix (live-confirmed on the drone; also offline-tested — `--selftest`, mock-drone tests incl. a
"no-session ⇒ stuck" guard, loopback drain, sim):**
- `wake_navdata()` now mirrors the proven `--probe` sequence: **`drone.init_session()` (register session)**
  → `_gated_config()` re-sends `navdata_demo=TRUE` and ACKs every `COMMAND_ACK` until option blocks appear.
- Phase 2 (only if `--full`/`--telemetry`) adds PWM two ways and prints which worked: **(a)** additive
  `general:navdata_options=513` (tags 0+9) in demo mode — **this is the one that worked live**; **(b)**
  fallback `navdata_demo=FALSE`. If neither yields tag 9, it recovers to attitude-only. Returns
  `'full'`/`'demo'`/`None`.

**⚠ Two AT senders collide.** Both `navdata.py` and `tango_fly.py` send AT commands to :5556, each with its
own sequence counter, and the drone drops any command whose seq isn't higher than the last it saw. Running
them side-by-side knocks navdata back into bootstrap. **So telemetry lives inside `tango_fly --telemetry`**
(one `Drone`, one seq stream) — don't run `navdata.py` at the same time as a flight.

**Confirmed live (2026-07):** `navdata.py --host 192.168.1.1 --full` → `attitude + motor PWM`, streamed
roll/pitch/yaw and `M[0 0 0 0]` (landed). Battery read 43% during that test — charge before a flight run.

**Next hardware run — the real capture (props OFF, one offline session):**
- **`tango_fly.py --host 192.168.1.1 --telemetry --log`**, arm, brief takeoff (self-cuts ~3 s), nudge a
  stick. One CSV then holds sticks + sent PCMD + attitude + motor PWM, so PWM will be **non-zero** during
  the takeoff window. Reconnect to normal WiFi and I'll read `data/radio/flight_*.csv` to diagnose the
  "compensating" motors (attitude near 0 while level? the four PWMs symmetric?).

**`navdata.py` flags:** `--probe` (verbose bit-name diagnostic) · `--full` (full navdata incl. motor PWM) ·
`--log` (CSV → `data/radio/`) · `--selftest` (offline parser check).

## Next steps (staged, safest first)
- [x] Safe rest states verified (A & D rest −1, latch; leave popped out before power-on).
- [x] AT send loop (`at_link.py`) + local tester (`fake_drone.py`).
- [x] Bridge built (`tango_fly.py`) and sim-verified.
- [x] **Flown on the real drone (props off):** arm / takeoff / land / emergency confirmed; sticks logged.
- [x] **Get navdata out of bootstrap** — register multiconfig session + ACK-gated `navdata_demo=TRUE`;
      PWM (tag 9) via additive `navdata_options=513`. **Live-confirmed:** `navdata.py --full` streamed
      attitude + `M[0 0 0 0]` (landed). Folded into `tango_fly --telemetry`. (See *Telemetry debugging*.)
- [ ] **Real capture (props OFF):** `tango_fly --telemetry --log` during a brief takeoff → **one** CSV
      with sticks + PCMD + attitude + **non-zero** motor PWM. **← current step.** Then diagnose the
      "compensating" motors (attitude near 0 when level? motor PWMs symmetric?). Charge the pack first (43%).
- [ ] Confirm stick **directions** on a real hover; flip `REVERSE_*` in `tango_fly.py` if needed.
- [ ] Flip `outdoor`→indoor before the first free hover ([system.md](system.md), [control.md](control.md)).
- [ ] **Path B (optional):** wireless trainer dongle for untethered control.
- [ ] **Path C (later):** onboard CRSF/Tracer Nano RX → our rate controller → acro.

## Gotchas
- **One AT sender at a time.** The drone tracks a single latest command-sequence number and drops
  anything not strictly higher, so two processes talking to :5556 (e.g. `navdata.py` + `tango_fly.py`)
  fight and lose commands. Read telemetry via `tango_fly --telemetry`, not a second process.
- **navdata bootstrap** clears only after you **register the multiconfig session** (`init_session`) *then*
  send `navdata_demo=TRUE` with ACKs — a CONFIG on an unregistered session is ACKed but silently dropped,
  and `=FALSE` from bootstrap never exits. `wake_navdata()` does register-then-TRUE.
- **Motor PWM is tag 9** and is *not* in the demo packet (demo = tags `[0,16]`, attitude only). Add it
  after bootstrap with **`navdata_options=513`** (live-confirmed → tags `[0,9,16]`); `wake_navdata()` does
  this automatically for `--full`/`--telemetry`.
- Switches are **axes**, not buttons — threshold the value (−1/0/+1), don't poll buttons.
- `AT*PCMD` must **stream** (~30 Hz) or the comms watchdog stops the drone; floats are sent as
  the int bit-pattern of the IEEE-754 value (see [system.md](system.md)).
- Path A is **tethered** — the Mac is the WiFi link, so walking range = WiFi range. Define a
  failsafe: sticks-centered / USB or WiFi drop → hover or land, never free-run.
- Props-off, takeoff **self-cuts after ~3 s** (no-load detection) — only a brief window to see
  stick→motor response. Bridge sends a **flat-trim on arm**, so start on a level surface.
- Stock firmware has **no manual arm/idle-throttle**: motors stay off until the auto-takeoff, and
  `gaz` is a climb/descend *rate*, not motor power. Manual arm-and-throttle is Path C (own controller).
