# Drone Revival — Parrot AR.Drone 2.0

Reviving an old AR.Drone 2.0 by treating it as the **embedded ARM Linux computer it is**:
`telnet` in over WiFi for a root shell (no password), reuse the stock electronics (9-DOF IMU,
sonar, both cameras, 4 brushless motors), and replace/retune the flight software on the
hardware that's already there — no Arduino. Endgame: auto-level → steady hover → routes.
Everything is reversible: a battery unplug/replug restores stock firmware.

---

> ## ⛔ CLAUDE CANNOT REACH THE DRONE. DO NOT TRY.
> The drone is a WiFi access point with **no internet**. To touch it, the user's Mac must
> **leave this network and join `ardrone2_067807`** — which **disconnects this Claude session**
> (I run in the cloud; no internet = no me). **I can never be connected to the drone and to the
> user at the same time.**
>
> So **never** run `ping 192.168.1.1`, `nc/telnet 192.168.1.1`, `airport -I`, or any command
> that probes the drone or WiFi — it will always fail and it wastes the user's time (this has
> happened every session). Instead: **write the code/commands, hand the user one batched block
> to run while they're on the drone WiFi and I'm offline, and work from what they paste back.**

---

## Hardware quick facts (this specific drone)

| Item | Value |
|---|---|
| Firmware | **2.4.8** (last official Parrot release) |
| Board / kernel | `mykonos2`, Linux 2.6.32 armv7l, BusyBox v1.14.0 |
| WiFi SSID / IP | `ardrone2_067807` / `192.168.1.1` |
| IMU | 3-axis gyro + 3-axis accel + magnetometer + barometer (navboard) |
| Sonar | downward ultrasonic (low-altitude height) |
| Cameras | front 720p (→640×360 H.264) + bottom QVGA 320×240 (optical flow) |
| Motors | 4× brushless, each an independent flashable "BLC" controller |
| Battery | 3S LiPo, 11.1 V nom, 1000 mAh, **10C ≈ 10 A** ceiling (healthy pack reads ~12.3 V) |

**Key device files / ports:**
- `/dev/ttyO1` = navboard (sensors) · `/dev/ttyO0` = motor bus
- Ports: `23` telnet · `5551` FTP · `5554` navdata (UDP) · `5555` video (TCP) · `5556` AT cmds (UDP) · `5559` control
- Writable partition: `/data` (custom binaries go in `/data/video/`); rootfs is mostly read-only.

---

## Ground rules (read before touching hardware)

1. **Props OFF for any motor work.** Bench-test propless and secured. A propless motor
   self-cuts after ~3 s (no-load detection) — a natural safety net.
2. **Power gate:** logic/sensors/cameras draw <1 A (any supply). **Motors need ~8–10 A**
   → a healthy battery or ≥10 A supply, or it browns out and reboots mid-spin.
3. **Polarity is the one irreversible mistake.** Reversed = dead board. Verify `+`/`−`
   with a multimeter against the old battery's red wire before connecting power.
4. **Kill the respawner FIRST.** `program.elf` is guarded by `program.elf.respawner.sh`;
   killing `program.elf` alone is useless — it relaunches. Respawner first, then the app.
5. **Reset to stock anytime:** unplug/replug the battery. Nothing done so far is permanent.
6. **Back up before flashing.** Only *flashing* firmware can lose the stock image; *running*
   your own binary (kill `program.elf`, run yours) never touches flash. See `docs/system.md`.

---

## Repo layout & where things go

| Path | Purpose |
|---|---|
| `CLAUDE.md` | This file — overview, facts, ground rules, doc index. |
| `docs/` | One markdown per project area, each with its own **Next steps**. |
| `scripts/{sensors,vision,motors,control}/` | Actual code/binaries, organized by area. |
| `data/` | Captured logs (sensor baselines, motor disturbance maps). |
| `backups/` | Stock-firmware escape kit (`program.elf`, `config.ini`, `/factory`). |
| `ardrone2_revival_log.md` | Historical master log / narrative archive. |

### Documentation index

- **[docs/system.md](docs/system.md)** — Foundation: connect, shell, ports, FTP, config &
  AT commands, power, backup/recovery. *Everything depends on this.*
- **[docs/toolchain.md](docs/toolchain.md)** — ARM cross-compiler setup. *Gates motors & control.*
- **[docs/sensors.md](docs/sensors.md)** — IMU/navboard: read live, log, characterize noise, fuse to attitude.
- **[docs/vision.md](docs/vision.md)** — Cameras: stream, switch front/bottom, computer vision, optical flow.
- **[docs/motors.md](docs/motors.md)** — Motor bus protocol, motorboard driver, safe single-motor test.
- **[docs/control.md](docs/control.md)** — PID, auto-level, hover, routes, and auto-tuning/learning.
- **[docs/radio.md](docs/radio.md)** — Manual flight with a real RC transmitter (TBS Tango 2):
  joystick→AT bridge now, onboard-RX acro later.
- **[docs/ideas.md](docs/ideas.md)** — Idea backlog (fast capture). *Not* a task list — see below.

### Ideas vs. next steps

- **New idea** → **[docs/ideas.md](docs/ideas.md)** (raw inbox, tagged by area; capture first, judge later).
- **Committed work** → the relevant area doc's **Next steps**. When an idea matures, graduate it
  into Next steps and mark it `promoted→<doc>` in ideas.md (park/reject the rest, with the reason).

---

## You are here (status)

**Confirmed working:** root shell, firmware, all 9-DOF sensors (live-verified), both cameras
(live), video pipeline, WiFi, full readable tuning config, healthy battery installed, **ARM
cross-toolchain (static armv7 musl, verified on-drone 2026-07-03 — see `docs/toolchain.md`)**.

**Next up:** characterize the motors now that we can **spin them under our own control** (props-off
constant-PWM, done 2026-07-03 — see below). Fold the winning config into `scripts/motors/motorspin.c`,
then map each motor 1–4 (min spin PWM, direction, slot→position) and log sensors during a spin. Standalone
wins needing no drone code: flip `outdoor`→indoor mode (`docs/system.md`) and capture the sensor idle
baseline (`docs/sensors.md`).
Also live (parallel track): **manual radio control** (`docs/radio.md`) — Path A **flies on the real
drone** via Tango sticks **or the laptop keyboard** (`tango_fly --keyboard`), props-off verified. **navdata
telemetry SOLVED & live-confirmed:** `wake_navdata()` registers the multiconfig session (required, or CONFIG
is ACKed-but-dropped), ACK-gates `navdata_demo=TRUE` to leave bootstrap, adds motor PWM (tag 9) via
`navdata_options=513`; folded into `tango_fly --telemetry` (one AT stream). **Motor/IMU diagnosis DONE
(2026-07-02):** motors + IMU healthy — the "compensating motors" was props-off stabiliser **integral
windup**, plus a **magnetometer-vs-motor-current yaw flip** (~180° at ~160+ PWM; see `docs/sensors.md`).
**Toolchain DONE (2026-07-03):** static armv7 musl cross-GCC (`messense` brew tap) verified end-to-end on
the drone. **MOTOR SPIN DONE (2026-07-03):** our own tool spun a motor props-off at constant PWM on
`/dev/ttyO0` (GPIO via `/dev/mem`; `ardrone/ardrone` is actually 1.0 code, Paparazzi's `boards/ardrone` is
the 2.0 ref). **Key fix:** select lines (GPIO 171–174) must be **hi-Z during the multicast PWM run**, NOT
driven low as Paparazzi does — Hugo's 1.0 hi-Z method was right. Full write-up in `docs/motors.md`.

> When a session makes progress, update the **Status** section of the relevant `docs/*.md`
> and this block, so the next session starts oriented.

## Conventions

- Mac is the dev box; the drone is the target. Cross-compile on Mac, FTP binaries to `/data/video/`.
- Connect (**the user does this, not Claude — see the ⛔ callout at top**): the user joins the
  drone's WiFi → `telnet 192.168.1.1` (or `nc 192.168.1.1 23`). Claude only prepares the commands.
- Name captured logs `data/<area>/<what>_<conditions>.csv` (e.g. `data/sensors/idle_flat_30s.csv`).
- Sources are community/hobbyist — see §10 of `ardrone2_revival_log.md` for URLs.
