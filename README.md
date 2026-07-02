# Drone Revival — Parrot AR.Drone 2.0

Reviving an old AR.Drone 2.0 by treating it as the **embedded ARM Linux computer it is**:
`telnet` in over WiFi for a root shell (no password), reuse the stock electronics (9-DOF IMU,
sonar, both cameras, 4 brushless motors), and replace/retune the flight software on the
hardware that's already there — no Arduino. Endgame: auto-level → steady hover → routes.

Everything is reversible: a battery unplug/replug restores stock firmware.

## Hardware

| Item | Value |
|---|---|
| Firmware | 2.4.8 (last official Parrot release) |
| Board / kernel | `mykonos2`, Linux 2.6.32 armv7l, BusyBox v1.14.0 |
| IMU | 3-axis gyro + 3-axis accel + magnetometer + barometer (navboard) |
| Sonar | downward ultrasonic (low-altitude height) |
| Cameras | front 720p (→640×360 H.264) + bottom QVGA 320×240 (optical flow) |
| Motors | 4× brushless, each an independent flashable "BLC" controller |
| Battery | 3S LiPo, 11.1 V nom, 1000 mAh, ~10 A ceiling |

## Repo layout

| Path | Purpose |
|---|---|
| `CLAUDE.md` | Project ground rules and conventions (also doubles as AI pair-programming instructions). |
| `docs/` | One markdown per project area — system, toolchain, sensors, vision, motors, control, radio, ideas — each with its own status and next steps. |
| `scripts/` | Code, organized by area (`sensors/`, `vision/`, `motors/`, `control/`). |
| `data/` | Captured logs (sensor baselines, motor disturbance maps, flight telemetry). |
| `backups/` | Stock-firmware escape kit — factory test scripts, config, recovery notes. |
| `ardrone2_revival_log.md` | Historical narrative archive of the project. |

Start with **[CLAUDE.md](CLAUDE.md)** for the full picture, then **[docs/system.md](docs/system.md)**
for how to connect and the safety ground rules before touching hardware.

## Status

See the **You are here** section in [CLAUDE.md](CLAUDE.md) for the current state of the project.
