# Parrot AR.Drone 2.0 — Revival Project Log

A working reference for bringing an old AR.Drone 2.0 back to life: tapping into the
stock electronics, verifying every subsystem, and building toward custom control
(PID / auto-level / routes). This log captures what was done, what was found, the
exact commands that worked, and the gotchas hit along the way.

---

## 0. The core insight

The AR.Drone 2.0 is **not just a flight board — it's a full embedded Linux computer**
(ARM Cortex-A8 / OMAP) that you log into over WiFi with a **root shell, no password**.
That means the smart revival path is to **reuse the stock electronics** (motors, sensors,
cameras, IMU) and either retune the existing flight software or replace it with your own
control program — *not* bolt on an Arduino. Almost nothing needs replacing except the
dead battery.

Two boards:
- **Mainboard** — the ARM/Linux computer. Runs WiFi, both cameras, video encoder, and
  the closed-source flight app `program.elf`.
- **Navboard** — carries the sensors (gyros, accelerometer, magnetometer, barometer,
  ultrasonic). A PIC microcontroller reads them and streams data to the mainboard over a
  serial UART.

---

## 1. This specific drone (confirmed values)

| Item | Value |
|---|---|
| Firmware | **2.4.8** (last official Parrot release — already maxed) |
| Board | `mykonos2` (AR.Drone 2.0 mainboard) |
| Kernel | Linux 2.6.32, armv7l, BusyBox v1.14.0 |
| Serial | PS721801BJ5C067807 |
| WiFi SSID | `ardrone2_067807` |
| Motors | 4× brushless inrunner, motor sw 1.43 / hw 6.0 |
| Total flying time logged | 19630 |
| Low-battery cutoff (`vbat_min`) | 9000 mV (9.0 V) |

All sensors, both cameras, WiFi, the video encoder, and the sensor/motor UARTs
enumerated cleanly. Nothing came up dead.

---

## 2. Connecting (macOS)

The shell is reached over **WiFi telnet**, not a USB cable.

```bash
# 1. Join the drone's WiFi (ardrone2_xxxxx, open network)
# 2. Confirm it's reachable
ping 192.168.1.1
# 3. Open a shell (macOS dropped telnet — install or use nc)
telnet 192.168.1.1            # brew install telnet, OR:
nc 192.168.1.1 23
```

**macOS gotchas:**
- `telnet` is not bundled on modern macOS → `brew install telnet` (or use `nc`).
- `ftp` is also gone → use `curl ftp://...` for file transfer.
- Don't be joined to another `192.168.1.x` LAN at the same time (IP conflict / "No route to host").
- If the drone was ever paired to a phone, telnet may be blocked → press the reset
  button in the battery tray to unpair.

**Key ports:**
| Port | Proto | Purpose |
|---|---|---|
| 23 | TCP | telnet shell (root) |
| 5551 | TCP | FTP (firmware upload, file transfer) |
| 5554 | UDP | navdata (telemetry out) |
| 5555 | TCP | video stream (PaVE-framed H.264) |
| 5556 | UDP | AT commands (control / config in) |
| 5559 | TCP | control channel (config read / ACK) |

---

## 3. Firmware / system facts learned

- The stock flight app `program.elf` owns the navboard, motors, and cameras while running.
- It is guarded by a **respawner** (`program.elf.respawner.sh`): killing `program.elf`
  alone is useless — the respawner relaunches it. Kill the respawner **first**.
- `/data` is the writable partition (~48 MB free). Custom binaries go in `/data/video/`.
  The root filesystem is mostly read-only.
- **No `gcc` and no `gpio` tool on board** → custom programs must be cross-compiled on
  the Mac and FTP'd over.
- Useful tools that ARE on board: `i2c_cmd`, `devmem2`, `dd`, `hexdump`, `strings`,
  `vi`, `strace`, `gdbserver`, `nc`, `awk`.
- `/factory/FVT*_scripts.zip` = Parrot's **factory test scripts** (likely contain the
  safe motor-spin and sensor-check routines used on the production line).

### What you can / can't read
- **Readable (plain text):** all shell scripts, the boot sequence, `/data/config.ini`
  (live tuning), the factory scripts, and the **GPL kernel source** (drivers for the
  navboard UART, cameras, GPIO — published by Parrot).
- **Not readable as source:** `program.elf` itself — the closed flight app (attitude
  estimator, PID loop, motor mixing). You can `strings`/disassemble it, but you won't get
  clean C back. This is *why* the community route is "replace it / run Paparazzi" rather
  than "edit Parrot's code."

---

## 4. Sensors — VERIFIED LIVE

Free the navboard, then read it directly. Everything below is reversible (a battery
unplug/replug restores stock firmware). **`/dev/ttyO1` is the navboard; `/dev/ttyO0` is
the motor bus.**

Live decoded readout (writes a script to /tmp and streams ACC + GYRO until Ctrl-C):

```bash
# kill respawner FIRST, then program.elf, then read /dev/ttyO1
kill -9 $(ps | grep respawner | grep -v grep | awk '{print $1}')
killall -9 program.elf
stty -F /dev/ttyO1 raw -echo
echo -en "\001" > /dev/ttyO1        # start acquisition
dd if=/dev/ttyO1 bs=4096 count=40 of=/dev/null 2>/dev/null   # flush stale frames
cat /dev/ttyO1 | hexdump -v -e '1/1 "%d\n"' | awk '
function s16(lo,hi){v=lo+hi*256; if(v>=32768)v-=65536; return v}
{ b[n++]=$1
  while (n>=58) {
    if (b[0]==58 && b[1]==0) {
      if (++c % 20 == 0)
        printf "ACC %6d %6d %6d  GYRO %6d %6d %6d\n",
          s16(b[4],b[5]),s16(b[6],b[7]),s16(b[8],b[9]),
          s16(b[10],b[11]),s16(b[12],b[13]),s16(b[14],b[15])
      for(i=58;i<n;i++) b[i-58]=b[i]; n-=58
    } else { for(i=1;i<n;i++) b[i-1]=b[i]; n-- }
  } }'
```

**Navboard frame layout** (each frame starts with `0x3A 0x00`):
- bytes 2–3: sequence counter (increments — proof the stream is live, ~200 Hz)
- +0x04 / +0x06 / +0x08: accelerometer X / Y / Z (16-bit)
- +0x0A / +0x0C / +0x0E: gyro X / Y / Z (16-bit)

**Results (confirmed by physically tilting/twisting the drone):**
- **Accelerometer** — all 3 axes work; correctly senses gravity (~530-count offset on
  whichever axis points down). At rest flat: ≈ `2052 / 2012 / 2564`.
- **Gyros** — all 3 axes work; spike during rotation, snap back to baseline when still,
  **no drift**. Tilting roll/pitch lit up gyro X/Y to ±2000; yaw (Z) stayed quiet =
  correct axis isolation.
- At-rest gyro readings sit at ≈ `0 / 35 / 48` — those nonzero values are the **bias
  offsets**, which match the calibration in config.ini exactly (see §6).

**Conclusion:** full 9-DOF sensor foundation for attitude estimation is alive, calibrated,
and healthy. Nothing on the sensing side blocks the project.

---

## 5. Cameras — BOTH VERIFIED

Camera chain (all confirmed up via `dmesg` modules `ov7670 soc1040 omap3_isp ar6000`
and a live decode):
- Front: 720p, streamed downscaled to 640×360 H.264.
- Bottom: QVGA 320×240, high frame rate — the optical-flow camera.
- Both run through the OMAP ISP → DSP H.264 encoder → network.

### Zero-install proof the stream is live (Mac)
```bash
nc 192.168.1.1 5555 | head -c 2000 | xxd | head -40
```
Look for `PaVE` (`50 61 56 45`) at the start = Parrot Video Encapsulation header.
Decodes: codec byte = H.264; bytes 8–11 = frame W/H little-endian; `00 00 00 01`
markers = H.264 NAL start codes (`67`=SPS, `68`=PPS, `65`=IDR keyframe).

### Watch it (Mac) — ffplay handles the PaVE framing best
```bash
brew install ffmpeg
ffplay -fflags nobuffer -flags low_delay tcp://192.168.1.1:5555
```
Harmless warnings (`Failed to parse header of NALU type 0`, `sps_id 32 out of range`)
= ffplay stepping over the PaVE headers. Picture still decodes fine.

### Switch to the bottom camera (Mac)
The config key needs its **section prefix** (`video:video_channel`, NOT just
`video_channel`) and is unreliable as a single UDP shot — hammer it with a rising
sequence number (seq starting at 1 also resets the drone's counter):

```bash
printf '\x01\x00\x00\x00' | nc -u -w0 192.168.1.1 5554   # nudge navdata alive
for i in $(seq 1 15); do
  printf "AT*CONFIG=$i,\"video:video_channel\",\"1\"\r" | nc -u -w0 192.168.1.1 5556
  sleep 0.05
done
# then restart ffplay
```
Channel values: `0`=front, `1`=pure bottom, `2`/`3`=picture-in-picture combos.
(If `1` looks odd, try `3` — bottom-dominant PiP.) **Confirmed working: bottom camera shows the floor.**

---

## 6. Tuning config (`/data/config.ini`) — the adjustable brain

This file is readable plain text and holds the live tuning. Highlights:

**Mode (likely culprit for "twitchy/junky" feel):**
```
outdoor              = TRUE          # <-- was flying in OUTDOOR mode
flight_without_shell = TRUE
indoor_euler_angle_max  = 0.209 rad (~12°)   outdoor_euler_angle_max = 0.349 rad (~20°)
indoor_control_yaw      = 1.74 rad/s          outdoor_control_yaw     = 3.49 rad/s (2x)
```
Flying outdoor-mode indoors = darty, nervous, hard to control. Flipping to indoor mode
is a **free, one-setting test** that may be the single biggest "feels better" change.

**Calibration (all populated & sane):**
```
gyros_offset = { 35.6  25.0  29.6 }   # matches the live at-rest gyro readings
accs_offset  = { ... }                # + 9-value 3x3 accs_gains correction matrix
magneto_offset / magneto_radius       # magnetometer calibrated too
```

**Other knobs:** `euler_angle_max` (max tilt), `control_vz_max` (climb rate),
`control_yaw` (turn rate), `vbat_min = 9000` (low-batt cutoff), `altitude_max/min`.

**Catch:** `program.elf` owns the in-memory config and rewrites `config.ini`, so editing
the file on disk doesn't reliably stick. The proper way to change settings live is the
**AT command channel** (`AT*CONFIG` on UDP 5556, with the `section:key` format).

---

## 7. Power

- Stock pack: **3S LiPo, 11.1 V nominal** (9.6 V empty → 12.6 V full charge).
- **Set a bench supply to 11.1 V** (12 V also fine — it's just "near-full"). Never exceed
  12.6 V; never drop below ~9.5 V (the `vbat_min` = 9.0 V cutoff will trip).
- **Current is the real limit:**
  - Logic / sensors / cameras / config: **< 1 A** → a 12 V 2 A supply runs all of it.
  - **Motors: ~8–10 A** → needs a real battery or a ≥10 A supply, or it browns out and
    reboots the instant a motor spins.
- Current rig: regulated 12 V 2 A barrel adapter, center-positive, measured 12.38 V
  (good, in range). **Fine for everything except spinning motors.**
- **Polarity = the one irreversible mistake.** Reversed = dead board. Match the supply's
  `+` to the drone's `+` pin (the one the old battery's red wire goes to); verify with a
  multimeter before connecting.

---

## 8. Why stabilization was poor (3 separate issues)

The stock drone **does** have a PID and attitude stabilization. The "never hovered well"
problem is really three different knobs:

1. **Leveling felt off** → mostly tuning/mechanical: stale calibration, worn props/motors,
   outdoor-mode aggressiveness, PID gains tuned for a healthy drone. *(Cheapest wins here.)*
2. **Drifts around the room (indoors)** → fundamentally limited by **optical-flow position
   sensing** (downward cam + sonar), which is fragile on plain/shiny/dim floors. Can't be
   fully fixed by tuning — it's a *sensing* limit.
3. **Can't hold a spot / fly routes (outdoors)** → needs a **GNSS module** (uBlox over USB).
   GPS does ~nothing indoors and nothing for leveling — it's an outdoor *position/route* aid.

So: steady indoor hover = tuning + calibration + mechanical health (GPS irrelevant).
Outdoor waypoint routes = add GNSS.

---

## 9. Status & next steps

**Confirmed working:** root shell, firmware, all 9-DOF sensors (live), both cameras (live),
video pipeline, WiFi, full readable tuning config, stable logic-level power.

**Open milestones:**
- [ ] Flip `outdoor` → indoor mode via AT command and feel the difference (free, no motors).
- [ ] Stage & study the `/factory/FVT*` scripts to learn Parrot's safe motor-test routine.
- [ ] Get a real battery or ≥10 A supply → spin motors (`/dev/ttyO0`) safely (props OFF;
      note the controller cuts out after ~3 s with no blade load).
- [ ] Set up the ARM cross-compiler on the Mac → build proper sensor/motor binaries.
- [ ] Decide the control path:
  - **DIY PID:** write own control loop on-board (read `/dev/ttyO1`, drive `/dev/ttyO0`).
  - **Paparazzi UAV:** upload a full open autopilot onto the existing ARM chip (reverts on
    battery cycle); add uBlox GPS for outdoor waypoint routes.

**Reset to stock anytime:** unplug and replug the battery. Nothing done so far is permanent.

---

## 10. Reference URLs

Mostly community/hobbyist sources (blogs, GitHub, wikis) — starting points, not gospel;
details vary by firmware version and some links are old. Most trustworthy for the road
ahead: the **Paparazzi wiki** and the **official Parrot SDK Developer Guide PDF**.

**Architecture / teardown / hackability**
- Chip Overclock teardown (telnet, BusyBox, internals): https://coverclock.blogspot.com/2011/02/deconstructing-ardrone-part-2.html
- KAPEJOD AR.Drone notes (navboard UART, program.elf): http://www.kapejod.org/en/category/ardrone/
- Drones Personalizados board/GPIO reference (mykonos2): https://dronespersonalizados.blogspot.com/p/ardrone.html
- Drone-hacking write-up (open WiFi, root, pairing bypass): https://github.com/markszabo/drone-hacking/blob/master/README.md

**Motors / navboard low-level**
- Perquin — motor controller protocol (9-bit / 200 Hz frames): https://blog.perquin.com/blog/ardrone-motor-controller/
- Custom firmware w/ working navboard/motorboard/attitude/vbat demos: https://github.com/ardrone/ardrone
- Paparazzi — reading the navboard directly (/dev/ttyO1, frame format): https://wiki.paparazziuav.org/wiki/AR_Drone_2/NAV_board

**Custom firmware / autopilot path**
- Paparazzi AR.Drone 2 main page (replace brain, GPS, reverts on battery cycle): https://wiki.paparazziuav.org/wiki/AR_Drone_2

**AT commands / camera switching / control protocol**
- Paparazzi AT command reference: https://wiki.paparazziuav.org/wiki/AR_Drone_2/AT_Commands
- node-ar-drone (confirmed `video:video_channel` camera switch): https://github.com/felixge/node-ar-drone
- JPCHanson low-level protocol/ports wiki: https://github.com/JPCHanson-Academic/ARdrone/wiki/Parrot-AR-Drone-2.0
- Instructables AT-command / navdata bootstrap details: https://www.instructables.com/ArDrone-20-Quadcopter-Control-Unit-on-MPU6050-and-/
- Parrot AR.Drone SDK Developer Guide (PDF, official): https://homes.cs.washington.edu/~shwetak/classes/ee472/notes/ARDrone_SDK_1_6_Developer_Guide.pdf

**Firmware versions / flashing / downgrade**
- Legacy firmware archive + downgrade procedure: https://github.com/rascafr/parrot-ardrone-2.0-firmwares

**General context**
- "What it can still do" overview (5555, PaVE, PS-Drone/python-ardrone): https://medium.com/@soeren_eckhardt/drone-hacking-for-beginners-what-the-ar-drone-2-0-can-still-do-in-2025-5fc5189ab49a
