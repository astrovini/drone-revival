# Motors (actuation)

The 4 brushless motors, each an independent reprogrammable controller ("BLC") on the motor
UART (`/dev/ttyO0`). **As of 2026-07-03 we can spin a motor under our own control** (props-off,
constant PWM, flight controller out of the loop) — see the Key finding below.

## Status
- ✅ Motor bus (`/dev/ttyO0`) enumerated; UART present.
- ✅ Power unblocked: healthy 3S battery (~10 A ceiling) installed.
- ✅ Factory test scripts read and understood (see Findings) — but **not reusable**.
- ✅ **Motors observed healthy via flight telemetry (2026-07-02).** In props-off takeoffs (`program.elf`
  driving), navdata motor PWM showed all four **identical to within 2 PWM** at constant thrust (`spread 0`
  in the ~1.65 s constant-idle window at 70→72 PWM); the in-flight asymmetry is stabiliser **integral
  windup** (no props = no feedback), not a bad motor. Full analysis in [radio.md](radio.md). This is
  observation only — we haven't commanded a motor ourselves.
- ✅ **MOTOR SPUN UNDER OUR OWN CONTROL (2026-07-03).** Our own tool (`scripts/motors/motortest.c`)
  did a props-off constant-PWM spin of motor 1 on `/dev/ttyO0` with the flight controller fully out of
  the loop — the definitive motor check, achieved. See the **Key finding** below for the one detail that
  made it work (select-line polarity).
- ✅ Cross-compiler done → [toolchain.md](toolchain.md); GPIO via `/dev/mem` (register access) working.
- ⬜ **Next:** fold the winning config into `motorspin` and characterize each motor 1–4 (min spin PWM,
  spin direction, map slot→physical position), then log sensors during a spin. See Next steps.

## Key finding — select-line polarity (2026-07-03, empirically nailed down)

Everything measurable was correct (GPIO writes land, `0xe0` handshake replies `e0 00` on all 4 BLCs,
framing matches the reference) yet **no motor moved** through many attempts. A back-to-back battery
(`motortest.c`, 8 approaches) isolated the cause to **the select-line state during the multicast PWM run**:

| Select lines (GPIO 171–174) during run | Result |
|---|---|
| **driven LOW** (`gpio_clear`) — Paparazzi's "active" | ❌ never spins (tests 1,4,5,6,7,8 all failed) |
| **hi-Z / input** — Hugo's 1.0 "active" | ✅ **spins, LED green** (healthy) — test 2 |
| **driven HIGH** | ✅ spins, LED red — test 3 |

**Conclusion:** for multicast PWM you must **de-select all motors (release to hi-Z)**, *not* drive them
low. **Paparazzi's `actuators.c` is wrong on this for our board; Hugo's original 1.0 hi-Z method is
correct.** (Individual select — driving one line low — is still used only during the per-motor `0xe0`
config handshake.) The green status LED under hi-Z confirms it's the correct/healthy enable; drive-high
spins but flags red. Neither reference driver is fully right on its own — the truth is Paparazzi's
device/GPIO-numbers/framing **plus** Hugo's select-line handling.

**BLC startup ramp (observed 2026-07-03):** on any enable + nonzero PWM the BLC runs its own open-loop
brushless spin-up ramp (audible) before settling to the setpoint — arming is the `0xa0` enable; the ramp
is the ESC's own motor-start. So in a *no-kick* sweep, **every** level 5→90 spun. Consequence: there's no
clean "won't start below X PWM" floor to find by ear — the ESC self-starts across the whole low range
(a healthy sign). The meaningful characterization is steady **speed vs PWM**, which needs an RPM estimate
(IMU-vibration FFT or an external tach), not by-ear spin detection.

## Motor specs (Parrot datasheet — props ON / loaded)
- Brushless, **~14.5–15 W**; hover **~28,000–28,500 RPM (motor)** → **~3,300 RPM (prop)** through an
  **8.625:1** Nylatron gear reduction. Operating range **~10,350–41,400 RPM (motor)**.
- Rare-earth magnets, self-lubricating bronze + micro ball bearings, tempered-steel prop shaft.
- Each motor has its own reflashable 8-bit AVR "BLC" controller on the `/dev/ttyO0` bus (see Findings).
- **Not published:** any PWM→RPM curve, Kv, pole count, or winding data — so we characterize empirically.
  These figures are **props-on (loaded)**; props-off, a given PWM spins *faster* and the ~3 s no-load
  cutout applies, so our bench numbers won't map straight onto them.
- Sources: [Parrot spare-part page](https://www.parrot.com/global/spareparts/drones/parrot-ardrone-20-brushless-motor),
  [AR.Drone 2.0 datasheet](https://manualzz.com/doc/3363829/parrot-ar.drone-2.0-data-sheet).

## How it works
Spinning a motor means writing a **proprietary binary frame protocol** to `/dev/ttyO0`:
an init/enable handshake, then per-motor speed frames (9-bit value per motor, packed,
transmitted ~every 5 ms at 200 Hz). There is **no safe shell one-liner** — a wrong byte can
do nothing or command all four unexpectedly. Use a compiled driver, not hand-crafted bytes.

Architecture (confirmed from factory scripts):
- 4 independently addressable BLC motors, numbered 1–4, each individually flashable.
- **Motor-fault signal on GPIO 176**; reset button on GPIO 87.
- Navboard is the flashable "MYKONOS2."

## Findings: the factory FVT scripts are NOT a shortcut
The scripts live in [`backups/factory/`](../backups/factory/) (`FVT{1,2,3}.zip` + extracted).
`FVT3/FVT3_DRONE/FVT3_DRONE.xml` motor step is:
```
drone_motor_control  FLASH_AND_START_MOTOR  number=N  path=/tmp/BLC.hex   (BLC_v1.31.hex)
drone_motor_control  CHECK_PRESENCE_FIRMWARE  number=N
drone_motor_control  CHECK_MOTORS_COHERENCE
```
Two reasons we can't borrow it:
1. **It's the PC-bench framework, not the drone** — `<call module="Product" ...>` runs in
   Parrot's factory tester software over USB (neighboring steps `SHOW` images and ask a human
   operator to pick the motor supplier). None of it is a drone shell command.
2. **It reflashes the motor controllers** (`FLASH_AND_START_MOTOR` uploads `BLC_v1.31.hex`
   to each motor's AVR) — manufacturing-time provisioning, far more than a bench spin, and
   not something to do casually.

`FVT1_MB.xml` only does a UART **loopback** check (`CHECK_MOTORx_UART_LOOP`), not a spin.

→ **Realistic path:** a small custom motor tool, cross-compiled and FTP'd over. See the
protocol section below for the source references and the key 1.0-vs-2.0 correction.

## AR.Drone 2.0 motor protocol (verified 2026-07-03)

Cross-checked two open-source drivers. **`ardrone/ardrone` (Hugo Perquin) is AR.Drone _1.0_ code**
— confirmed by its `/dev/ttyPA1` device, its `/usr/sbin/gpio` helper, and P6-era GPIO numbers.
**Neither driver is fully right for our board on its own** (see the Key finding above): use Paparazzi's
device path / GPIO numbers / framing **but Hugo's select-line handling** (hi-Z, not drive-low). The
protocol *bytes* are identical between the two; what differs:

| Field | 1.0 (Hugo) | 2.0 (Paparazzi) | **What works on OUR board** |
|---|---|---|---|
| Motor UART | `/dev/ttyPA1` | `/dev/ttyO0` | **`/dev/ttyO0`** @ 115200 |
| Motor-select GPIOs | 68–71 | 171–174 | **171–174** (one per motor) |
| IRQ flip-flop / fault | 106,107 | 175,176 | **175** (reset), **176** (fault in) |
| Per-motor init | `0xe0`→reply `e0 00`, then `m+1` | same | **same** |
| Enable all motors | 5× `0xa0` (multicast) | same | **same** |
| PWM frame (5 bytes, 4×9-bit) | `0x20\|(p0>>4)`, `(p0<<4)\|(p1>>5)`, … | same | **same** |
| Refresh rate | ~200 Hz (every 5 ms) | same | **same** |
| **Select lines during multicast run** | **hi-Z (input)** | **drive low** | **hi-Z ✅** (drive-low never spins) |
| PWM range | `0x000`–`0x1ff`; ~`0xff` to start, then ≥`0x50` | same | **same** |

**GPIO access — RESOLVED on-drone (2026-07-03):** sysfs *is* present (`/sys/class/gpio` has
`export`, `gpiochip160`, and firmware-exported pins like `gpio177/180/181`) — **but exporting the
motor pins 171–176 is refused**: the firmware claims them on the kernel side. So we poke the OMAP3630
GPIO registers directly via **`/dev/mem`** (what `program.elf` itself does). Bank 6 (gpios 160–191)
base `0x49058000`; regs OE `+0x034`, DATAIN `+0x038`, CLEARDATAOUT `+0x090`, SETDATAOUT `+0x094`;
pin bit = `gpio-160`. Bank 6 is already clocked (gpio177 in use), so register access is safe.

Power reference (Hugo, @11 V, all 4 running): PWM 80→1.3 A, 100→1.5 A, 150→2.0 A, 190→2.5 A.

## Next steps (in order)
- [x] Cross-compiler set up. → [toolchain.md](toolchain.md) (static armv7 musl, verified on-drone 2026-07-03)
- [x] GPIO method figured out: sysfs refuses the motor pins → direct `/dev/mem` register access (works).
- [x] **First spin under our control (2026-07-03)** via `motortest.c` — isolated the select-line polarity
      (hi-Z, not drive-low) as the fix. Handshake replies `e0 00` on all 4.
- [ ] **Fold the winning config (hi-Z select during run) into `motorspin.c`** so we have a clean
      `motorspin <motor 1-4> <pwm> <sec>` characterization tool. *(in progress this session)*
- [ ] **Props OFF, drone secured.** Characterize each motor 1–4: minimum reliable spin PWM, whether the
      LED reads green (healthy), and confirm the ~3 s no-load cutout. `fault(gpio176)` read is currently
      unreliable (input buffer likely off — DATAIN reads 0); revisit pad input-enable if we need it.
- [ ] Map motor number/slot (1–4) → physical position and spin direction (CW/CCW).
- [ ] **Log sensors during a single-motor spin** → vibration/disturbance map. Cross-links with [sensors.md](sensors.md).

## Gotchas
- **Props off, always.** Propless motors self-cut after ~3 s (no-load detect) — use it as a
  natural safety limit for first tests.
- Undersized power = brownout-reboot mid-spin (and undervolt on a board driving an inductive
  load is a good way to corrupt state). Battery or ≥10 A only.
- Don't reflash the BLCs — they already have working firmware; flashing is unnecessary risk.
