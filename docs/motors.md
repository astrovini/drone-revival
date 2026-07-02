# Motors (actuation)

The 4 brushless motors, each an independent reprogrammable controller ("BLC") on the motor
UART (`/dev/ttyO0`). **This is the first area that needs real motor power** — now unblocked
by the healthy battery, but still gated on the cross-compiler.

## Status
- ✅ Motor bus (`/dev/ttyO0`) enumerated; UART present.
- ✅ Power unblocked: healthy 3S battery (~10 A ceiling) installed.
- ✅ Factory test scripts read and understood (see Findings) — but **not reusable**.
- ⬜ Cross-compiler not set up → can't build a motor driver yet. → [toolchain.md](toolchain.md)
- ⬜ No motor has been spun yet.

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

→ **Realistic path:** the open-source `motorboard` driver from `ardrone/ardrone` (GitHub),
which implements the documented init + 200 Hz PWM frames. Cross-compile and FTP it over.

## Next steps (in order)
- [ ] Set up the cross-compiler. → [toolchain.md](toolchain.md)
- [ ] Build the `ardrone/ardrone` `motorboard` demo (and `vbat` for voltage sanity).
- [ ] **Props OFF, drone secured, healthy battery.** Free hardware (kill respawner +
      `program.elf`), then spin **ONE** motor briefly at low speed.
- [ ] Map motor number (1–4) → physical position and spin direction (CW/CCW).
- [ ] Characterize each ESC: minimum reliable spin, responsiveness, and confirm the
      ~3 s no-load cutout (propless self-shutdown).
- [ ] **Log sensors during a single-motor spin** → vibration/disturbance map.
      Cross-links with [sensors.md](sensors.md).

## Gotchas
- **Props off, always.** Propless motors self-cut after ~3 s (no-load detect) — use it as a
  natural safety limit for first tests.
- Undersized power = brownout-reboot mid-spin (and undervolt on a board driving an inductive
  load is a good way to corrupt state). Battery or ≥10 A only.
- Don't reflash the BLCs — they already have working firmware; flashing is unnecessary risk.
