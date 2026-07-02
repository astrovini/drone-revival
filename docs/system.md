# System & Foundation

Access, ports, config, power, and recovery — the layer every other area depends on.

## Status
- ✅ Root shell over WiFi telnet, confirmed.
- ✅ FTP file transfer working (port 21 root `/data/video`, port 5551 root `/update`).
- ✅ Config (`/data/config.ini`) readable; AT command channel understood.
- ✅ Mac→drone AT flight-command loop built + locally verified (`scripts/control/at_link.py`); real-drone test pending.
- ✅ Healthy 3S battery installed (~12.3 V). Logic-level power rock solid.
- ⬜ Stock-firmware escape kit not yet backed up (see Next steps).

## How it works

**Two boards:** the **mainboard** (ARM/Linux: WiFi, cameras, video, runs `program.elf`)
and the **navboard** (sensors → PIC → UART to mainboard).

**The stock flight app** `program.elf` owns the navboard, motors, and cameras while running,
and is guarded by `program.elf.respawner.sh`. To free the hardware:
```sh
kill -9 $(ps | grep respawner | grep -v grep | awk '{print $1}')   # respawner FIRST
killall -9 program.elf
```
A battery cycle restores everything to stock.

**Port map**
| Port | Proto | Purpose |
|---|---|---|
| 23 | TCP | telnet shell (root, no password) |
| 21 / 5551 | TCP | FTP — port 21 root = `/data/video`, port 5551 root = `/update` |
| 5554 | UDP | navdata (telemetry out) |
| 5555 | TCP | video stream (PaVE-framed H.264) |
| 5556 | UDP | AT commands (control / config in) |
| 5559 | TCP | control channel (config read / ACK) |

## Recipes that worked

**Connect (macOS):**
```bash
ping 192.168.1.1            # confirm reachable on the drone's WiFi
telnet 192.168.1.1          # macOS dropped telnet → `brew install telnet`, or:
nc 192.168.1.1 23
```
macOS gotchas: no bundled `telnet`/`ftp` (use `nc` / `curl ftp://...`); don't be on another
`192.168.1.x` LAN simultaneously; if ever phone-paired, press the reset button to unpair.

**Pull a file off the drone (port 21):**
```bash
# on the drone: cp /the/file /data/video/    then on the Mac:
curl -o local.name ftp://192.168.1.1/the.file
```
No `base64`/`unzip` on board → unzip on the Mac; for tiny binaries, `od -An -tx1 -v file`
in telnet and paste the hex out.

**Change a live setting (config doesn't stick if edited on disk — use AT commands):**
```bash
printf '\x01\x00\x00\x00' | nc -u -w0 192.168.1.1 5554        # nudge navdata alive
for i in $(seq 1 15); do
  printf "AT*CONFIG=$i,\"section:key\",\"value\"\r" | nc -u -w0 192.168.1.1 5556
  sleep 0.05
done
```
Note the **`section:key`** format (e.g. `control:outdoor`). `program.elf` rewrites
`config.ini` from memory, so on-disk edits are unreliable — the AT channel is authoritative.

## Power (the gating constraint)
- 3S LiPo: 11.1 V nominal (9.6 V empty → 12.6 V full). Bench supply: set **11.1 V**
  (12 V fine). Never exceed 12.6 V; never drop below ~9.5 V (`vbat_min = 9000` mV cutoff).
- **Current is the real limit:** logic/sensors/cameras <1 A (any supply); **motors ~8–10 A**
  (battery or ≥10 A supply only — a 2 A supply browns out the instant a motor spins).
- **Polarity = the one irreversible mistake.** Match `+` to the drone's `+` pin (old
  battery's red wire); verify with a multimeter before connecting.

## Next steps
- [ ] **Free win:** flip `outdoor`→indoor via AT (`AT*CONFIG=…,"control:outdoor","FALSE"`)
      and feel the difference — likely the biggest single "feels less twitchy" change. No motors.
- [x] **Mac→drone flight-command loop — built + locally verified.** `scripts/control/at_link.py`
      speaks `AT*REF` (takeoff/land/emergency), `AT*PCMD` (roll/pitch/gaz/yaw + enable), `AT*COMWDG`,
      `AT*FTRIM`, `AT*LED` as a continuous ~30 Hz sequenced stream; PCMD floats encoded as the
      IEEE-754 int bit-pattern. Verified end-to-end against `scripts/control/fake_drone.py` (decodes
      every command; float round-trip and strict sequence order confirmed). No drone/WiFi for that test.
- [ ] **Confirm on the real drone:** join its WiFi, run `at_link.py --host 192.168.1.1` (LED only)
      to prove the pipe, then `--flight` **props OFF** to watch the motors respond.
- [ ] **Back up the escape kit** before any flashing experiments (one-time insurance):
      copy `program.elf`, `/data/config.ini` (your unique calibration!), and `/factory` to `backups/`.
- [ ] Dump and skim the boot scripts (`/etc/init.d/rcS`, `check_update.sh`) to fully map startup.
- [ ] Decide whether to ever flash (Paparazzi/custom kernel) — until then we stay in the
      *run-your-own-binary* model that never touches flash.

## Gotchas
- `program.elf` reclaims hardware via the respawner — kill it first, every time.
- Two FTP servers on different ports with different roots — easy to hit the wrong one.
- BusyBox is stripped: `head -50` fails (use `head -n 50`), no `base64`/`unzip`/`gcc`.
