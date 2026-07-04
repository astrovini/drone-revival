# Build Toolchain (ARM cross-compiler)

The drone has **no `gcc` on board**, so any custom program (motor test, sensor logger,
PID controller) is **cross-compiled on the Mac** and FTP'd over. This is a discrete setup
task that **gates the motors and control areas**.

## Status
- ✅ Cross-toolchain installed on the Mac: **`armv7-unknown-linux-musleabihf` (GCC 15.2.0)**
  via the `messense/macos-cross-toolchains` brew tap — a *native macOS* cross-GCC (no VM/Docker).
  Needed a one-time `brew trust messense/macos-cross-toolchains` (new Homebrew blocks untrusted taps).
- ✅ `hello.c` builds to a **fully static armv7 hard-float Linux ELF** (35 KB; no INTERP block,
  zero NEEDED libs, Version5 EABI) — verified on the Mac with the toolchain's own `readelf`.
  Source + `build.sh` live in [`scripts/toolchain/`](../scripts/toolchain/).
- ✅ **Confirmed on the drone (2026-07-03):** FTP'd `hello` to `/data/video`, ran it over telnet →
  `pointer width: 4 bytes` and `uname: Linux uclibc 2.6.32.9-g980dab2 armv7l`. **Toolchain proven end-to-end.**
  Bonus fact from that run: the rootfs hostname/userland is **`uclibc`** — the drone's own libc is
  **uClibc**, not glibc. A *dynamic* glibc binary would likely not have run; fully-static musl made libc a non-issue.

> **Why musl, not the glibc `gnueabihf` the plan below first suggested:** for a *fully static*
> binary the drone's own libc is irrelevant (nothing links against it), so we pick the libc that
> statically links cleanest on a 2.6.32 kernel — that's **musl** (glibc-static has the NSS-dlopen
> footgun and occasionally needs newer kernel features). Hard-float is fine for a static binary:
> it carries its own libc and the Cortex-A8 has VFPv3/NEON, so nothing on the drone has to match its ABI.

## Target facts (what we're compiling for)
- CPU: ARM Cortex-A8 (OMAP3630), `armv7l`, with NEON / VFPv3.
- OS: Linux 2.6.32, glibc-era, BusyBox userland.
- Binaries must be statically linked or match the on-board libc — **static is simplest**
  to avoid library-version mismatches on the old rootfs.

## Plan / next steps
- [x] Install a static-capable armv7 cross toolchain on macOS. **Chosen:** the prebuilt
      Homebrew cross-GCC (`messense/macos-cross-toolchains` → `armv7-unknown-linux-musleabihf`).
      No VM. (Docker was the other option but this Mac has no container runtime, so the native
      brew toolchain is far lighter — see the "why musl" note above.)
- [x] Compile a trivial `hello.c` **statically** (`-static -march=armv7-a -mfpu=neon`) and
      confirm it's a static armv7 ELF. Done via `scripts/toolchain/build.sh`.
- [x] FTP `hello` to `/data/video/`, `chmod +x`, run it over telnet — proved the toolchain
      end-to-end **on the hardware** (2026-07-03). ✅
- [ ] Get a motor/nav driver to cross-compile. **Correction (2026-07-03):** `ardrone/ardrone`
      (Hugo Perquin) is AR.Drone **1.0** code — same serial framing as the 2.0, but its device
      path (`/dev/ttyPA1`) and GPIO init are 1.0-specific and won't work on our board. For the
      **2.0** use **Paparazzi UAV's `sw/airborne/boards/ardrone/`** (`/dev/ttyO0`, GPIO 171–176).
      Full 1.0-vs-2.0 diff in **[motors.md](motors.md)**.
- [ ] Build a minimal motor/nav tool (2.0-correct) → hands off to **[motors.md](motors.md)**
      and **[sensors.md](sensors.md)**.

## Deploy workflow (once toolchain works)
```bash
# Mac: build → push  (build.sh wraps the exact flags below)
armv7-unknown-linux-musleabihf-gcc -static -march=armv7-a -mfpu=neon -O2 -o myprog myprog.c
#   or simply:  scripts/toolchain/build.sh myprog.c
# copy onto the drone (port 21 FTP root = /data/video) — Mac must be on the drone's WiFi
curl -T myprog ftp://192.168.1.1/myprog
# drone (telnet): make runnable + run after freeing hardware
chmod +x /data/video/myprog
# (kill respawner + program.elf first — see system.md)
/data/video/myprog
```

## Gotchas
- macOS native compilers won't produce ARM-Linux binaries — must be a *cross* toolchain
  (or build inside Linux/Docker).
- Dynamic linking against a newer glibc will fail on the 2.6.32 rootfs → prefer `-static`.
- Keep source under `scripts/<area>/`; keep built binaries out of git-noise (they're large).
