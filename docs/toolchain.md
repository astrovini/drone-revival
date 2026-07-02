# Build Toolchain (ARM cross-compiler)

The drone has **no `gcc` on board**, so any custom program (motor test, sensor logger,
PID controller) is **cross-compiled on the Mac** and FTP'd over. This is a discrete setup
task that **gates the motors and control areas**.

## Status
- ⬜ Not set up yet. This is the next concrete unblocked task.

## Target facts (what we're compiling for)
- CPU: ARM Cortex-A8 (OMAP3630), `armv7l`, with NEON / VFPv3.
- OS: Linux 2.6.32, glibc-era, BusyBox userland.
- Binaries must be statically linked or match the on-board libc — **static is simplest**
  to avoid library-version mismatches on the old rootfs.

## Plan / next steps
- [ ] Install an `arm-linux-gnueabihf` (hard-float) cross toolchain on macOS. Options,
      easiest first:
  - **Docker** with a Linux ARM cross image (most reproducible, no macOS toolchain pain).
  - Prebuilt macOS cross-GCC (Homebrew tap or ARM's official GNU toolchain).
- [ ] Compile a trivial `hello.c` **statically** (`-static -march=armv7-a -mfpu=neon`),
      FTP to `/data/video/`, `chmod +x`, and run it over telnet — proves the toolchain
      end-to-end before anything hardware-touching.
- [ ] Clone `ardrone/ardrone` (GitHub) — it has working `navboard`, `attitude`,
      `motorboard`, and `vbat` drivers/demos written for exactly this hardware.
- [ ] Build the `motorboard` and `navboard` demos first (they exercise the two UARTs we
      care about) → hands off to **[motors.md](motors.md)** and **[sensors.md](sensors.md)**.

## Deploy workflow (once toolchain works)
```bash
# Mac: build → push
arm-linux-gnueabihf-gcc -static -march=armv7-a -mfpu=neon -o myprog myprog.c
# copy onto the drone (port 21 FTP root = /data/video)
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
