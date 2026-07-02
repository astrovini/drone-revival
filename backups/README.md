# backups/ — stock-firmware escape kit

One-time insurance so the stock firmware is *unloseable* before any flashing experiments.
Only *flashing* can lose the stock image — *running* your own binary never touches flash —
but back up anyway. Pull these off the drone (FTP via port 21, see `docs/system.md`):

- ✅ `factory/` — Parrot's factory FVT test scripts (`FVT{1,2,3}.zip` + extracted XML),
  pulled off the drone's `/factory`. Analyzed for the motor-test procedure — see `docs/motors.md`.
- ⬜ `program.elf` — the stock closed flight app (`cp /bin/program.elf /data/video/` then FTP).
- ⬜ `config.ini` — **your drone's unique calibration** (the one genuinely irreplaceable file).
- ⬜ rest of `/factory/` — factory calibration data (`*calibration*.txt`, `production_info.xml`).

Stock 2.4.8 firmware `.plf` is also archived online (see log §10) as a last-resort reflash.
