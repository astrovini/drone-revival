# Sensors (IMU / navboard)

The 9-DOF sensing foundation: 3-axis gyro + 3-axis accel + magnetometer + barometer +
downward sonar, all on the navboard, streamed over `/dev/ttyO1` at ~200 Hz.

## Status
- ✅ **Accelerometer** — all 3 axes live; correctly senses gravity (~530-count offset on
  the down axis). At-rest flat ≈ `2052 / 2012 / 2564` (raw ADC counts).
- ✅ **Gyros** — all 3 axes live; spike on rotation, snap back to baseline, **no drift**.
  Correct axis isolation (roll/pitch lit X/Y to ±2000, yaw Z stayed quiet).
- ✅ **Idle baseline captured & characterized** — 6000 frames @ ~200 Hz, flat/still/untouched,
  motors off (`data/sensors/idle_flat_30s.csv`). At-rest per-axis mean ± std (raw counts → physical):
  - accel  ax `2074.6 ±1.9` (−0.015 g ±3.8 mg)  ay `2047.3 ±1.7` (+0.005 g ±3.2 mg)  az `2567.3 ±2.0` (−0.99 g ±4.0 mg)
  - gyro   gx `5.5 ±1.3` (±0.080 °/s)  gy `33.9 ±1.2` (±0.073 °/s)  gz `57.6 ±1.0` (±0.060 °/s)
  - No meaningful drift over 30 s (≤2.6 counts LSQ, within noise), no periodic tone. Noise is
    **quantization-limited** (~3–4 distinct accel codes) — random noise is sub-LSB. Accel resolves
    the surface as ~0.9° off level. Reference fingerprint; compare motor-on logs against it. → [motors.md](motors.md)
- ✅ **Calibration decoded & validated from `config.ini`** (saved to `backups/config.ini`):
  - **gyro scale** `gyros_gains ≈ ±1.059e-3` rad/s per count → **1 count ≈ 0.0607 °/s** (~16.5 counts/°/s,
    the ±2000 °/s MEMS sensitivity); gain signs `{+,−,−}` → Y,Z axes inverted vs X.
  - **accel scale** `accs_gains` (3×3) maps counts→milli-g, diagonal ≈1.95 → **1 count ≈ 1.95 mg, 1 g ≈ 512 counts**.
    Formula `a_mg = accs_gains·raw + accs_offset` (offset in mg-space) verified: at-rest yields ~1 g on Z, ~0 on X/Y.
- ⚠️ **Gyro bias is volatile across boots — never hardcode it.** Stored `gyros_offset = {35.6, 25.0, 29.6}`
  counts vs our measured at-rest `{5.5, 33.9, 57.6}` differ by up to ~28 counts ≈ **1.8 °/s** (which
  integrates to ~108 °/min of phantom yaw). The stock firmware itself re-estimates and rewrites
  `gyros_offset` every boot (see `pwm_ref_gyros`, `gyro_offset_thr_*`), so the stored value is just the
  last boot's. **Trust the gains; measure the gyro offset fresh from stillness at each startup.**
  *(Supersedes the earlier note that bias "matched `gyros_offset` exactly" — it does not.)*
- ⬜ Magnetometer / barometer / sonar present but not yet decoded in our raw reader.
- ⚠️ **Magnetometer is corrupted by motor current at high thrust (2026-07-02).** Seen via the fused
  navdata yaw during props-off takeoffs (`data/radio/kbd_2026070*_18*.csv`): yaw snaps ~180° (to ≈−177°)
  in a **single frame**, **yaw-only** (roll/pitch unmoved → it's the mag path, the sole absolute heading
  reference), at a repeatable **thrust threshold** (~160–185 PWM), reproducibly. The motors' magnetic field
  overwhelms Earth's ~0.3 G field at the sensor. **Implications:** any yaw/heading use must weight the gyro
  heavily and treat the mag as a low-gain slow correction (or gyro-only yaw-hold for short flights); a mag
  calibration is needed and even so is imperfect because current varies. Full diagnosis in
  [radio.md](radio.md) (*Motor / attitude diagnosis*).
- ✅ **Fused-attitude idle baseline** (`data/sensors/idle_navdata_attitude_43s.csv`, 43 s via navdata @15 Hz,
  armed & still): roll/pitch hold within ±0.2° of a <0.3° bias (std <0.08°), yaw drifts +0.36°/min (gyro),
  sonar `alt_mm`=0, motors `0 0 0 0`. Confirms `program.elf`'s own fusion is quiet and unbiased at rest —
  complements the raw-count baseline above.
- ⬜ Scale factors known & validated (above), but not yet wired into the live reader. No sensor fusion yet.

## How it works
Free the navboard (kill respawner + `program.elf`), set the port raw, send `\001` to start
acquisition, then read frames. **Each frame starts `0x3A 0x00`:**
- bytes 2–3: sequence counter (increments → proof the stream is live)
- +0x04 / +0x06 / +0x08: accel X / Y / Z (int16)
- +0x0A / +0x0C / +0x0E: gyro X / Y / Z (int16)
- (further offsets carry mag / baro / sonar / vbat — to be mapped)

## Recipes that worked

**Live decoded readout (ACC + GYRO, until Ctrl-C):** see the awk script in
`ardrone2_revival_log.md` §4 — works today, watch the numbers move when you tilt/twist.

**Quick raw frame peek:**
```sh
stty -F /dev/ttyO1 raw -echo
echo -en "\001" > /dev/ttyO1
dd if=/dev/ttyO1 bs=4096 count=20 of=/dev/null 2>/dev/null   # flush stale
dd if=/dev/ttyO1 bs=60 count=4 2>/dev/null | hexdump -C
```

**Idle baseline logger:** [`scripts/sensors/log_idle.sh`](../scripts/sensors/log_idle.sh) —
run on the drone (flat/still), captures ~30 s of decoded ACC+GYRO to `/data/video/idle.csv`,
then pull it: `curl -o data/sensors/idle_flat_30s.csv ftp://192.168.1.1/idle.csv`.

## Next steps
- [x] **Capture a clean idle baseline** — drone flat, still, untouched, ~30 s @ 200 Hz →
      `data/sensors/idle_flat_30s.csv`, using [`scripts/sensors/log_idle.sh`](../scripts/sensors/log_idle.sh).
- [x] **Characterize noise** from that CSV: per-axis mean, stddev, slow drift, periodicity →
      results in **Status** above (quiet, no drift, quantization-limited; gyro bias not run-to-run repeatable).
- [ ] **Extend the decoder** to also pull magnetometer, barometer, sonar, and battery voltage
      (map their byte offsets in the frame).
- [~] **Convert raw counts → physical units** (deg/s, g) — scale factors derived & validated
      (see Status: `1 count ≈ 0.0607 °/s`, `1 count ≈ 1.95 mg`). Remaining: wire the conversion
      into the live reader, and **measure gyro offset fresh at startup** (don't use stored `gyros_offset`).
      Still TODO: magneto cal (`magneto_offset`, `magneto_radius`) — but note the mag is swamped by motor
      current at high thrust (see ⚠️ above), so plan for gyro-dominated yaw regardless of cal.
- [ ] **Sensor fusion** — complementary filter first (cheap), then optionally Kalman →
      clean roll/pitch/yaw estimate. *This output is the input to control.* → [control.md](control.md)
- [ ] **(After motors unblocked)** log sensors while one motor spins → vibration/disturbance
      map. Cross-links with [motors.md](motors.md).

## Gotchas
- Raw values are uncalibrated ADC counts — fine for "is it alive," but convert before any control.
- Must flush buffered frames after sending `\001` or you read seconds-old data.
- A bad attitude estimate makes everything downstream "learn a lie" — get fusion solid first.
