# Vision (cameras)

Two cameras through the OMAP ISP → DSP H.264 encoder → network. The drone encodes; the Mac
does the heavy lifting. **This whole area works on the bench with no motors and no flight.**

## Status
- ✅ **Front camera** — 720p, streamed downscaled to 640×360 H.264. Live in `ffplay`.
- ✅ **Bottom camera** — QVGA 320×240 optical-flow cam. Switched to and confirmed (floor view).
- ✅ Full chain verified: `ov7670` + `soc1040` + `omap3_isp` + DSP H.264 encoder all up.
- ⬜ No clean PaVE-stripping pipeline (ffplay tolerates the framing but throws warnings).
- ⬜ No computer vision / capture / optical-flow experiments yet.

## How it works
Video is served on **TCP 5555** as a **PaVE-framed H.264 stream** (Parrot Video
Encapsulation: a 64-byte header before each frame, then standard H.264 NAL units —
`67`=SPS, `68`=PPS, `65`=IDR keyframe). `program.elf` does the encoding, so you *don't*
kill it for video. Camera selection is a config value (`video:video_channel`), not a port.

## Recipes that worked

**Prove the stream is live (zero install, Mac):**
```bash
nc 192.168.1.1 5555 | head -c 2000 | xxd | head -40   # look for "PaVE" = 50 61 56 45
```

**Watch it (ffplay handles PaVE framing best):**
```bash
brew install ffmpeg
ffplay -fflags nobuffer -flags low_delay tcp://192.168.1.1:5555
```
The `Failed to parse header of NALU type 0` / `sps_id 32 out of range` warnings are
harmless — ffplay stepping over PaVE headers; the picture still decodes.

**Switch to the bottom camera (Mac):** hammer the AT config with a rising seq (see
[system.md](system.md) AT recipe), key `video:video_channel`, value `1` (pure bottom;
`0`=front, `2`/`3`=picture-in-picture). Restart ffplay to pick it up.

## Next steps
- [ ] **Clean PaVE stripper** — a small script that drops the 64-byte header per frame and
      pipes pure H.264 to any player or to OpenCV (kills the warnings, enables CV input).
- [ ] **Capture stills/clips** from both cameras → `data/vision/`.
- [ ] **Computer vision on the Mac** (live feed): start simple — color tracking, then
      AprilTag/marker detection, motion detection, face/object tracking.
- [ ] **Tap the built-in detection** — config `[detect]` section (`enemy_colors`,
      `groundstripe_colors`, `detect_type`); Parrot's onboard tag/color tracker, readable via navdata.
- [ ] **Bottom-camera optical flow** experiment — the basis of indoor position/drift estimation.
- [ ] **(Later) vision-guided flight** — line-follow, hover-over-marker, patrol. Sits on top
      of stable flight → depends on [control.md](control.md).

## Gotchas
- Pulling a *clean, viewable* picture is fiddlier than the sensor test — PaVE framing fights
  naive players. ffplay is the pragmatic default; the stripper is the robust fix.
- Bottom cam is low-res/grainy *by design* (built for fast ground tracking, not pretty video).
- ffplay's banner reports the resolution from the first SPS at connect — it won't re-announce
  a mid-stream camera switch; reconnect to see the new size.
