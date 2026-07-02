#!/usr/bin/env python3
"""AT-command link to the AR.Drone 2.0 — the Mac->drone "send loop".

Path A, drone half: speaks the stock firmware's AT command protocol over UDP so
the Mac can fly the drone (takeoff, tilt, land). Later the Tango bridge feeds
move() from the sticks; for now a scripted demo drives it so we can test the
protocol without a radio.

Protocol facts (AR.Drone 2.0):
  - UDP to <host>:5556. Each command is ASCII text ending in '\\r'.
  - Every command's first arg is a sequence number that must strictly increase;
    the drone drops anything older than the last it saw. We start at 1.
  - Commands must STREAM (~30 Hz) or the comms watchdog stops the drone.
  - PCMD tilt/gaz/yaw are floats -1..1 sent as the *integer bit-pattern* of the
    IEEE-754 float (the classic AR.Drone gotcha) -> see _f2i().

Test locally first (no drone, no WiFi change) — two terminals:
    python3 scripts/control/fake_drone.py
    python3 scripts/control/at_link.py --host 127.0.0.1 --flight
The fake drone decodes and prints every command so you can see the intent.

Then the real drone (join its WiFi first):
    python3 scripts/control/at_link.py --host 192.168.1.1            # LED test only
    python3 scripts/control/at_link.py --host 192.168.1.1 --flight   # PROPS OFF!
"""

import argparse
import socket
import struct
import time

AT_PORT = 5556

# AT*REF bit-field: bits 18,20,22,24,28 are always 1 (0x11540000 = "land/idle").
REF_BASE = 0x11540000
REF_TAKEOFF = REF_BASE | (1 << 9)    # set bit 9
REF_EMERGENCY = REF_BASE | (1 << 8)  # set bit 8

# Multiconfig session IDs (any consistent 8-hex values). On 2.4.x firmware every AT*CONFIG
# must name a REGISTERED session via CONFIG_IDS, or it is acked but silently dropped.
SESSION_ID = "a1b2c3d4"
USER_ID = "11112222"
APP_ID = "33334444"


def _f2i(x):
    """Reinterpret a float's 32-bit IEEE-754 pattern as a signed int (PCMD encoding)."""
    return struct.unpack("i", struct.pack("f", float(x)))[0]


def _clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


class Drone:
    """Fire-and-forget AT command sender. Not thread-safe; drive it from one loop."""

    def __init__(self, host, port=AT_PORT, verbose=True):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0
        self.verbose = verbose

    def _send(self, name, args=""):
        self.seq += 1
        payload = f"AT*{name}={self.seq}"
        if args:
            payload += f",{args}"
        payload += "\r"
        self.sock.sendto(payload.encode("ascii"), self.addr)
        if self.verbose:
            print("  -> " + payload.strip())

    # --- config / housekeeping -------------------------------------------
    def comwdg(self):
        self._send("COMWDG")                      # reset comms watchdog

    def config_ids(self):
        # Every AT*CONFIG must be preceded by CONFIG_IDS naming the multiconfig session.
        self._send("CONFIG_IDS", f'"{SESSION_ID}","{USER_ID}","{APP_ID}"')

    def config(self, key, value):
        self._send("CONFIG", f'"{key}","{value}"')

    def init_session(self):
        # Register the session so later CONFIG is APPLIED (2.4.x requires this). Referencing
        # a fresh session_id in CONFIG_IDS creates it; do it once at startup.
        for key, val in (("custom:session_id", SESSION_ID),
                         ("custom:profile_id", USER_ID),
                         ("custom:application_id", APP_ID)):
            self.config_ids()
            self.config(key, val)

    def ctrl_ack(self):
        # AT*CTRL ACK_CONTROL_MODE(5): acknowledge a received CONFIG (clears the state
        # COMMAND_MASK bit) so the drone applies it and leaves navdata bootstrap.
        self._send("CTRL", "5,0")

    def flat_trim(self):
        self._send("FTRIM")                       # set level reference on flat ground

    def led(self, anim=0, hz=2.0, seconds=3):
        self._send("LED", f"{anim},{_f2i(hz)},{seconds}")

    # --- flight ----------------------------------------------------------
    def takeoff(self):
        self._send("REF", str(REF_TAKEOFF))

    def land(self):
        self._send("REF", str(REF_BASE))

    def emergency(self):
        self._send("REF", str(REF_EMERGENCY))

    def hover(self):
        self._send("PCMD", "0,0,0,0,0")           # flag 0 -> ignore tilt, auto-hover

    def move(self, roll=0.0, pitch=0.0, gaz=0.0, yaw=0.0):
        # roll:  -left/+right   pitch: -forward/+back   gaz: -down/+up   yaw: -CCW/+CW
        vals = [_f2i(_clamp(roll)), _f2i(_clamp(pitch)),
                _f2i(_clamp(gaz)), _f2i(_clamp(yaw))]
        self._send("PCMD", "1," + ",".join(str(v) for v in vals))

    # --- helper: keep sending `fn` at `hz` for `seconds` -----------------
    def stream(self, seconds, fn, hz=30):
        period = 1.0 / hz
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            fn()
            time.sleep(period)


def demo(drone, flight):
    print(f"Target: {drone.addr[0]}:{drone.addr[1]}")
    drone.comwdg()
    drone.config("control:outdoor", "FALSE")      # indoor mode (gentler)
    drone.flat_trim()

    print("LED test (harmless — no motors)...")
    drone.led(anim=0, hz=2.0, seconds=2)
    drone.stream(2, drone.hover)                   # keep the link alive

    if not flight:
        print("Done: LED + link test. Re-run with --flight for takeoff/move/land.")
        return

    print("\n*** FLIGHT DEMO — PROPS OFF on the real drone! ***")
    for n in (3, 2, 1):
        print(f"  taking off in {n}...")
        time.sleep(1)
    drone.takeoff()
    drone.stream(2, drone.hover)                   # climb + settle
    print("nudge forward...")
    drone.stream(1, lambda: drone.move(pitch=-0.2))
    print("hover...")
    drone.stream(1, drone.hover)
    print("yaw right...")
    drone.stream(1, lambda: drone.move(yaw=0.3))
    print("landing...")
    drone.stream(1.5, drone.land)
    print("Done.")


def main():
    p = argparse.ArgumentParser(description="AR.Drone 2.0 AT-command send loop")
    p.add_argument("--host", default="127.0.0.1", help="drone IP (192.168.1.1) or 127.0.0.1 to test")
    p.add_argument("--port", type=int, default=AT_PORT)
    p.add_argument("--flight", action="store_true",
                   help="include takeoff/move/land (PROPS OFF on the real drone!)")
    a = p.parse_args()

    drone = Drone(a.host, a.port)
    try:
        demo(drone, a.flight)
    except KeyboardInterrupt:
        print("\ninterrupted -> sending land")
        drone.land()


if __name__ == "__main__":
    main()
