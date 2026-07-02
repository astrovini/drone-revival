#!/usr/bin/env python3
"""A stand-in for the AR.Drone's AT-command port, for testing at_link.py locally.

Listens on UDP <host>:5556, decodes every AT command it receives, and prints
what the real drone would understand — including reversing the PCMD float
encoding so you can confirm the numbers survive the round-trip. No drone, no
WiFi change.

    python3 scripts/control/fake_drone.py
    # then, in another terminal:
    python3 scripts/control/at_link.py --host 127.0.0.1 --flight
"""

import argparse
import socket
import struct

REF_BASE = 0x11540000
TAKEOFF_BIT = 1 << 9
EMERGENCY_BIT = 1 << 8


def _i2f(i):
    """Reverse of at_link._f2i: int bit-pattern back to the float it encodes."""
    return struct.unpack("f", struct.pack("i", int(i)))[0]


def decode(cmd):
    head, args = cmd.split("=", 1)
    name = head[3:]                       # strip "AT*"
    parts = args.split(",")
    seq, rest = parts[0], parts[1:]

    if name == "REF":
        v = int(rest[0])
        what = "TAKEOFF" if v & TAKEOFF_BIT else "EMERGENCY" if v & EMERGENCY_BIT else "LAND / idle"
        return seq, f"REF    {what}"
    if name == "PCMD":
        flag = int(rest[0])
        if flag == 0:
            return seq, "PCMD   HOVER (auto-stabilise)"
        roll, pitch, gaz, yaw = (_i2f(x) for x in rest[1:5])
        return seq, f"PCMD   roll={roll:+.2f} pitch={pitch:+.2f} gaz={gaz:+.2f} yaw={yaw:+.2f}"
    if name == "LED":
        anim, freq_i, dur = rest
        return seq, f"LED    anim={anim} {_i2f(freq_i):.1f}Hz {dur}s"
    if name == "COMWDG":
        return seq, "COMWDG (watchdog reset)"
    if name == "FTRIM":
        return seq, "FTRIM  (flat trim)"
    if name == "CONFIG":
        return seq, "CONFIG " + ",".join(rest)
    return seq, f"{name} {rest}"


def main():
    p = argparse.ArgumentParser(description="Fake AR.Drone AT-command listener")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5556)
    a = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.host, a.port))
    print(f"fake drone listening on {a.host}:{a.port}  (Ctrl-C to stop)")

    last_seq = 0
    try:
        while True:
            data, _ = sock.recvfrom(4096)
            for cmd in data.decode("ascii", "replace").split("\r"):
                cmd = cmd.strip()
                if not cmd:
                    continue
                try:
                    seq, text = decode(cmd)
                    order = "" if int(seq) > last_seq else "   <-- OUT OF ORDER"
                    last_seq = max(last_seq, int(seq))
                    print(f"seq {int(seq):<4} {text}{order}")
                except Exception:
                    print("?? " + cmd)
    except KeyboardInterrupt:
        print("\nfake drone stopped")


if __name__ == "__main__":
    main()
