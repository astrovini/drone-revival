#!/usr/bin/env python3
"""Read the AR.Drone 2.0 navdata stream — attitude + per-motor PWM.

The drone streams telemetry over UDP 5554. This tool wakes that stream, decodes
it, and shows/logs what the drone thinks is happening while the stock firmware
flies it:

  - ATTITUDE (roll/pitch/yaw): if the drone believes it is tilted while sitting
    flat, its stabiliser drives the motors asymmetrically to "correct".
  - MOTOR PWM (m1..m4, with --full): the command sent to each motor.

Bootstrap handshake (confirmed live on this 2.4.8 drone): after the wake packet the drone sends
empty "bootstrap" packets (no option blocks). To leave bootstrap you must FIRST register the
multiconfig session (CONFIG_IDS + custom:session_id/profile_id/application_id) — a CONFIG naming an
unregistered session is ACKed but silently dropped — THEN send general:navdata_demo=TRUE with the
ACK cycle (AT*CONFIG, wait for COMMAND_ACK, AT*CTRL=5,0). Motor PWM (tag 9) is then added with
general:navdata_options=513 (attitude + PWM). See wake_navdata().

--log writes a timestamped CSV to data/radio/ (only real data rows, not
bootstrap). Run it FIRST, wait until attitude appears, then start tango_fly.py.

    python3 scripts/control/navdata.py --host 192.168.1.1              # demo (attitude)
    python3 scripts/control/navdata.py --host 192.168.1.1 --full --log # + motor PWM
    python3 scripts/control/navdata.py --selftest                      # verify parser
"""

import argparse
import csv
import datetime
import os
import socket
import struct
import sys
import time

from at_link import Drone

NAVDATA_PORT = 5554
HEADER = 0x55667788
WAKE = b"\x01\x00\x00\x00"
COMMAND_MASK = 1 << 6   # ARDRONE state bit: a CONFIG was received, waiting for AT*CTRL ack
PWM_TAG = 9                            # NAVDATA_PWM_TAG (per-motor PWM lives here)
NAVDATA_OPTS_PWM = (1 << 0) | (1 << PWM_TAG)   # navdata_options bitmask: demo (tag 0) + motor PWM

CTRL_STATES = {0: "DEFAULT", 1: "INIT", 2: "LANDED", 3: "FLYING", 4: "HOVERING",
               5: "TEST", 6: "TAKEOFF", 7: "GOTOFIX", 8: "LANDING", 9: "LOOPING"}

# ardrone_state bit names (navdata_common.h)
STATE_BITS = {
    0: "FLY", 1: "VIDEO", 2: "VISION", 3: "CONTROL", 4: "ALTITUDE", 5: "USER_FB",
    6: "COMMAND_ACK", 7: "CAMERA", 8: "TRAVELLING", 9: "USB", 10: "NAVDATA_DEMO",
    11: "BOOTSTRAP", 12: "MOTORS_PROBLEM", 13: "COM_LOST", 14: "SW_FAULT", 15: "VBAT_LOW",
    16: "USER_EL", 17: "TIMER", 18: "MAGNETO_CALIB", 19: "ANGLES_OOR", 20: "WIND",
    21: "ULTRASOUND", 22: "CUTOUT", 23: "PIC_VERSION", 24: "ATCODEC_THREAD",
    25: "NAVDATA_THREAD", 26: "VIDEO_THREAD", 27: "ACQ_THREAD", 28: "CTRL_WATCHDOG",
    29: "ADC_WATCHDOG", 30: "COM_WATCHDOG", 31: "EMERGENCY",
}


def decode_state(state):
    return " ".join(name for bit, name in STATE_BITS.items() if state & (1 << bit))


def probe(host):
    """Verbose bootstrap diagnostic: send wake + config + steady COMWDG + ACK, and print
    every packet's decoded state bits. Goal: watch BOOTSTRAP disappear. Ctrl-C to stop."""
    print("PROBE — sending: wake, CONFIG navdata_demo, COMWDG (2.5 Hz), ACK when asked.")
    print("Watch the bit list on each line: we want BOOTSTRAP to drop off.\n")
    drone = Drone(host, verbose=True)                 # verbose: prints each AT command we send
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", NAVDATA_PORT))                      # reference clients bind locally to 5554
    sock.settimeout(0.5)

    def wake():
        sock.sendto(WAKE, (host, NAVDATA_PORT))

    wake()
    drone.init_session()                               # register multiconfig session FIRST
    drone.config_ids()
    drone.config("general:navdata_demo", "TRUE")
    last_wdg = last_cfg = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now - last_wdg > 0.4:
                drone.comwdg()
                last_wdg = now
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                wake()
                continue
            nd = parse_navdata(data)
            if not nd:
                print(f"  <- non-navdata packet, {len(data)} bytes")
                continue
            if nd["state"] & COMMAND_MASK:
                drone.ctrl_ack()
            elif not has_options(nd) and now - last_cfg > 2.0:
                drone.config_ids()
                drone.config("general:navdata_demo", "TRUE")
                last_cfg = now
            print(f"  <- len {len(data):>4} seq {nd['seq']:<6} state 0x{nd['state']:08x} "
                  f"tags {nd['tags']}  [{decode_state(nd['state'])}]")
    except KeyboardInterrupt:
        print("\nprobe stopped")


def parse_navdata(data):
    """Return {seq, state, tags[], demo{...}, pwm[m1..m4]} from one packet, or None."""
    if len(data) < 16:
        return None
    header, state, seq, _vision = struct.unpack_from("<IIII", data, 0)
    if header != HEADER:
        return None
    out = {"seq": seq, "state": state, "tags": []}
    off = 16
    while off + 4 <= len(data):
        tag, size = struct.unpack_from("<HH", data, off)
        if size < 4 or off + size > len(data):
            break
        out["tags"].append(tag)
        payload = data[off + 4: off + size]
        if tag == 0 and len(payload) >= 24:                      # navdata_demo
            ctrl_state, vbat = struct.unpack_from("<II", payload, 0)
            theta, phi, psi = struct.unpack_from("<fff", payload, 8)   # milli-degrees
            altitude = struct.unpack_from("<i", payload, 20)[0]
            out["demo"] = {"ctrl": ctrl_state, "bat": vbat,
                           "roll": phi / 1000.0, "pitch": theta / 1000.0,
                           "yaw": psi / 1000.0, "alt_mm": altitude}
        elif tag == 9 and len(payload) >= 4:                     # navdata_pwm (NAVDATA_PWM_TAG=9)
            out["pwm"] = list(struct.unpack_from("<BBBB", payload, 0))  # motor1..4 are first 4 bytes
        if tag == 0xFFFF:
            break
        off += size
    return out


def has_options(nd):
    return "demo" in nd or "pwm" in nd


def format_line(nd):
    d = nd.get("demo")
    if d:
        st = CTRL_STATES.get(d["ctrl"] >> 16, d["ctrl"] >> 16)
        base = (f"seq {nd['seq']:<6} {st:<9} bat {d['bat']:>3}%  "
                f"roll {d['roll']:+6.2f}° pitch {d['pitch']:+6.2f}° yaw {d['yaw']:+7.2f}°  "
                f"alt {d['alt_mm']:>5}mm")
    else:
        base = f"seq {nd['seq']:<6} (attitude off)"
    m = nd.get("pwm")
    base += f"  M[{m[0]:3d} {m[1]:3d} {m[2]:3d} {m[3]:3d}]" if m else "  M[motor PWM off]"
    return base


def selftest():
    demo_payload = struct.pack("<IIfffiffff", (2 << 16), 78,
                               1500.0, -800.0, 12400.0, 0, 0.0, 0.0, 0.0, 0.0)
    demo = struct.pack("<HH", 0, 4 + len(demo_payload)) + demo_payload
    pwm_payload = struct.pack("<BBBBBBBB", 130, 128, 131, 129, 0, 0, 0, 0)
    pwm = struct.pack("<HH", 9, 4 + len(pwm_payload)) + pwm_payload   # tag 9 = NAVDATA_PWM_TAG
    cksum = struct.pack("<HHI", 0xFFFF, 8, 0)
    packet = struct.pack("<IIII", HEADER, 0, 42, 0) + demo + pwm + cksum
    nd = parse_navdata(packet)
    print("parsed:", nd)
    print(format_line(nd))
    assert nd["tags"] == [0, 9, 0xFFFF]
    assert nd["demo"]["bat"] == 78
    assert abs(nd["demo"]["pitch"] - 1.5) < 1e-6 and abs(nd["demo"]["roll"] + 0.8) < 1e-6
    assert nd["pwm"] == [130, 128, 131, 129]
    print("selftest OK")


def open_log():
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    d = os.path.join(repo, "data", "radio")
    os.makedirs(d, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(d, f"navdata_{ts}.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "wall", "seq", "state", "bat", "roll_deg", "pitch_deg", "yaw_deg",
                "alt_mm", "m1", "m2", "m3", "m4"])
    return f, w, path


def _gated_config(drone, sock, host, key, value, settle, want_tag=None):
    """Apply one config the way --probe does (empirically proven on this drone): keep (re)sending
    CONFIG_IDS + CONFIG, ACK whenever the drone raises COMMAND_ACK, and watch the stream. The caller
    must have registered the multiconfig session first (drone.init_session), or the stock 2.4.x
    firmware ACKs the CONFIG but silently drops it. Returns the option tags of the last packet seen
    ([] while still bootstrap); returns early once `want_tag` appears (or any option block if None)."""
    deadline = time.monotonic() + settle
    last_wake = last_wdg = 0.0
    last_cfg = time.monotonic()
    last_tags = []
    drone.config_ids()
    drone.config(key, value)
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_wake > 1.0:
            sock.sendto(WAKE, (host, NAVDATA_PORT)); last_wake = now
        if now - last_wdg > 0.4:
            drone.comwdg(); last_wdg = now
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        nd = parse_navdata(data)
        if not nd:
            continue
        opts = [t for t in nd["tags"] if t != 0xFFFF]
        last_tags = opts
        if want_tag is None and opts:                   # bootstrap cleared
            return opts
        if want_tag is not None and want_tag in opts:   # got the tag we were after
            return opts
        if nd["state"] & COMMAND_MASK:                  # drone raised ACK -> acknowledge it
            drone.ctrl_ack()
        elif now - last_cfg > 2.0:                      # otherwise keep re-sending the config
            drone.config_ids()
            drone.config(key, value)
            last_cfg = now
    return last_tags


def wake_navdata(drone, sock, host, want_full=False, settle=15.0):
    """Break navdata out of BOOTSTRAP and, if want_full, upgrade to motor PWM. Prints one line per
    phase; returns 'full' (attitude + PWM), 'demo' (attitude only), or None (never left bootstrap).

    Two facts proven live on this 2.4.8 drone: (1) a CONFIG naming an unregistered multiconfig
    session is ACKed but silently dropped, so we MUST register the session first (init_session);
    (2) only navdata_demo=TRUE exits bootstrap. So: register session, break bootstrap with TRUE,
    then (if want_full) add PWM (tag 9) — first additively via navdata_options, then via full navdata."""
    drone.init_session()                                # register multiconfig session or CONFIG is dropped
    tags = _gated_config(drone, sock, host, "general:navdata_demo", "TRUE", settle)
    if not tags:
        return None
    print(f"  bootstrap cleared — attitude live (tags {sorted(tags)}).")
    if not want_full:
        return "demo"

    # 2a: ask for the PWM tag additively via navdata_options, staying in demo mode.
    tags = _gated_config(drone, sock, host, "general:navdata_options", str(NAVDATA_OPTS_PWM),
                         6.0, want_tag=PWM_TAG)
    if PWM_TAG in tags:
        print(f"  motor PWM live via navdata_options (tags {sorted(tags)}).")
        return "full"

    # 2b: fall back to full navdata (navdata_demo=FALSE, options already set above).
    tags = _gated_config(drone, sock, host, "general:navdata_demo", "FALSE", 6.0, want_tag=PWM_TAG)
    if PWM_TAG in tags:
        print(f"  motor PWM live via full navdata (tags {sorted(tags)}).")
        return "full"
    if not tags:                                        # FALSE knocked us back to bootstrap -> recover
        _gated_config(drone, sock, host, "general:navdata_demo", "TRUE", settle)
    print(f"  ! no motor PWM (tag 9) yet — last tags {sorted(tags)}. Attitude still available.")
    return "demo"


def main():
    p = argparse.ArgumentParser(description="AR.Drone 2.0 navdata (attitude + motor PWM) reader")
    p.add_argument("--host", default="192.168.1.1")
    p.add_argument("--full", action="store_true", help="enable full navdata (adds per-motor PWM)")
    p.add_argument("--log", action="store_true", help="write a timestamped CSV to data/radio/")
    p.add_argument("--probe", action="store_true", help="verbose diagnostic: print every packet's state bits")
    p.add_argument("--selftest", action="store_true", help="parse a synthetic packet and exit")
    a = p.parse_args()

    if a.selftest:
        selftest()
        return
    if a.probe:
        probe(a.host)
        return

    want_full = a.full                         # full adds motor PWM (tag 9) on top of attitude
    drone = Drone(a.host, verbose=False)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", NAVDATA_PORT))                      # reference clients bind locally to 5554
    sock.settimeout(0.5)

    print(f"listening for navdata from {a.host}:{NAVDATA_PORT}  "
          f"(requesting {'attitude + motor PWM' if a.full else 'attitude only'}, Ctrl-C to stop)")
    f = w = path = None
    if a.log:
        f, w, path = open_log()
        print(f"logging to {path}")

    print("waking navdata out of bootstrap (clean ACK-gated handshake)...")
    status = wake_navdata(drone, sock, a.host, want_full)
    if status == "full":
        print("navdata live: attitude + motor PWM.\n")
    elif status == "demo":
        print("navdata live: attitude.\n")
    else:
        print("! still in bootstrap after handshake — will keep retrying (run --probe if it persists).\n")

    t0 = time.monotonic()
    last_wdg = t0
    last_retry = t0
    try:
        while True:
            now = time.monotonic()
            if now - last_wdg > 0.4:
                drone.comwdg()                            # keep the comms watchdog reset
                last_wdg = now
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                sock.sendto(WAKE, (a.host, NAVDATA_PORT))
                continue

            nd = parse_navdata(data)
            if not nd:
                continue

            if not has_options(nd):                       # slipped back to bootstrap -> re-handshake
                if now - last_retry > 2.0:
                    wake_navdata(drone, sock, a.host, want_full, settle=6.0)
                    last_retry = time.monotonic()
                continue

            sys.stdout.write("\r\033[2K" + format_line(nd))
            sys.stdout.flush()
            if w:
                d = nd.get("demo", {})
                m = nd.get("pwm", ["", "", "", ""])
                w.writerow([f"{time.monotonic() - t0:.3f}", f"{time.time():.3f}", nd["seq"],
                            (d.get("ctrl", 0) >> 16), d.get("bat", ""),
                            d.get("roll", ""), d.get("pitch", ""), d.get("yaw", ""),
                            d.get("alt_mm", ""), m[0], m[1], m[2], m[3]])
    except KeyboardInterrupt:
        if f:
            f.close()
            print(f"\nlog saved: {path}")
        else:
            sys.stdout.write("\n")


if __name__ == "__main__":
    main()
