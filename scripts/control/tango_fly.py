#!/usr/bin/env python3
"""Tango -> drone bridge: fly the AR.Drone 2.0 with the TBS Tango 2.

Path A, the payoff: reads the Tango as a USB joystick (like tango_read.py) and
drives the drone over AT commands (via at_link.Drone). A small arm/emergency
state machine sits in the middle so the radio behaves safely.

Sticks (Mode 2, TAER):   axis0 throttle · axis1 roll · axis2 pitch · axis3 yaw
Switches (as axes):      A(4) arm · B(5) takeoff/land · C(6) mode · D(7) EMERGENCY

State machine:
    SAFE-LOCK : arm was ON at startup -> refuse to arm until you toggle it OFF once
    DISARMED  : no flight commands (link kept alive); flip A on -> flat-trim + ARMED
    ARMED     : B up = takeoff, B down = land, B center = fly with the sticks
    EMERGENCY : D on -> instant motor cut; flip D off to return to DISARMED

--log writes a timestamped CSV to data/radio/ recording, every frame, the raw
stick/switch values AND exactly what was sent to the drone. After a flight,
reconnect to normal WiFi and I can read that file directly off your Mac.

--telemetry ALSO reads navdata (attitude + per-motor PWM) over the SAME AT link
and folds it into that CSV, so each row pairs "what I sent" with "what the motors
did". It must share this one process because two AT senders fight over the command
sequence number; running navdata.py separately would knock this out of sync.

Test locally first (no drone, no WiFi) — two terminals:
    python3 scripts/control/fake_drone.py
    python3 scripts/control/tango_fly.py --host 127.0.0.1          # real sticks
    python3 scripts/control/tango_fly.py --host 127.0.0.1 --sim    # synthetic sticks

Real drone (join its WiFi, PROPS OFF for first tests):
    python3 scripts/control/tango_fly.py --host 192.168.1.1 --log
    python3 scripts/control/tango_fly.py --host 192.168.1.1 --telemetry --log   # + motor PWM
"""

import argparse
import csv
import datetime
import math
import os
import socket
import sys
import time

from at_link import Drone
from navdata import NAVDATA_PORT, has_options, parse_navdata, wake_navdata

# --- channel map (indices into the 8-axis list) ---------------------------
THROTTLE, ROLL, PITCH, YAW = 0, 1, 2, 3
SW_ARM, SW_TAKEOFF, SW_MODE, SW_EMERG = 4, 5, 6, 7

# --- stick shaping / direction -------------------------------------------
DEADBAND = 0.08      # ignore tiny center jitter
EXPO = 0.30          # 0 = linear, 1 = very soft center
RATE_HZ = 30
# Flip any of these to True if a control moves the wrong way on the real drone.
REVERSE_ROLL = False
REVERSE_PITCH = False
REVERSE_YAW = False
REVERSE_THROTTLE = False


def shape(v, reverse=False):
    """Deadband + expo, result in -1..1."""
    if reverse:
        v = -v
    if abs(v) < DEADBAND:
        return 0.0
    s = (abs(v) - DEADBAND) / (1.0 - DEADBAND)
    s = (1.0 - EXPO) * s + EXPO * s ** 3
    return math.copysign(s, v)


def bpos(v):
    return "TAKEOFF" if v > 0.5 else "LAND" if v < -0.5 else "hold"


class Logger:
    """Timestamped CSV under data/radio/ — raw sticks + what was actually sent."""

    COLS = ["t", "wall", "state", "action", "arm", "b", "emerg",
            "raw_thr", "raw_roll", "raw_pitch", "raw_yaw",
            "sent_roll", "sent_pitch", "sent_gaz", "sent_yaw"]
    TELE_COLS = ["roll_deg", "pitch_deg", "yaw_deg", "alt_mm", "m1", "m2", "m3", "m4"]

    def __init__(self, tag, telemetry=False):
        repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        d = os.path.join(repo, "data", "radio")
        os.makedirs(d, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(d, f"{tag}_{ts}.csv")
        self.f = open(self.path, "w", newline="")
        self.w = csv.writer(self.f)
        self.telemetry = telemetry
        self.w.writerow(self.COLS + (self.TELE_COLS if telemetry else []))
        self.t0 = time.monotonic()

    def row(self, state, act, ax, pcmd, nd=None):
        r = [f"{time.monotonic() - self.t0:.3f}", f"{time.time():.3f}", state, act,
             f"{ax[SW_ARM]:.2f}", f"{ax[SW_TAKEOFF]:.2f}", f"{ax[SW_EMERG]:.2f}",
             f"{ax[THROTTLE]:.3f}", f"{ax[ROLL]:.3f}", f"{ax[PITCH]:.3f}", f"{ax[YAW]:.3f}"]
        r += [f"{v:.3f}" for v in pcmd] if pcmd else ["", "", "", ""]
        if self.telemetry:
            d = (nd or {}).get("demo", {})
            m = (nd or {}).get("pwm") or ["", "", "", ""]
            r += [d.get("roll", ""), d.get("pitch", ""), d.get("yaw", ""), d.get("alt_mm", ""),
                  m[0], m[1], m[2], m[3]]
        self.w.writerow(r)

    def close(self):
        self.f.close()
        return self.path


class Bridge:
    SAFE, DISARMED, ARMED, EMERGENCY = "SAFE-LOCK", "DISARMED", "ARMED", "EMERGENCY"

    def __init__(self, drone, arm_high_at_start):
        self.drone = drone
        self.state = self.SAFE if arm_high_at_start else self.DISARMED
        self._tick = 0

    def update(self, ax):
        """Feed one frame of axis values; sends the right AT command.
        Returns (state, action_str, pcmd) where pcmd is (roll,pitch,gaz,yaw) or None."""
        arm = ax[SW_ARM] > 0.5
        emerg = ax[SW_EMERG] > 0.5
        b = ax[SW_TAKEOFF]

        # --- transitions ---
        if emerg:
            self.state = self.EMERGENCY
        elif self.state == self.EMERGENCY:
            self.state = self.SAFE if arm else self.DISARMED   # D released
        elif self.state == self.SAFE:
            if not arm:
                self.state = self.DISARMED                     # arm toggled off -> unlocked
        elif self.state == self.DISARMED:
            if arm:
                self.drone.flat_trim()                         # level reference while still
                self.state = self.ARMED
        elif self.state == self.ARMED:
            if not arm:
                self.drone.land()                              # disarm in flight -> land
                self.state = self.DISARMED

        # --- actions ---
        if self.state == self.EMERGENCY:
            self.drone.emergency()
            return self.state, "EMERGENCY CUT", None
        if self.state in (self.SAFE, self.DISARMED):
            self._tick += 1
            if self._tick % 15 == 0:
                self.drone.comwdg()                            # keep the link alive
            return self.state, "idle", None
        # ARMED
        if b > 0.5:
            self.drone.takeoff()
            return self.state, "TAKEOFF", None
        if b < -0.5:
            self.drone.land()
            return self.state, "LAND", None
        roll = shape(ax[ROLL], REVERSE_ROLL)
        pitch = shape(ax[PITCH], REVERSE_PITCH)
        yaw = shape(ax[YAW], REVERSE_YAW)
        gaz = shape(ax[THROTTLE], REVERSE_THROTTLE)
        self.drone.move(roll=roll, pitch=pitch, gaz=gaz, yaw=yaw)
        return self.state, f"move r{roll:+.2f} p{pitch:+.2f} g{gaz:+.2f} y{yaw:+.2f}", (roll, pitch, gaz, yaw)


def _drain_navdata(sock):
    """Non-blocking: return the most recent navdata packet that carries option blocks, else None.
    Called once per flight frame so a burst of queued packets never stalls the 30 Hz PCMD loop."""
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except (BlockingIOError, OSError):
            break
        nd = parse_navdata(data)
        if nd and has_options(nd):
            latest = nd
    return latest


def _start_telemetry(drone, host):
    """Open the navdata socket and wake attitude + motor PWM over the SAME AT link (`drone`), so the
    telemetry and flight commands share one sequence stream. Returns a non-blocking socket to drain."""
    ndsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ndsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ndsock.bind(("", NAVDATA_PORT))
    ndsock.settimeout(0.5)
    print("waking navdata (attitude + motor PWM) out of bootstrap...")
    status = wake_navdata(drone, ndsock, host, want_full=True)
    print({"full": "navdata live: attitude + motor PWM.\n",
           "demo": "navdata live: attitude only (no motor PWM yet).\n"}.get(
               status, "! navdata still bootstrapping; telemetry may be empty.\n"), end="")
    ndsock.settimeout(0.0)                                # non-blocking drain inside the flight loop
    return ndsock


def _tele_str(ndsock, last_nd):
    """Short ' M[..]' suffix for the status line when motor PWM is available."""
    if ndsock is not None and last_nd and last_nd.get("pwm"):
        m = last_nd["pwm"]
        return f"  M[{m[0]:3d} {m[1]:3d} {m[2]:3d} {m[3]:3d}]"
    return ""


def run_real(host, port, log, telemetry=False):
    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No joystick. Is the Tango in USB Joystick mode and plugged in?")
    js = pygame.joystick.Joystick(0)
    js.init()
    n = js.get_numaxes()
    print(f"Joystick: {js.get_name()}  ({n} axes)   ->  drone {host}:{port}")
    if log:
        print(f"Logging to {log.path}")
    print("A=arm  B=takeoff/land  D=EMERGENCY.  Ctrl-C to quit.\n")

    def read_axes():
        pygame.event.pump()
        return [js.get_axis(i) if i < n else 0.0 for i in range(8)]

    drone = Drone(host, port, verbose=False)

    last_nd = None
    ndsock = _start_telemetry(drone, host) if telemetry else None

    bridge = Bridge(drone, arm_high_at_start=read_axes()[SW_ARM] > 0.5)
    period = 1.0 / RATE_HZ
    try:
        while True:
            ax = read_axes()
            state, act, pcmd = bridge.update(ax)
            if ndsock is not None:
                nd = _drain_navdata(ndsock)
                if nd:
                    last_nd = nd
            if log:
                log.row(state, act, ax, pcmd, last_nd)
            sys.stdout.write("\r\033[2K" + f"[{state:<9}] B:{bpos(ax[SW_TAKEOFF]):<7} "
                             f"send: {act}{_tele_str(ndsock, last_nd)}")
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        drone.land()
        sys.stdout.write("\nlanded / exit\n")
    finally:
        if log:
            print(f"log saved: {log.close()}")
        pygame.quit()


def _axes(throttle=-1.0, roll=0.0, pitch=0.0, yaw=0.0, arm=-1.0, b=0.0, mode=0.0, emerg=-1.0):
    return [throttle, roll, pitch, yaw, arm, b, mode, emerg]


def run_sim(host, port, log):
    """Drive the same state machine with a scripted stick sequence (no pygame)."""
    segments = [
        ("startup (disarmed, throttle down)", 0.4, _axes(throttle=-1)),
        ("flip A -> arm",                     0.4, _axes(throttle=-1, arm=1)),
        ("B up -> takeoff",                   0.6, _axes(throttle=0.4, arm=1, b=1)),
        ("fly: pitch forward",                0.6, _axes(arm=1, throttle=0, pitch=-0.6)),
        ("fly: roll right + yaw right",       0.6, _axes(arm=1, throttle=0, roll=0.4, yaw=0.5)),
        ("fly: throttle up (climb)",          0.6, _axes(arm=1, throttle=0.7)),
        ("B down -> land",                    0.6, _axes(throttle=-1, arm=1, b=-1)),
        ("flip A -> disarm",                  0.4, _axes(throttle=-1, arm=-1)),
        ("D on -> EMERGENCY",                 0.4, _axes(emerg=1)),
    ]
    drone = Drone(host, port, verbose=False)
    bridge = Bridge(drone, arm_high_at_start=segments[0][2][SW_ARM] > 0.5)
    period = 1.0 / RATE_HZ
    print(f"SIM -> drone {host}:{port}" + (f"   logging to {log.path}" if log else "") + "\n")
    for label, secs, ax in segments:
        state = act = None
        end = time.monotonic() + secs
        while time.monotonic() < end:
            state, act, pcmd = bridge.update(ax)
            if log:
                log.row(state, act, ax, pcmd)
            time.sleep(period)
        print(f"  {label:<34} [{state:<9}] {act}")
    if log:
        print(f"\nlog saved: {log.close()}")
    print("sim done.")


def main():
    p = argparse.ArgumentParser(description="Fly the AR.Drone 2.0 with a TBS Tango 2")
    p.add_argument("--host", default="127.0.0.1", help="drone IP (192.168.1.1) or 127.0.0.1 to test")
    p.add_argument("--port", type=int, default=5556)
    p.add_argument("--sim", action="store_true", help="run a scripted stick sequence (no joystick)")
    p.add_argument("--log", action="store_true", help="write a timestamped CSV to data/radio/")
    p.add_argument("--telemetry", action="store_true",
                   help="also read navdata (attitude + motor PWM) over the same link and log it")
    a = p.parse_args()
    telemetry = a.telemetry and not a.sim          # navdata read needs the real drone, not the fake one
    log = Logger("sim" if a.sim else "flight", telemetry=telemetry) if a.log else None
    if a.sim:
        run_sim(a.host, a.port, log)
    else:
        run_real(a.host, a.port, log, telemetry)


if __name__ == "__main__":
    main()
