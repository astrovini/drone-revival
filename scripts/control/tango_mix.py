#!/usr/bin/env python3
"""Stream TBS Tango 2 (or keyboard) stick positions to the onboard `manualmix` motor mixer.

This is the Mac half of the PROPS-OFF bench mixer test. It does NOT use the stock firmware or
AT commands — it sends raw stick values over UDP to `scripts/motors/manualmix.c` running on the
drone, which arms the motors to idle and mixes the sticks straight to the four motors.

    ⚠ NO STABILIZATION — bench test only, props off. Not flight. See docs/control.md.

Packet (ASCII, ~50 Hz):   "M <arm> <emerg> <thr01> <roll> <pitch> <yaw>\\n"
  arm,emerg in {0,1};  thr01 in [0,1] (throttle stick bottom->0, top->1);  roll/pitch/yaw in [-1,1].

Run (drone WiFi, manualmix already running on the drone):
    python3 scripts/control/tango_mix.py --host 192.168.1.1                 # Tango
    python3 scripts/control/tango_mix.py --host 192.168.1.1 --keyboard      # laptop keyboard
Offline check (no drone, no joystick):
    python3 scripts/control/tango_mix.py --sim                              # prints the packets it would send

Tango: A(4)=arm  D(7)=EMERGENCY  left-V(0)=throttle  right-H(1)=roll  right-V(2)=pitch  left-H(3)=yaw
Keyboard: W/S=throttle up/down (held)  A/D=yaw  arrows=pitch/roll  Enter=arm  Space=EMERGENCY  Esc=quit
"""

import argparse
import math
import socket
import sys
import time

# indices into the 8-axis Tango vector (same convention as tango_fly.py)
THROTTLE, ROLL, PITCH, YAW = 0, 1, 2, 3
SW_ARM, SW_EMERG = 4, 7

DEADBAND = 0.08
EXPO = 0.30
# Flip if a control drives the wrong motors on the bench.
REVERSE_ROLL = False
REVERSE_PITCH = False
REVERSE_YAW = False
REVERSE_THROTTLE = False
KB_DEFLECT = 0.6
KB_THR_RATE = 0.5      # keyboard throttle change per second while W/S held

KB_HELP = [
    "manualmix keyboard control — PROPS OFF (no stabilization, bench test)",
    "  W/S : throttle up / down (held; ramps)",
    "  A/D : yaw left / right    arrows: pitch (UP/DN) / roll (L/R)",
    "  Enter: ARM / disarm  (throttle must be 0 to arm)",
    "  Space: EMERGENCY cut (toggle)   Shift: full deflection   Esc/Q: quit",
]


def shape(v, reverse=False):
    """Deadband + expo -> [-1,1]."""
    if reverse:
        v = -v
    if abs(v) < DEADBAND:
        return 0.0
    s = (abs(v) - DEADBAND) / (1.0 - DEADBAND)
    s = (1.0 - EXPO) * s + EXPO * s ** 3
    return math.copysign(s, v)


def pack(arm, emerg, thr, roll, pitch, yaw):
    thr = max(0.0, min(1.0, thr))
    return f"M {1 if arm else 0} {1 if emerg else 0} {thr:.3f} {roll:.3f} {pitch:.3f} {yaw:.3f}\n".encode()


class Link:
    """UDP sender (or dry-run printer)."""
    def __init__(self, host, port, dry):
        self.addr = (host, port)
        self.dry = dry
        self.sock = None if dry else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, pkt):
        if self.dry:
            sys.stdout.write("\r\033[2K" + pkt.decode().strip())
            sys.stdout.flush()
        else:
            self.sock.sendto(pkt, self.addr)

    def disarm(self):
        for _ in range(5):
            self.send(pack(False, False, 0, 0, 0, 0))
            time.sleep(0.01)


def run_tango(link, hz):
    import pygame
    pygame.init(); pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("No joystick. Is the Tango in USB Joystick mode and plugged in?")
    js = pygame.joystick.Joystick(0); js.init()
    n = js.get_numaxes()
    print(f"Tango: {js.get_name()} ({n} axes) -> manualmix {link.addr}")
    print("A=arm  D=EMERGENCY.  Throttle stick controls motor power.  Ctrl-C to quit.\n")

    def axes():
        pygame.event.pump()
        return [js.get_axis(i) if i < n else 0.0 for i in range(8)]

    # SAFE-LOCK: if arm is already up at startup, refuse to arm until it's toggled off once.
    safe_locked = axes()[SW_ARM] > 0.5
    if safe_locked:
        print("SAFE-LOCK: arm switch is UP — flip it DOWN once to unlock.\n")
    period = 1.0 / hz
    try:
        while True:
            ax = axes()
            arm = ax[SW_ARM] > 0.5
            emerg = ax[SW_EMERG] > 0.5
            if safe_locked:
                if not arm:
                    safe_locked = False
                arm = False
            raw_thr = -ax[THROTTLE] if REVERSE_THROTTLE else ax[THROTTLE]
            thr = max(0.0, min(1.0, (raw_thr + 1.0) / 2.0))   # stick bottom(-1)->0, top(+1)->1
            roll = shape(ax[ROLL], REVERSE_ROLL)
            pitch = shape(ax[PITCH], REVERSE_PITCH)
            yaw = shape(ax[YAW], REVERSE_YAW)
            link.send(pack(arm, emerg, thr, roll, pitch, yaw))
            lock = " [SAFE-LOCK]" if safe_locked else ""
            sys.stdout.write("\r\033[2K" + f"arm:{'ON ' if arm else 'off'} emg:{'!' if emerg else '.'} "
                             f"thr:{thr:.2f} r{roll:+.2f} p{pitch:+.2f} y{yaw:+.2f}{lock}")
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        link.disarm()
        print("\ndisarmed / exit")
        pygame.quit()


def run_keyboard(link, hz):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((600, 220))
    pygame.display.set_caption("AR.Drone manualmix (keyboard)")
    font = pygame.font.SysFont("Menlo", 15)
    clock = pygame.time.Clock()
    print(f"Keyboard -> manualmix {link.addr}   (click the pop-up to give it focus)")
    for line in KB_HELP:
        print("  " + line)
    print()

    arm = emerg = False
    thr = 0.0

    def draw(roll, pitch, yaw):
        screen.fill((16, 18, 22))
        y = 8
        for line in KB_HELP:
            screen.blit(font.render(line, True, (150, 160, 170)), (10, y)); y += 18
        y += 8
        st = "EMERGENCY" if emerg else ("ARMED" if arm else "disarmed")
        col = (240, 120, 120) if emerg else ((240, 240, 120) if arm else (150, 160, 170))
        screen.blit(font.render(f"state: {st}     throttle: {thr:.2f}", True, col), (10, y)); y += 20
        screen.blit(font.render(f"cmd:   yaw {yaw:+.2f}   pitch {pitch:+.2f}   roll {roll:+.2f}",
                                True, (120, 220, 160)), (10, y))
        pygame.display.flip()

    running = True
    try:
        while running:
            dt = clock.tick(hz) / 1000.0
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_RETURN:
                        arm = not arm
                    elif e.key == pygame.K_SPACE:
                        emerg = not emerg
                        if emerg:
                            arm = False
                    elif e.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
            k = pygame.key.get_pressed()
            mag = 1.0 if (k[pygame.K_LSHIFT] or k[pygame.K_RSHIFT]) else KB_DEFLECT
            if k[pygame.K_w]:
                thr = min(1.0, thr + KB_THR_RATE * dt)
            if k[pygame.K_s]:
                thr = max(0.0, thr - KB_THR_RATE * dt)
            yaw = (mag if k[pygame.K_d] else 0.0) - (mag if k[pygame.K_a] else 0.0)
            pitch = (mag if k[pygame.K_UP] else 0.0) - (mag if k[pygame.K_DOWN] else 0.0)
            roll = (mag if k[pygame.K_RIGHT] else 0.0) - (mag if k[pygame.K_LEFT] else 0.0)
            link.send(pack(arm, emerg, thr, roll, pitch, yaw))
            draw(roll, pitch, yaw)
            st = "EMERG" if emerg else "ARMED" if arm else "disarmed"
            sys.stdout.write("\r\033[2K" + f"[{st:<9}] thr:{thr:.2f} r{roll:+.2f} p{pitch:+.2f} y{yaw:+.2f}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        link.disarm()
        print("\ndisarmed / exit")
        pygame.quit()


def run_sim(link):
    """Offline: push a scripted sequence through the packer so the format/mapping can be checked."""
    seq = [
        ("disarmed, throttle low", (False, False, 0.0, 0, 0, 0)),
        ("arm (throttle low)",     (True, False, 0.0, 0, 0, 0)),
        ("throttle up",            (True, False, 0.4, 0, 0, 0)),
        ("roll right",             (True, False, 0.4, 0.6, 0, 0)),
        ("pitch fwd + yaw",        (True, False, 0.4, 0, -0.5, 0.5)),
        ("EMERGENCY",              (True, True, 0.4, 0, 0, 0)),
        ("disarm",                 (False, False, 0.0, 0, 0, 0)),
    ]
    print("SIM — packets that would be sent:")
    for label, a in seq:
        pkt = pack(*a)
        print(f"  {label:<24} {pkt.decode().strip()}")
        if not link.dry:
            link.send(pkt)
        time.sleep(0.2)
    print("sim done.")


def main():
    p = argparse.ArgumentParser(description="Stream Tango/keyboard sticks to the onboard manualmix mixer")
    p.add_argument("--host", default="127.0.0.1", help="drone IP (192.168.1.1)")
    p.add_argument("--port", type=int, default=5560)
    p.add_argument("--hz", type=int, default=50)
    p.add_argument("--keyboard", action="store_true", help="use the laptop keyboard instead of the Tango")
    p.add_argument("--sim", action="store_true", help="offline: print the packet sequence, don't require a joystick")
    p.add_argument("--print", dest="dry", action="store_true", help="dry-run: print packets instead of sending")
    a = p.parse_args()
    link = Link(a.host, a.port, dry=a.dry or a.sim)
    if a.sim:
        run_sim(link)
    elif a.keyboard:
        run_keyboard(link, a.hz)
    else:
        run_tango(link, a.hz)


if __name__ == "__main__":
    main()
