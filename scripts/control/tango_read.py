#!/usr/bin/env python3
"""Live readout of the TBS Tango 2 (or any HID gamepad) on the Mac.

This is Path A, piece 2: read the radio's sticks/switches as a USB joystick and
print them live in the terminal. It does NOT touch the drone — it's the input
half of the eventual joystick->AT bridge. Later we swap the print loop for an
AT*PCMD sender (see docs/system.md "Mac->drone flight-command loop").

Setup:
  - Tango 2 in USB Joystick (HID) mode, plugged into the Mac (see docs: Path A).
  - pip install pygame      (Python 3, pygame 2.x / SDL2)

Run:
  python3 scripts/control/tango_read.py

Use it to fill in AXIS_LABELS below: push one stick, see which row's bar moves,
note that axis index, and label it. That map is what the bridge will rely on.
Ctrl-C to quit.
"""

import sys
import time

try:
    import pygame
except ImportError:
    sys.exit("pygame not found  ->  pip install pygame")

# ----------------------------------------------------------------------------
# CONFIG — verify these against YOUR radio by wiggling each stick and watching
# which axis index moves. Defaults are a Mode-2 guess; correct them as needed.
# ----------------------------------------------------------------------------
AXIS_LABELS = {
    0: "throttle",
    1: "roll   (aileron)",
    2: "pitch  (elevator)",
    3: "yaw    (rudder)",
    4: "[SW A] arm",
    5: "[SW B] takeoff/land",
    6: "[SW C] flight mode",
    7: "[SW D] EMERGENCY",
}
REFRESH_HZ = 30          # ~ the rate the drone wants AT*PCMD later
BAR_WIDTH = 21           # odd number; center column is the zero mark


def bar(value):
    """Render value in [-1, 1] as a centered ASCII bar."""
    value = max(-1.0, min(1.0, value))
    center = BAR_WIDTH // 2
    pos = center + int(round(value * center))
    cells = []
    for i in range(BAR_WIDTH):
        if i == center:
            cells.append("|" if i != pos else "#")
        elif i == pos:
            cells.append("#")
        else:
            cells.append("-")
    return "".join(cells)


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        sys.exit("No joystick detected. Is the Tango in USB Joystick mode and plugged in?")

    js = pygame.joystick.Joystick(0)
    js.init()
    name = js.get_name()
    n_axes = js.get_numaxes()
    n_btns = js.get_numbuttons()
    n_hats = js.get_numhats()

    header = [
        f"Device : {name}",
        f"Axes {n_axes}  Buttons {n_btns}  Hats {n_hats}   (Ctrl-C to quit)",
        "-" * (BAR_WIDTH + 30),
    ]
    for line in header:
        print(line)

    period = 1.0 / REFRESH_HZ
    block_lines = n_axes + 2  # axes + buttons row + hats row
    first = True
    try:
        while True:
            pygame.event.pump()  # refresh joystick state (needed every frame)

            lines = []
            for a in range(n_axes):
                v = js.get_axis(a)
                label = AXIS_LABELS.get(a, f"axis {a}")
                lines.append(f"{a:>2} {label:<18} {v:+0.2f} [{bar(v)}]")

            btns = "".join("#" if js.get_button(b) else "." for b in range(n_btns))
            lines.append(f"   buttons{'':<13} [{btns}]")

            hats = " ".join(str(js.get_hat(h)) for h in range(n_hats)) or "(none)"
            lines.append(f"   hats{'':<16} {hats}")

            buf = ""
            if not first:
                buf += f"\033[{block_lines}A"  # jump back to top of the block
            for line in lines:
                buf += "\033[2K" + line + "\n"  # clear line, write, newline
            sys.stdout.write(buf)
            sys.stdout.flush()
            first = False

            time.sleep(period)
    except KeyboardInterrupt:
        sys.stdout.write("\n")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
