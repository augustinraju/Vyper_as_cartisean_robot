"""G-code sequences for the Vyper gantry gripper.

Each function returns the list of lines to send, in order. No serial or GUI code,
so scripts (e.g. a path planner) can reuse them with PrinterConnection.send_lines().
"""

from __future__ import annotations

from typing import Mapping

from .config import FEEDRATE

ENABLE_ENDSTOPS = "M120"  # check endstops on every move, not only during G28
REPORT_POSITION = "M114"
WAIT_FOR_MOVES = "M400"  # "ok" only once all planned moves have finished


def strip_comment(line: str) -> str:
    """Drop everything after ';' (G-code comment) and surrounding whitespace."""
    return line.split(";", 1)[0].strip()


def non_ascii_chars(line: str) -> str:
    """The distinct non-ASCII characters in line (empty if it is valid G-code text)."""
    return "".join(sorted({ch for ch in line if not ch.isascii()}))


def jog(axis: str, distance: float, feedrate: int = FEEDRATE) -> list[str]:
    """Relative move on one axis, then back to absolute positioning.

    With X_DUAL_STEPPER_DRIVERS (M606 S0), one G1 X moves X1 and X2 together.
    """
    return ["G91", f"G1 {axis}{distance:.2f} F{feedrate}", "G90"]


def jog_x_motor(motor: int, distance: float, feedrate: int = FEEDRATE) -> list[str]:
    """Move one X motor only. motor: 1 = X1, 2 = X2 (firmware M606 S1 X1|X2)."""
    if motor not in (1, 2):
        raise ValueError(f"X motor must be 1 or 2, got {motor}")
    return [
        f"M606 S1 X{motor}",  # lock the other motor
        *jog("X", distance, feedrate),
        "M606 S0",  # back to both motors; waits for the move to finish
    ]


GRIPPER_MODE = "M606 S2"  # both X motors at once, opposite directions; G1 X = half the gap
BOTH_X_MODE = "M606 S0"  # both X motors together; G1 X = gripper midpoint


def change_gap(delta: float, feedrate: int = FEEDRATE) -> list[str]:
    """Open (delta > 0) or close (delta < 0) the gripper by delta mm, keeping its midpoint.

    Both jaws move at the same time, delta/2 each, at `feedrate` per jaw.
    """
    return [GRIPPER_MODE, *jog("X", delta / 2, feedrate), BOTH_X_MODE]


def set_gap(gap: float, feedrate: int = FEEDRATE) -> list[str]:
    """Move both jaws at once to the given gap (mm), keeping the midpoint."""
    return [GRIPPER_MODE, "G90", f"G1 X{gap / 2:.3f} F{feedrate}", BOTH_X_MODE, REPORT_POSITION]


def home(axis: str) -> list[str]:
    """Home one axis. X1 homes to X min, X2 to X max; the other X motor stays put."""
    if axis == "X1":
        return [
            "M606 S1 X1",  # lock X2
            "G28 X",  # X1 to its switch at X min
            "M606 S0",  # queued even if the homing fails, so no motor stays locked
        ]
    if axis == "X2":
        return ["M606 H2"]  # firmware locks X1, homes X2 to X max, ends in M606 S0
    return [f"G28 {axis}"]


def goto(targets: Mapping[str, float], feedrate: int = FEEDRATE) -> list[str]:
    """Absolute move to the given axes (missing axis = unchanged), then report.

    In M606 S0 the firmware's logical X is the gripper midpoint, so G1 X<target>
    moves both motors and keeps the X1/X2 gap.
    """
    if not targets:
        return []
    words = " ".join(f"{axis}{value:.2f}" for axis, value in targets.items())
    return ["G90", f"G1 {words} F{feedrate}", REPORT_POSITION]


def set_zero() -> list[str]:
    """Declare the current physical position as X0 Y0 Z0 (no motion).

    Also resets the X1/X2 counters in the firmware.
    """
    return ["G92 X0 Y0 Z0", REPORT_POSITION]
