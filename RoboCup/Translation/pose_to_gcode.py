"""Printer coordinates (PrinterPose, mm) -> G-code lines for the Vyper gantry gripper.

Follows the firmware conventions used by RoboCup/Controller/vyper_host/commands.py:
    M606 S0 : both X motors together; G1 X = jaw midpoint, the gap is kept
    M606 S2 : both X motors mirrored;  G1 X = half the gap, the midpoint is kept
Output is a plain list of lines, ready for PrinterConnection.send_lines().

When a waypoint changes both the position and the gap, the gap is changed first (at
the previous position), then the move is made. The new gap is range-checked at that
previous position, where M606 S2 actually runs (the firmware would otherwise clamp the
opening silently). To grasp at a point, give two poses with the same X/Y/Z: the open
one, then the closed one.
"""

from __future__ import annotations

from typing import Iterable, Optional, Sequence

from sim_to_printer import DEFAULT, PrinterPose, SimToPrinter

FEEDRATE = 1200  # mm/min, same default as the controller (vyper_host/config.py)
DECIMALS = 3     # X is 400 steps/mm -> 0.0025 mm resolution
EPS_MM = 10 ** -DECIMALS / 2

BOTH_X_MODE = "M606 S0"
GRIPPER_MODE = "M606 S2"
ABSOLUTE = "G90"
WAIT_FOR_MOVES = "M400"
REPORT_POSITION = "M114"


def _num(v: float) -> str:
    s = f"{v:.{DECIMALS}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def speed_to_feedrate(speed_m_s: float) -> int:
    """Simulation speed [m/s] -> G-code feedrate F [mm/min]."""
    return max(1, round(speed_m_s * 1000 * 60))


def set_gap(gap_mm: float, feedrate: int = FEEDRATE) -> list[str]:
    """Open/close both jaws to gap_mm (X2 - X1), keeping the midpoint. Ends in M606 S0."""
    return [GRIPPER_MODE, ABSOLUTE, f"G1 X{_num(gap_mm / 2)} F{feedrate}", BOTH_X_MODE]


def move_to(pose: PrinterPose, feedrate: int = FEEDRATE) -> list[str]:
    """Absolute move of the jaw midpoint to pose X/Y/Z (M606 S0, gap unchanged)."""
    return [BOTH_X_MODE, ABSOLUTE, _g1(pose, feedrate)]


def _g1(pose: PrinterPose, feedrate: int) -> str:
    return f"G1 X{_num(pose.x)} Y{_num(pose.y)} Z{_num(pose.z)} F{feedrate}"


def _same(a: float, b: Optional[float]) -> bool:
    return b is not None and abs(a - b) < EPS_MM


def poses_to_gcode(poses: Iterable[PrinterPose], feedrate: int = FEEDRATE,
                   gap_feedrate: Optional[int] = None,
                   current: Optional[PrinterPose] = None,
                   transform: SimToPrinter = DEFAULT,
                   wait: bool = True, report: bool = True) -> list[str]:
    """Waypoints -> G-code. Raises ValueError (before any line is produced) if a
    waypoint, or a gap change at the position where it happens, breaks a firmware limit.

    current: the printer's pose (from M114) before the first waypoint. If None, the
    position and gap are unknown: the first waypoint's gap is then set after moving
    there, at a position that has been checked. Consecutive duplicate positions are skipped.
    wait: end with M400 (all moves finished). report: end with M114.
    """
    gap_feedrate = feedrate if gap_feedrate is None else gap_feedrate
    lines = [BOTH_X_MODE, ABSOLUTE]
    gap = current.gap if current else None
    last = (current.x, current.y, current.z) if current else None
    for i, pose in enumerate(poses):
        transform.check_pose(pose)
        xyz = (pose.x, pose.y, pose.z)
        moves = last is None or not all(_same(a, b) for a, b in zip(xyz, last))
        if not _same(pose.gap, gap):
            if last is None:  # position unknown: move first, then set the gap here
                lines.append(_g1(pose, feedrate))
                last, moves = xyz, False
            try:
                transform.check_pose(PrinterPose(*last, pose.gap))
            except ValueError as exc:
                raise ValueError(f"waypoint {i}: gap {pose.gap:.3f} mm cannot be set at "
                                 f"X={last[0]:.3f} before the move: {exc}") from None
            lines += set_gap(pose.gap, gap_feedrate)  # leaves M606 S0 + G90
            gap = pose.gap
        if moves:
            lines.append(_g1(pose, feedrate))
            last = xyz
    if wait:
        lines.append(WAIT_FOR_MOVES)
    if report:
        lines.append(REPORT_POSITION)
    return lines


def sim_path_to_gcode(path: Iterable[Sequence[float]], gap_m: float,
                      transform: SimToPrinter = DEFAULT, **kwargs) -> list[str]:
    """Planner path [(x, y, z), ...] [m] with a fixed gap [m] -> G-code.

    Positions are converted and range-checked by `transform` first, so an out-of-range
    waypoint raises ValueError before any G-code is produced. kwargs go to poses_to_gcode().
    """
    return poses_to_gcode(transform.path_to_printer(path, gap_m), transform=transform, **kwargs)
