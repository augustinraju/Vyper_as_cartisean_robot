"""Send waypoints to the printer one at a time, checking each one against the printer.

Per waypoint:
    1. G-code from pose_to_gcode (gap change if needed, G1, M400, M114)
    2. wait until every line is acknowledged and the move has finished (M400)
    3. read the M114 report (X1, X2, Y, Z) and compare it with the target
Stops with WaypointError on a fault (endstop hit, firmware error, rejected M606), a
position mismatch (e.g. the firmware clamped the move to a soft limit), a missing
report, a timeout or a lost connection. The connection drops the lines still queued
when a fault arrives, so nothing after the failing waypoint is executed.

Every waypoint (and every gap change at the position where it happens) is range-checked
before the first line is sent. Endstops are enabled (M120) at the start of each run.

Usage (owns the connection's rx_queue: do not run it next to the GUI on one connection):
    import queue
    from waypoint_sender import PrinterConnection, WaypointSender
    rx = queue.Queue()
    link = PrinterConnection(rx)
    link.connect("/dev/ttyUSB0")
    sender = WaypointSender(link, rx)
    sender.run_sim_path(path, gap_m=0.03)      # planner path in metres
"""

from __future__ import annotations

import os
import queue
import sys
import time
from typing import Callable, Iterable, Optional, Sequence

from pose_to_gcode import FEEDRATE, poses_to_gcode
from sim_to_printer import DEFAULT, PrinterPose, SimToPrinter

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from RoboCup.Controller.vyper_host.connection import PrinterConnection  # noqa: E402

ENABLE_ENDSTOPS = "M120"
REPORT_POSITION = "M114"


class WaypointError(RuntimeError):
    """The run was stopped; `index` is the failing waypoint (None = before the first)."""

    def __init__(self, message: str, index: Optional[int] = None):
        super().__init__(message if index is None else f"waypoint {index}: {message}")
        self.index = index


class WaypointSender:
    def __init__(self, link: PrinterConnection, rx_queue: queue.Queue,
                 transform: SimToPrinter = DEFAULT, feedrate: int = FEEDRATE,
                 gap_feedrate: Optional[int] = None, tol_mm: float = 0.05,
                 timeout_s: float = 120.0, log: Callable[[str], None] = print):
        """tol_mm: allowed |reported - target| per axis (M114 prints 2 decimals).
        timeout_s: longest time one waypoint may take (sending + motion)."""
        self.link, self.rx = link, rx_queue
        self.transform = transform
        self.feedrate, self.gap_feedrate = feedrate, gap_feedrate
        self.tol_mm, self.timeout_s = tol_mm, timeout_s
        self.log = log

    # ---- public API ------------------------------------------------------------------
    def read_pose(self, index: Optional[int] = None) -> PrinterPose:
        """Current printer pose from M114 (waits for the printer to be free)."""
        return self._execute([REPORT_POSITION], index)

    def home(self, timeout_s: float = 300.0) -> PrinterPose:
        """G28 (X1 to min, X2 to max, Y, Z), then the pose it ends at."""
        return self._execute(["G28", REPORT_POSITION], None, timeout_s=timeout_s)

    def run(self, poses: Iterable[PrinterPose]) -> list[PrinterPose]:
        """Move through the waypoints; returns the pose reported after each one."""
        poses = list(poses)
        self._execute([ENABLE_ENDSTOPS], None, want_pose=False)
        current = self.read_pose()
        # Dry run: every waypoint and gap change is checked before anything moves
        try:
            poses_to_gcode(poses, current=current, transform=self.transform)
        except ValueError as exc:
            raise WaypointError(f"not started: {exc}") from None

        reached = []
        for i, pose in enumerate(poses):
            lines = poses_to_gcode([pose], self.feedrate, self.gap_feedrate,
                                   current=current, transform=self.transform)
            reported = self._execute(lines, i)
            self._verify(i, pose, reported)
            self.log(f"waypoint {i + 1}/{len(poses)} reached: X1={reported.x1:.2f} "
                     f"X2={reported.x2:.2f} Y={reported.y:.2f} Z={reported.z:.2f}")
            reached.append(reported)
            current = reported
        return reached

    def run_sim_path(self, path: Iterable[Sequence[float]], gap_m: float,
                     goal_gap_m: Optional[float] = None) -> list[PrinterPose]:
        """Planner path [(x, y, z), ...] [m] with a fixed gap [m] (use the smoothed
        waypoints from GripperPlanner.plan(), not densify()). With goal_gap_m, the
        jaws then open/close to that gap at the last waypoint."""
        try:
            path = [tuple(q) for q in path]
            poses = self.transform.path_to_printer(path, gap_m)
            if goal_gap_m is not None and abs(goal_gap_m - gap_m) > 1e-9:
                # same position, new gap -> poses_to_gcode emits only the gap change
                poses.append(self.transform.conf_to_printer(path[-1], goal_gap_m))
        except ValueError as exc:
            raise WaypointError(f"not started: {exc}") from None
        return self.run(poses)

    # ---- internals -------------------------------------------------------------------
    def _execute(self, lines: list[str], index: Optional[int], want_pose: bool = True,
                 timeout_s: Optional[float] = None) -> Optional[PrinterPose]:
        """Send one batch, wait until it is done, return the M114 pose it reported."""
        timeout_s = self.timeout_s if timeout_s is None else timeout_s
        if not self.link.connected:
            raise WaypointError("not connected", index)
        if self.link.busy:
            raise WaypointError("printer busy with other commands", index)
        self._handle_backlog()
        self.link.send_lines(lines)

        pos = dual = fault = None
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                item = self.rx.get(timeout=0.1)
            except queue.Empty:
                if not self.link.busy:
                    break  # all lines acked; their replies were queued before the acks
                if time.monotonic() > deadline:
                    raise WaypointError(f"no completion within {timeout_s:g} s", index)
                continue
            if not isinstance(item, tuple):
                self.log(item)
                continue
            kind, *values = item
            if kind == "fault":
                fault = fault or values[0]
            elif kind == "pos":
                pos = values
            elif kind == "dualx":
                dual = values
            elif kind == "lost":
                raise WaypointError(f"connection lost: {values[1]}", index)

        if fault:
            raise WaypointError(f"printer fault: {fault}", index)
        if not want_pose:
            return None
        if pos is None or dual is None:
            raise WaypointError("no position report (M114 with X1/X2) received", index)
        (x1, x2), (_, y, z) = dual, pos
        return PrinterPose((x1 + x2) / 2, y, z, x2 - x1)

    def _handle_backlog(self) -> None:
        """Log messages left over from before this batch (they belong to no waypoint)."""
        while True:
            try:
                item = self.rx.get_nowait()
            except queue.Empty:
                return
            if not isinstance(item, tuple):
                self.log(item)
            elif item[0] == "lost":
                raise WaypointError(f"connection lost: {item[2]}")

    def _verify(self, i: int, target: PrinterPose, got: PrinterPose) -> None:
        errors = [
            f"{name} {g:.3f} != {t:.3f}"
            for name, t, g in (("X1", target.x1, got.x1), ("X2", target.x2, got.x2),
                               ("Y", target.y, got.y), ("Z", target.z, got.z))
            if abs(g - t) > self.tol_mm
        ]
        if errors:
            raise WaypointError("position mismatch (soft limit clamp or lost steps?): "
                                + ", ".join(errors), i)
