"""Coordinate transforms between the PyBullet simulation and the Vyper printer controller.

Only geometry lives here (no pybullet, no serial), so the simulation, the planner and
the controller can all import it.

Simulation  (vyper.urdf / plan_gripper_move.py) -- metres, joint values
    x1_axis, x2_axis : jaw inner faces on the X beam; 0 = beam centre. Both joints point
                       along world -X (world x = -joint). Named like the printer:
                       x1_axis = right jaw (world +X side), x2_axis = left jaw, so
                       x1_axis <= x2_axis
    y_axis           : bed displacement along +Y; 0 = bed centred under the gantry
    z_axis           : gantry height; 0 = finger undersides ~2.5 mm above the bed
    planner conf     : (x, y, z) with x = jaw midpoint (joint value), plus a fixed
                       gap = x2_axis - x1_axis

Printer     (Marlin + M606 firmware) -- millimetres, logical positions
    X1, X2 : per-motor X positions (M114 "X1:" / "X2:"); X1 switch = 0, X2 switch = 181
    X      : jaw midpoint (what G1 X moves in M606 S0)
    gap    : X2 - X1 (M606 S2: G1 X = gap / 2)
    Y, Z   : logical Y / Z

Every axis is an affine map:   printer_mm = sign * 1000 * sim_m + offset_mm
The defaults below are first guesses from the URDF and Configuration*.h; calibrate
them on the machine (see the CALIBRATE notes) before trusting them for real moves.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, NamedTuple, Sequence

from marlin_config import numeric_defines

M_TO_MM = 1000.0

# ---- firmware limits, read from Marlin/Configuration.h and Configuration_adv.h -------
_FW = numeric_defines(
    "X_MIN_POS",            # X1 switch
    "X_GRIPPER_HOME_GAP",   # X2 switch = X_MIN_POS + X_GRIPPER_HOME_GAP
    "X_GRIPPER_MIN_GAP",    # smallest X2 - X1 the firmware allows
    "Y_MIN_POS", "Y_MAX_POS",
    "Z_MIN_POS", "Z_MAX_POS",
)
X_MIN_POS = _FW["X_MIN_POS"]
X_GRIPPER_HOME_GAP = _FW["X_GRIPPER_HOME_GAP"]
X_GRIPPER_MIN_GAP = _FW["X_GRIPPER_MIN_GAP"]
Y_MIN_POS, Y_MAX_POS = _FW["Y_MIN_POS"], _FW["Y_MAX_POS"]
Z_MIN_POS, Z_MAX_POS = _FW["Z_MIN_POS"], _FW["Z_MAX_POS"]


@dataclass(frozen=True)
class AxisMap:
    """One axis: printer_mm = sign * 1000 * sim_m + offset_mm, valid in [min_mm, max_mm]."""

    name: str
    sign: float
    offset_mm: float
    min_mm: float
    max_mm: float

    def to_printer(self, sim_m: float) -> float:
        return self.sign * M_TO_MM * sim_m + self.offset_mm

    def to_sim(self, printer_mm: float) -> float:
        return (printer_mm - self.offset_mm) / (self.sign * M_TO_MM)

    def check(self, printer_mm: float, tol: float = 1e-6) -> None:
        if not self.min_mm - tol <= printer_mm <= self.max_mm + tol:
            raise ValueError(
                f"{self.name}={printer_mm:.3f} mm outside printer range "
                f"[{self.min_mm:g}, {self.max_mm:g}]"
            )


# ---- default calibration ---------------------------------------------------------------
# X: sim beam centre (jaws closed, 0) = midpoint between the two X switches. X1 homes to
#    printer X min on the world +X (right) side, so printer X runs along world -X. The
#    URDF X joints point along world -X too, so joint values follow printer X: sign +1.
#    CALIBRATE: after G28 the jaw faces should sit at sim x1_axis = -0.0905 (world
#    x = +0.0905, right), x2_axis = +0.0905 (world x = -0.0905, left).
X_MAP = AxisMap("X", +1.0, X_MIN_POS + X_GRIPPER_HOME_GAP / 2, X_MIN_POS, X_MIN_POS + X_GRIPPER_HOME_GAP)
# Y: sim bed centred (0) = middle of the printer's Y soft range (127.5 mm).
#    CALIBRATE: offset (printer Y with the bed centred under the gripper) and sign
#    (flip to -1 if printer +Y moves the bed towards sim -Y).
Y_MAP = AxisMap("Y", +1.0, (Y_MIN_POS + Y_MAX_POS) / 2, Y_MIN_POS, Y_MAX_POS)
# Z: sim gantry at 0 (fingers just above the bed) = printer Z_MIN_POS.
#    CALIBRATE: offset = printer Z where the fingers just clear the bed.
Z_MAP = AxisMap("Z", +1.0, Z_MIN_POS, Z_MIN_POS, Z_MAX_POS)


class PrinterPose(NamedTuple):
    """Printer target in mm: jaw midpoint X, Y, Z and the jaw gap (X2 - X1)."""

    x: float
    y: float
    z: float
    gap: float

    @property
    def x1(self) -> float:
        return self.x - self.gap / 2

    @property
    def x2(self) -> float:
        return self.x + self.gap / 2

    def axes(self) -> dict[str, float]:
        """{'X', 'Y', 'Z'} for commands.goto() (M606 S0: X is the midpoint)."""
        return {"X": self.x, "Y": self.y, "Z": self.z}


class SimToPrinter:
    """Simulation (metres, joint values) <-> printer controller (mm, logical axes)."""

    def __init__(self, x: AxisMap = X_MAP, y: AxisMap = Y_MAP, z: AxisMap = Z_MAP,
                 min_gap_mm: float = X_GRIPPER_MIN_GAP):
        self.x, self.y, self.z = x, y, z
        self.min_gap_mm = min_gap_mm

    # ---- simulation -> printer -----------------------------------------------------
    def jaws_to_printer(self, x1_m: float, x2_m: float, check: bool = True) -> tuple[float, float]:
        """Sim jaw joints (x1_axis, x2_axis) -> printer (X1, X2) in mm, name for name.

        With check, crossed jaws (X2 - X1 below the minimum gap) raise ValueError.
        """
        x1, x2 = self.x.to_printer(x1_m), self.x.to_printer(x2_m)
        if check:
            self.x.check(x1)
            self.x.check(x2)
            self._check_gap(x2 - x1)
        return x1, x2

    def gap_to_printer(self, gap_m: float) -> float:
        """Sim jaw gap (x2 - x1) -> printer gap (X2 - X1) in mm."""
        return abs(self.x.sign) * M_TO_MM * gap_m

    def conf_to_printer(self, q: Sequence[float], gap_m: float, check: bool = True) -> PrinterPose:
        """Planner conf (x_mid, y, z) [m] + gap [m] -> PrinterPose [mm]."""
        x_m, y_m, z_m = q
        lo_m, hi_m = x_m - gap_m / 2, x_m + gap_m / 2  # joint values, lo <= hi
        x1, x2 = self.jaws_to_printer(*self._order_to_names(lo_m, hi_m), check)
        pose = PrinterPose((x1 + x2) / 2, self.y.to_printer(y_m), self.z.to_printer(z_m), x2 - x1)
        if check:
            self.y.check(pose.y)
            self.z.check(pose.z)
        return pose

    def joints_to_printer(self, x1_m: float, x2_m: float, y_m: float, z_m: float,
                          check: bool = True) -> PrinterPose:
        """Sim joint values (x1_axis, x2_axis, y_axis, z_axis) -> PrinterPose [mm]."""
        x1, x2 = self.jaws_to_printer(x1_m, x2_m, check)
        pose = PrinterPose((x1 + x2) / 2, self.y.to_printer(y_m), self.z.to_printer(z_m), x2 - x1)
        if check:
            self.y.check(pose.y)
            self.z.check(pose.z)
        return pose

    def path_to_printer(self, path: Iterable[Sequence[float]], gap_m: float,
                        check: bool = True) -> list[PrinterPose]:
        """Planner path [(x, y, z), ...] with a fixed gap -> printer waypoints."""
        return [self.conf_to_printer(q, gap_m, check) for q in path]

    # ---- printer -> simulation (e.g. mirror M114 feedback in the sim) ---------------
    def printer_to_joints(self, x1_mm: float, x2_mm: float, y_mm: float,
                          z_mm: float) -> tuple[float, float, float, float]:
        """Printer (X1, X2, Y, Z) [mm] -> sim (x1_axis, x2_axis, y_axis, z_axis) [m]."""
        return self.x.to_sim(x1_mm), self.x.to_sim(x2_mm), self.y.to_sim(y_mm), self.z.to_sim(z_mm)

    def printer_to_conf(self, pose: PrinterPose) -> tuple[tuple[float, float, float], float]:
        """PrinterPose [mm] -> (planner conf (x_mid, y, z) [m], gap [m])."""
        x1, x2, y, z = self.printer_to_joints(pose.x1, pose.x2, pose.y, pose.z)
        return ((x1 + x2) / 2, y, z), abs(x1 - x2)

    # ---- validation of printer-side poses ------------------------------------------
    def check_pose(self, pose: PrinterPose) -> None:
        """Raise ValueError if the pose breaks a firmware limit (jaws, gap, Y, Z)."""
        self.x.check(pose.x1)
        self.x.check(pose.x2)
        self._check_gap(pose.gap)
        self.y.check(pose.y)
        self.z.check(pose.z)

    # ---- helpers ---------------------------------------------------------------------
    def _order_to_names(self, lo_m: float, hi_m: float) -> tuple[float, float]:
        """Sim jaws by joint value -> (x1_axis, x2_axis): X1 is the jaw nearer printer
        X min, i.e. the lower joint value when the X sign is +1 (default)."""
        return (lo_m, hi_m) if self.x.sign > 0 else (hi_m, lo_m)

    def _check_gap(self, gap_mm: float, tol: float = 1e-6) -> None:
        if gap_mm < self.min_gap_mm - tol:
            raise ValueError(
                f"gap {gap_mm:.3f} mm below the firmware minimum {self.min_gap_mm:g} mm"
            )


DEFAULT = SimToPrinter()
