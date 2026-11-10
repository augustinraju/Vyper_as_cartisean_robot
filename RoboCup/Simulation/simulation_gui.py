"""Open the simplified Anycubic Vyper URDF in the PyBullet GUI with joint sliders.

Sliders: z_axis, y_axis, and for the jaws x_mid (gripper centre, joint value) + gap
(x2 - x1); both jaws are commanded from these, so they can never cross.

Extra: set goal_x / goal_y / goal_z / goal_gap and press "Plan & move to goal" to move
the gripper (both jaws together, current gap kept) along a collision-free path planned
with pybullet_planning (see plan_gripper_move.py), then open/close it to goal_gap there.

Printer link (optional): with Translation/sim_printer_bridge.py running, settled slider
targets and planned moves are sent to the real printer, and the sim jumps to the
printer's real position when the bridge reports it (startup, endstop hit, error).

    python simulation_gui.py             # OpenGL3: textures + shadows
    python simulation_gui.py --opengl2   # simpler 3D view, but PyBullet's own menu/tab
                                         # text is no longer clipped on the left
"""
import argparse
import math
import os
import sys
import time

import pybullet as p
import pybullet_data

from plan_gripper_move import STEP, GripperPlanner, add_bed_obstacle, update_bed_obstacles

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Translation"))
from sim_link import sim_endpoint  # noqa: E402  (standard library only)

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vyper.urdf")
DT = 1 / 240
MOVE_SPEED = 0.05  # planned-move speed in (x, y, z) space [m/s]

# Lego blocks riding on the bed: (x, y, z) size, 3 cm across the jaws (X), 6 cm along
# the fingers (Y), 3 cm tall. Stacked vertically at the bed centre.
LEGO_SIZE = (0.03, 0.06, 0.03)
LEGO_XY = (0.0, 0.0)  # stack position in the bed frame
LEGO_COLORS = [(0.85, 0.1, 0.1, 1), (0.95, 0.8, 0.1, 1)]  # bottom red, top yellow
HOME_Z = 0.08  # initial gantry height: fingers clear the 6 cm stack
# PyBullet's OpenGL3 GUI clips the first few pixels of every text; leading spaces keep
# our slider/button labels readable (its own menu and tab names stay clipped)
LABEL_PAD = "  "
# The x_mid / gap slider ranges depend on each other; they are rebuilt once those sliders
# have been still this long (rebuilding mid-drag would drop the slider being dragged)
RANGE_SETTLE_S = 0.4
MIN_SLIDER_SPAN = 1e-4  # [m] keeps a slider usable when its reachable range is empty


def add_lego_stack(client):
    return [
        add_bed_obstacle(LEGO_SIZE, LEGO_XY, z=i * LEGO_SIZE[2], rgba=c, client=client)
        for i, c in enumerate(LEGO_COLORS)
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--opengl2", action="store_true",
                    help="OpenGL2 renderer: unclipped GUI text, no textures/shadows")
    args = ap.parse_args()

    gui = p.connect(p.GUI, options="--opengl2" if args.opengl2 else "")
    # Hide the (empty) synthetic-camera preview panels drawn over the left of the view
    for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW, p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                 p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW):
        p.configureDebugVisualizer(flag, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF, useFixedBase=True)

    # Headless copy of the robot used only for collision checking while planning,
    # so planning never disturbs the simulated robot shown in the GUI.
    plan_cid = p.connect(p.DIRECT)
    plan_robot = p.loadURDF(URDF, useFixedBase=True, physicsClientId=plan_cid)

    # Same Lego stack in both worlds: shown in the GUI, avoided by the planner
    legos = add_lego_stack(gui)
    plan_legos = add_lego_stack(plan_cid)

    p.resetDebugVisualizerCamera(0.9, 45, -30, [0, 0, 0.2])

    try:
        printer_link = sim_endpoint()  # talks to sim_printer_bridge.py, if it runs
    except OSError as e:
        printer_link = None
        print(f"Printer link disabled (another simulation running?): {e}")

    # Prismatic joints: (joint index, name, lower, upper, max force)
    joints = []
    for j in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        if info[2] == p.JOINT_PRISMATIC:
            joints.append((j, info[1].decode(), info[8], info[9], info[10]))
    force = {j: f for j, _, _, _, f in joints}
    lim = {j: (lo_, hi_) for j, _, lo_, hi_, _ in joints}
    names = {name: j for j, name, *_ in joints}
    x1, x2 = names["x1_axis"], names["x2_axis"]
    # Names follow the printer (X1 homes to printer X min = world right side). The X
    # joints point along world -X, so the right jaw (X1) has the LOWER joint value:
    # lo = x1 <= hi = x2, gap = hi - lo
    lo, hi = x1, x2
    jy, jz = names["y_axis"], names["z_axis"]
    x_lo, x_hi = max(lim[lo][0], lim[hi][0]), min(lim[lo][1], lim[hi][1])  # shared X travel

    # Slider keys: joint index for Y/Z, "x_mid"/"gap" for the jaws (both commanded together)
    slider_specs = [(jz, "z_axis"), ("x_mid", "x_mid (gripper centre)"),
                    ("gap", "gap (x2 - x1)"), (jy, "y_axis")]
    # Goal: the gripper moves to (goal_x, goal_y, goal_z) with its current gap, then
    # opens/closes to goal_gap there
    goal_specs = [("goal_x (gripper centre)", "x_mid"), ("goal_y (bed)", jy),
                  ("goal_z (gantry)", jz), ("goal_gap (set at goal)", "goal_gap")]

    cmd = {j: HOME_Z if j == jz else 0.0 for j in force}  # commanded joint targets
    for j, v in cmd.items():
        p.resetJointState(robot, j, v)
    update_bed_obstacles(robot, jy, legos, gui)

    def mid_gap():
        return (cmd[lo] + cmd[hi]) / 2, cmd[hi] - cmd[lo]

    def set_jaws(mid, gap):
        cmd[lo], cmd[hi] = mid - gap / 2, mid + gap / 2

    def opening_collisions(q, gap, new_gap):
        """New collisions while opening from gap to new_gap at q = (x_mid, y, z)."""
        if new_gap <= gap:
            return []
        planner = GripperPlanner(plan_robot, gap, obstacles=plan_legos, client=plan_cid)
        return planner.opening_collisions(q, new_gap)

    def gap_fit(mid):
        """Largest gap that fits around centre `mid` inside the X travel."""
        return max(0.0, 2 * min(mid - x_lo, x_hi - mid))

    def reachable(key, goal_x=None):
        """Slider range: only what can actually be reached from the current pose.

        gap runs from 0 up to what fits around the current centre; x_mid (and goal_x)
        keeps half the current gap inside the X travel; goal_gap is what fits around
        goal_x; Y/Z use the joint limits.
        """
        mid, gap = mid_gap()
        if key == "x_mid":
            rlo, rhi = x_lo + gap / 2, x_hi - gap / 2
        elif key == "gap":
            rlo, rhi = 0.0, gap_fit(mid)
        elif key == "goal_gap":
            rlo, rhi = 0.0, gap_fit(goal_x)
        else:
            rlo, rhi = lim[key]
        if rhi - rlo < MIN_SLIDER_SPAN:  # PyBullet needs max > min (min > max = button)
            c = (rlo + rhi) / 2
            rlo, rhi = c - MIN_SLIDER_SPAN / 2, c + MIN_SLIDER_SPAN / 2
        return rlo, rhi

    def slider_value(key):
        return dict(zip(("x_mid", "gap"), mid_gap())).get(key) if key in ("x_mid", "gap") \
            else cmd[key]

    def build_params(goal_vals):
        """(Re)create every GUI parameter, in the same order, from cmd and goal_vals.

        PyBullet cannot move an existing slider or change its range, so showing a new
        value (printer sync, end of a planned move) or a new reachable range (x_mid or
        gap changed) means removing all parameters and adding them again.
        Returns (sliders, goal_sliders, plan_btn); the old ids become invalid.
        """
        p.removeAllUserParameters()
        clip = lambda v, r: min(max(v, r[0]), r[1])
        new_sliders = {}
        for key, label in slider_specs:
            r = reachable(key)
            new_sliders[key] = p.addUserDebugParameter(LABEL_PAD + label, *r,
                                                       clip(slider_value(key), r))
        # Goal sliders + button (a parameter with min > max is a button in PyBullet)
        new_goals = []
        goal_x = None
        for (label, key), v in zip(goal_specs, goal_vals):
            r = reachable(key, goal_x)
            v = clip(v, r)
            if key == "x_mid":
                goal_x = v  # goal_gap's range depends on it (goal_x comes first)
            new_goals.append(p.addUserDebugParameter(LABEL_PAD + label, *r, v))
        plan = p.addUserDebugParameter(LABEL_PAD + "Plan & move to goal", 1, 0, 0)
        return new_sliders, new_goals, plan

    sliders, goal_sliders, plan_btn = build_params((0.0, 0.0, HOME_Z, 0.0))
    btn_prev = p.readUserDebugParameter(plan_btn)
    refresh_sliders = False  # rebuild the sliders from cmd at the top of the next loop
    x_moved_at = None        # last x_mid/gap/goal_x change; ranges rebuild after RANGE_SETTLE_S
    goal_prev = [p.readUserDebugParameter(s) for s in goal_sliders]

    gap_blocked = False      # manual gap opening currently refused (print once)

    raw_prev = {}
    trajectory = []     # densified (x, y, z, gap) states being executed
    traj_idx = 0.0
    path_lines = []

    while p.isConnected(gui):
        if refresh_sliders:
            # Show cmd on the sliders; the new readings become raw_prev, so the rebuild
            # itself is not seen as slider input (no move abort, no goal to the printer)
            goal_vals = [p.readUserDebugParameter(s) for s in goal_sliders]
            sliders, goal_sliders, plan_btn = build_params(goal_vals)
            btn_prev = p.readUserDebugParameter(plan_btn)
            raw_prev = {k: p.readUserDebugParameter(s) for k, s in sliders.items()}
            goal_prev = [p.readUserDebugParameter(s) for s in goal_sliders]
            refresh_sliders = False

        raw = {k: p.readUserDebugParameter(s) for k, s in sliders.items()}
        changed = [k for k in raw if raw_prev and raw[k] != raw_prev[k]]
        if changed and trajectory:
            print("Planned move aborted by slider input")
            trajectory = []
        for k in changed:
            if k in force:  # Y / Z
                cmd[k] = raw[k]
        if "x_mid" in changed or "gap" in changed:
            # Both jaws move together. Until the ranges are rebuilt a slider can still
            # be dragged past what is reachable, so clamp: the gap must fit around the
            # current centre, and the centre keeps half the gap inside the X travel.
            mid, gap = mid_gap()
            if "gap" in changed:
                new_gap = min(max(raw["gap"], 0.0), gap_fit(mid))
                # Opening is only allowed if the jaws do not run into anything
                hits = opening_collisions((mid, cmd[jy], cmd[jz]), gap, new_gap)
                if hits:
                    if not gap_blocked:
                        print(f"Gap opening blocked by collision: {hits}")
                    gap_blocked = True
                else:
                    gap, gap_blocked = new_gap, False
            if "x_mid" in changed:
                mid = min(max(raw["x_mid"], x_lo + gap / 2), x_hi - gap / 2)
            set_jaws(mid, gap)
            x_moved_at = time.time()
        goal_now = [p.readUserDebugParameter(s) for s in goal_sliders]
        if goal_now[0] != goal_prev[0]:  # goal_x moved: goal_gap's range changes
            x_moved_at = time.time()
        goal_prev = goal_now
        if x_moved_at is not None and time.time() - x_moved_at > RANGE_SETTLE_S:
            x_moved_at = None
            refresh_sliders = True  # new ranges for x_mid, gap, goal_x and goal_gap
        if printer_link and raw_prev and changed:
            # The bridge waits until the slider is still before sending it
            printer_link.send({"type": "goal", "joints": [cmd[x1], cmd[x2], cmd[jy], cmd[jz]]})
        raw_prev = raw

        # ---- printer position from the bridge (startup, endstop hit, error) ------------
        for msg in printer_link.receive() if printer_link else ():
            if msg.get("type") == "sync":
                cmd[x1], cmd[x2], cmd[jy], cmd[jz] = msg["joints"]
                if trajectory:
                    print("Planned move aborted by printer sync")
                    trajectory = []
                refresh_sliders = True
                print(f"Synced to printer ({msg.get('reason', '')}): "
                      + ", ".join(f"{v:+.4f}" for v in msg["joints"]))

        # ---- Plan & move button -------------------------------------------------
        btn = p.readUserDebugParameter(plan_btn)
        if btn != btn_prev:
            btn_prev = btn
            gap = max(0.0, cmd[hi] - cmd[lo])
            planner = GripperPlanner(plan_robot, gap, obstacles=plan_legos, client=plan_cid)
            clamp = lambda q: tuple(min(max(v, lo), hi)
                                    for v, lo, hi in zip(q, planner.lower, planner.upper))
            start = clamp(((cmd[lo] + cmd[hi]) / 2, cmd[jy], cmd[jz]))
            *goal, goal_gap = (p.readUserDebugParameter(s) for s in goal_sliders)
            goal = clamp(goal)
            goal_gap = min(max(goal_gap, 0.0), gap_fit(goal[0]))
            try:
                path = planner.plan(start, goal, max_time=5.0)
            except ValueError as e:
                path = None
                print(f"Planning failed: {e}")
            if path is not None:
                # Opening at the goal must be collision-free, or the whole command is
                # refused. Closing is not checked: gripping a block is meant to touch it.
                hits = planner.opening_collisions(path[-1], goal_gap)
                if hits:
                    print(f"Command refused: opening to {goal_gap * 1000:.1f} mm at the goal "
                          f"collides: {hits}")
                    path = []
            if path is None:
                print("No path found")
            elif path:
                # Move with the current gap, then open/close to goal_gap at the goal.
                dense = planner.densify(path)
                trajectory = [(*q, gap) for q in dense]
                n = math.ceil(abs(goal_gap - gap) / STEP)
                trajectory += [(*dense[-1], gap + (goal_gap - gap) * i / n)
                               for i in range(1, n + 1)]
                traj_idx = 0.0
                print(f"Planned {len(path)} waypoints, gap {gap * 1000:.1f} mm: {start} -> {goal}"
                      f", then gap -> {goal_gap * 1000:.1f} mm")
                if printer_link:  # smoothed waypoints, not the densified trajectory
                    printer_link.send({"type": "path", "waypoints": [list(q) for q in path],
                                       "gap": gap, "goal_gap": goal_gap})
                for line in path_lines:
                    p.removeUserDebugItem(line)
                tips = [planner.tip_in_bed_frame(q) for q in dense[::5] + dense[-1:]]
                path_lines = [
                    p.addUserDebugLine(a, b, (0, 0.8, 0), 3,
                                       parentObjectUniqueId=robot, parentLinkIndex=jy)
                    for a, b in zip(tips, tips[1:])
                ]

        # ---- follow the planned trajectory ---------------------------------------
        if trajectory:
            x, y, z, g = trajectory[min(int(traj_idx), len(trajectory) - 1)]
            set_jaws(x, g)
            cmd[jy], cmd[jz] = y, z
            traj_idx += MOVE_SPEED * DT / STEP
            if traj_idx >= len(trajectory):
                trajectory = []
                refresh_sliders = True  # sliders show where the planned move ended
                print("Reached goal")

        for j, f in force.items():
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=cmd[j], force=f,
            )
        p.stepSimulation()
        update_bed_obstacles(robot, jy, legos, gui)  # blocks ride on the bed
        time.sleep(DT)


if __name__ == "__main__":
    main()
