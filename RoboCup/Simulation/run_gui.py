"""Open the simplified Anycubic Vyper URDF in the PyBullet GUI with joint sliders.

Extra: set goal_x / goal_y / goal_z and press "Plan & move to goal" to move the
gripper (both jaws together, current gap kept) along a collision-free path
planned with pybullet_planning (see plan_gripper_move.py).
"""
import os
import time

import pybullet as p
import pybullet_data

from plan_gripper_move import STEP, GripperPlanner, add_bed_obstacle, update_bed_obstacles

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vyper.urdf")
DT = 1 / 240
MOVE_SPEED = 0.05  # planned-move speed in (x, y, z) space [m/s]

# Lego blocks riding on the bed: (x, y, z) size, 3 cm across the jaws (X), 6 cm along
# the fingers (Y), 3 cm tall. Stacked vertically at the bed centre.
LEGO_SIZE = (0.03, 0.06, 0.03)
LEGO_XY = (0.0, 0.0)  # stack position in the bed frame
LEGO_COLORS = [(0.85, 0.1, 0.1, 1), (0.95, 0.8, 0.1, 1)]  # bottom red, top yellow
HOME_Z = 0.08  # initial gantry height: fingers clear the 6 cm stack


def add_lego_stack(client):
    return [
        add_bed_obstacle(LEGO_SIZE, LEGO_XY, z=i * LEGO_SIZE[2], rgba=c, client=client)
        for i, c in enumerate(LEGO_COLORS)
    ]


def main():
    gui = p.connect(p.GUI)
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

    sliders = []
    for j in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, j)
        if info[2] == p.JOINT_PRISMATIC:
            name = info[1].decode()
            init = HOME_Z if name == "z_axis" else 0.0
            p.resetJointState(robot, j, init)
            sliders.append(
                (j, p.addUserDebugParameter(name, info[8], info[9], init), info[10])
            )

    names = {p.getJointInfo(robot, j)[1].decode(): j for j, _, _ in sliders}
    x1, x2 = names.get("x1_axis"), names.get("x2_axis")
    jy, jz = names["y_axis"], names["z_axis"]
    update_bed_obstacles(robot, jy, legos, gui)
    lim = lambda j: p.getJointInfo(robot, j)[8:10]

    # Goal sliders + button (a parameter with min > max is a button in PyBullet)
    goal_sliders = [
        p.addUserDebugParameter("goal_x (gripper centre)", *lim(x1), 0.0),
        p.addUserDebugParameter("goal_y (bed)", *lim(jy), 0.0),
        p.addUserDebugParameter("goal_z (gantry)", *lim(jz), HOME_Z),
    ]
    plan_btn = p.addUserDebugParameter("Plan & move to goal", 1, 0, 0)
    btn_prev = p.readUserDebugParameter(plan_btn)

    raw_prev = {}
    last_moved = None
    cmd = {}            # commanded joint targets (persist while a slider is untouched)
    trajectory = []     # densified (x, y, z) path being executed
    traj_gap = 0.0
    traj_idx = 0.0
    path_lines = []

    while p.isConnected(gui):
        raw = {j: p.readUserDebugParameter(s) for j, s, _ in sliders}
        changed = [j for j in raw if not raw_prev or raw[j] != raw_prev[j]]
        if changed and trajectory and raw_prev:
            print("Planned move aborted by slider input")
            trajectory = []
        for j in changed:
            cmd[j] = raw[j]

        # Jaws share one spindle and must not cross: the left jaw (x1) may
        # travel from the left motor up to the right jaw, and the right jaw
        # (x2) from the right motor up to the left jaw. The jaw being moved
        # stops when it reaches the other one.
        if x1 is not None and x2 is not None:
            for j in (x1, x2):
                if raw_prev and j in changed:
                    last_moved = j
            if cmd[x1] > cmd[x2]:
                if last_moved == x2:
                    cmd[x2] = cmd[x1]
                else:
                    cmd[x1] = cmd[x2]
        raw_prev = raw

        # ---- Plan & move button -------------------------------------------------
        btn = p.readUserDebugParameter(plan_btn)
        if btn != btn_prev:
            btn_prev = btn
            gap = max(0.0, cmd[x2] - cmd[x1])
            planner = GripperPlanner(plan_robot, gap, obstacles=plan_legos, client=plan_cid)
            clamp = lambda q: tuple(min(max(v, lo), hi)
                                    for v, lo, hi in zip(q, planner.lower, planner.upper))
            start = clamp(((cmd[x1] + cmd[x2]) / 2, cmd[jy], cmd[jz]))
            goal = clamp(tuple(p.readUserDebugParameter(s) for s in goal_sliders))
            try:
                path = planner.plan(start, goal, max_time=5.0)
            except ValueError as e:
                path = None
                print(f"Planning failed: {e}")
            if path is None:
                print("No path found")
            else:
                trajectory, traj_gap, traj_idx = planner.densify(path), gap, 0.0
                print(f"Planned {len(path)} waypoints, gap {gap * 1000:.1f} mm: {start} -> {goal}")
                for line in path_lines:
                    p.removeUserDebugItem(line)
                tips = [planner.tip_in_bed_frame(q) for q in trajectory[::5] + trajectory[-1:]]
                path_lines = [
                    p.addUserDebugLine(a, b, (0, 0.8, 0), 3,
                                       parentObjectUniqueId=robot, parentLinkIndex=jy)
                    for a, b in zip(tips, tips[1:])
                ]

        # ---- follow the planned trajectory ---------------------------------------
        if trajectory:
            q = trajectory[min(int(traj_idx), len(trajectory) - 1)]
            cmd[x1], cmd[x2] = q[0] - traj_gap / 2, q[0] + traj_gap / 2
            cmd[jy], cmd[jz] = q[1], q[2]
            traj_idx += MOVE_SPEED * DT / STEP
            if traj_idx >= len(trajectory):
                trajectory = []
                print("Reached goal")

        for j, _, force in sliders:
            p.setJointMotorControl2(
                robot, j, p.POSITION_CONTROL,
                targetPosition=cmd[j], force=force,
            )
        p.stepSimulation()
        update_bed_obstacles(robot, jy, legos, gui)  # blocks ride on the bed
        time.sleep(DT)


if __name__ == "__main__":
    main()
