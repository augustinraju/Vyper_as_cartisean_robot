"""Plan and execute a collision-free move of the closed/fixed-gap gripper from A to B.

Uses pybullet_planning's bidirectional RRT (birrt) + path smoothing over a coupled
3-D configuration space instead of the raw joints:

    conf = (x, y, z)
      x : gripper centre, joint value    -> right jaw (x1_axis) = x - gap/2,
          (X joints point along world -X)   left jaw  (x2_axis) = x + gap/2
                                            world x of the centre = -x
      y : bed position (y_axis)          -> the bed moves, not the gripper
      z : gantry height (z_axis)

Both jaws are always commanded together (M606 S0 style), so the gap is constant
along the whole path and the jaws can never cross.

Run with the assembly_station venv, e.g.:
    ../../assembly_station/bin/python plan_gripper_move.py
    ../../assembly_station/bin/python plan_gripper_move.py \
        --start -0.07 0.06 0.005 --goal 0.07 0.12 0.005 --gap 0.03
    ../../assembly_station/bin/python plan_gripper_move.py --no-obstacle

In the GUI: press 'r' to replay the motion, close the window to quit.
"""
import argparse
import math
import random
import time

import pybullet as p
import pybullet_data
from pybullet_planning import birrt, smooth_path

from build_urdf import build as build_urdf

STEP = 0.002            # interpolation resolution for collision checking [m]
FINGER_TIP = (0.135, -0.0565)  # finger tip centre (y, z) in the jaw link frame (see vyper.urdf)


class GripperPlanner:
    """Coupled (x, y, z) planner for the Vyper gripper on top of pybullet_planning."""

    def __init__(self, robot, gap, obstacles=(), client=0):
        self.robot = robot
        self.gap = gap
        self.cid = client
        j = {p.getJointInfo(robot, i, physicsClientId=client)[1].decode(): i
             for i in range(p.getNumJoints(robot, physicsClientId=client))}
        # Jaws by side (names follow the printer: X1 = right jaw, X2 = left jaw)
        self.jleft, self.jright = j["x2_axis"], j["x1_axis"]
        # Jaws by joint value: the X joints point along world -X, so x1 <= x2
        self.jlo, self.jhi = self.jright, self.jleft
        self.jy, self.jz = j["y_axis"], j["z_axis"]
        # child link index == joint index in PyBullet
        self.bed, self.gantry = self.jy, self.jz
        self.left_jaw, self.right_jaw = self.jleft, self.jright
        self.frame = j["frame_mount"]
        self.base = -1

        lim = lambda i: p.getJointInfo(robot, i, physicsClientId=client)[8:10]
        x_lo, x_hi = lim(self.jlo)
        self.lower = (x_lo + gap / 2, *lim(self.jy)[:1], lim(self.jz)[0])
        self.upper = (x_hi - gap / 2, lim(self.jy)[1], lim(self.jz)[1])
        if self.lower[0] > self.upper[0]:
            raise ValueError(f"gap {gap} m is larger than the X travel")

        # obstacles: list of (body_id, local pose in bed frame) -- they ride on the bed
        self.obstacles = list(obstacles)

        # Link pairs worth checking (adjacent parent/child pairs are skipped on purpose:
        # carriages wrap the uprights, jaws sit on the spindle, jaws touch when closed).
        moving = (self.gantry, self.left_jaw, self.right_jaw)
        self.self_pairs = [(self.bed, l) for l in moving + (self.frame, self.base)]
        self.self_pairs += [(l, s) for l in (self.left_jaw, self.right_jaw)
                            for s in (self.frame, self.base)]
        self.obstacle_links = moving + (self.frame,)

    # ---- configuration handling -------------------------------------------------
    def set_conf(self, q):
        x, y, z = q
        for joint, val in ((self.jlo, x - self.gap / 2), (self.jhi, x + self.gap / 2),
                           (self.jy, y), (self.jz, z)):
            p.resetJointState(self.robot, joint, val, physicsClientId=self.cid)
        update_bed_obstacles(self.robot, self.bed, self.obstacles, self.cid)

    def in_limits(self, q):
        return all(lo - 1e-9 <= v <= hi + 1e-9 for v, lo, hi in zip(q, self.lower, self.upper))

    def colliding_pairs(self, q):
        """Return a list of human-readable colliding pairs (empty = free)."""
        self.set_conf(q)
        hits = []
        name = lambda l: "base_link" if l == -1 else \
            p.getJointInfo(self.robot, l, physicsClientId=self.cid)[12].decode()
        for a, b in self.self_pairs:
            if p.getClosestPoints(self.robot, self.robot, 0.0, a, b, physicsClientId=self.cid):
                hits.append((name(a), name(b)))
        for body, _ in self.obstacles:
            for l in self.obstacle_links:
                if p.getClosestPoints(self.robot, body, 0.0, l, -1, physicsClientId=self.cid):
                    hits.append((name(l), f"obstacle#{body}"))
        return hits

    def opening_collisions(self, q, new_gap):
        """Collisions that appear while opening from self.gap to new_gap at conf q.

        The opening is stepped every STEP. Contacts already present at the current gap
        are ignored, so a jaw touching something can still move away from it. Closing
        (new_gap <= self.gap) is not checked and returns []."""
        if new_gap <= self.gap:
            return []
        old = self.gap
        try:
            before = set(self.colliding_pairs(q))
            n = max(1, math.ceil((new_gap - old) / STEP))
            for i in range(1, n + 1):
                self.gap = old + (new_gap - old) * i / n
                hits = [h for h in self.colliding_pairs(q) if h not in before]
                if hits:
                    return hits
            return []
        finally:
            self.gap = old

    # ---- functions handed to pybullet_planning -----------------------------------
    def collision_fn(self, q, diagnosis=False, **kwargs):
        if not self.in_limits(q):
            return True
        hits = self.colliding_pairs(q)
        if hits and diagnosis:
            print("collision at", q, hits)
        return bool(hits)

    def sample_fn(self):
        return tuple(random.uniform(lo, hi) for lo, hi in zip(self.lower, self.upper))

    @staticmethod
    def distance_fn(q1, q2):
        return math.dist(q1, q2)

    @staticmethod
    def extend_fn(q1, q2):
        n = max(1, int(math.ceil(math.dist(q1, q2) / STEP)))
        for i in range(1, n):
            t = i / n
            yield tuple(a + t * (b - a) for a, b in zip(q1, q2))
        yield tuple(q2)

    # ---- planning ----------------------------------------------------------------
    def plan(self, start, goal, max_time=10.0, smooth=True):
        for label, q in (("start", start), ("goal", goal)):
            if not self.in_limits(q):
                raise ValueError(f"{label} {q} outside limits lower={self.lower} upper={self.upper}")
            hits = self.colliding_pairs(q)
            if hits:
                raise ValueError(f"{label} {q} is in collision: {hits}")

        path = birrt(start, goal, self.distance_fn, self.sample_fn, self.extend_fn,
                     self.collision_fn, max_time=max_time)
        if path is None:
            return None
        if smooth:
            path = smooth_path(path, self.extend_fn, self.collision_fn,
                               distance_fn=self.distance_fn, max_smooth_iterations=100)
        return [tuple(q) for q in path]

    def densify(self, path):
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            out.extend(self.extend_fn(a, b))
        return out

    # ---- helpers for drawing ---------------------------------------------------
    def tip_in_bed_frame(self, q):
        """Gripper tip centre expressed in the bed link frame (bed frame is axis-aligned)."""
        self.set_conf(q)
        jaw = p.getLinkState(self.robot, self.left_jaw, physicsClientId=self.cid)[4]
        jaw_r = p.getLinkState(self.robot, self.right_jaw, physicsClientId=self.cid)[4]
        bed = p.getLinkState(self.robot, self.bed, physicsClientId=self.cid)[4]
        # world midpoint of the two inner faces (independent of the X joint direction)
        tip = ((jaw[0] + jaw_r[0]) / 2, jaw[1] + FINGER_TIP[0], jaw[2] + FINGER_TIP[1])
        return tuple(t - b for t, b in zip(tip, bed))


BED_SURFACE = 0.01  # bed top face (visual) in the bed link frame [m]


def add_bed_obstacle(size=(0.03, 0.03, 0.03), xy=(0.0, 0.005), z=0.0,
                     rgba=(0.8, 0.1, 0.1, 1), client=0):
    """Box riding on the bed, its bottom `z` above the bed surface.

    Returns (body, local pose in bed frame); keep it on the bed with update_bed_obstacles().
    """
    half = [s / 2 for s in size]
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=client)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=client)
    body = p.createMultiBody(0, col, vis, physicsClientId=client)
    return body, ((xy[0], xy[1], BED_SURFACE + z + half[2]), (0, 0, 0, 1))


def update_bed_obstacles(robot, bed_link, obstacles, client=0):
    """Move every (body, local_pose) obstacle so it follows the bed link."""
    bed_pose = p.getLinkState(robot, bed_link, physicsClientId=client)[4:6]
    for body, local in obstacles:
        pos, orn = p.multiplyTransforms(*bed_pose, *local)
        p.resetBasePositionAndOrientation(body, pos, orn, physicsClientId=client)


def draw_in_bed_frame(robot, bed, points, color, width=3):
    for a, b in zip(points, points[1:]):
        p.addUserDebugLine(a, b, color, width, parentObjectUniqueId=robot, parentLinkIndex=bed)


def execute(planner, dense_path, speed):
    dt = STEP / speed
    for q in dense_path:
        if not p.isConnected():
            return
        planner.set_conf(q)
        time.sleep(dt)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", nargs=3, type=float, default=[-0.07, 0.06, 0.005],
                    metavar=("X", "Y", "Z"), help="point A: gripper centre x, bed y, gantry z [m]")
    ap.add_argument("--goal", nargs=3, type=float, default=[0.07, 0.12, 0.005],
                    metavar=("X", "Y", "Z"), help="point B: gripper centre x, bed y, gantry z [m]")
    ap.add_argument("--gap", type=float, default=0.03, help="jaw opening kept during the move [m]")
    ap.add_argument("--no-obstacle", action="store_true", help="don't place the demo block on the bed")
    ap.add_argument("--speed", type=float, default=0.05, help="playback speed [m/s in conf space]")
    ap.add_argument("--max-time", type=float, default=10.0, help="planner time budget [s]")
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(build_urdf(), useFixedBase=True)  # joint limits = printer limits
    p.resetDebugVisualizerCamera(0.9, 45, -30, [0, 0, 0.2])

    obstacles = [] if args.no_obstacle else [add_bed_obstacle()]
    planner = GripperPlanner(robot, args.gap, obstacles)
    start, goal = tuple(args.start), tuple(args.goal)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    t0 = time.time()
    try:
        path = planner.plan(start, goal, max_time=args.max_time)
    finally:
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    if path is None:
        print(f"No path found in {args.max_time}s")
        planner.set_conf(start)
        while p.isConnected():
            time.sleep(0.1)
        return
    print(f"Planned in {time.time() - t0:.2f}s, {len(path)} waypoints (x, y, z):")
    for q in path:
        print("  " + "  ".join(f"{v:+.4f}" for v in q))

    dense = planner.densify(path)
    tips = [planner.tip_in_bed_frame(q) for q in dense]
    draw_in_bed_frame(robot, planner.bed, tips, (0, 0.8, 0))
    for q, color, label in ((start, (0, 0, 1), "A"), (goal, (1, 0, 1), "B")):
        tip = planner.tip_in_bed_frame(q)
        p.addUserDebugText(label, [tip[0], tip[1], tip[2] + 0.02], color, 1.5,
                           parentObjectUniqueId=robot, parentLinkIndex=planner.bed)

    planner.set_conf(start)
    time.sleep(0.5)
    execute(planner, dense, args.speed)
    print("Reached B. Press 'r' in the GUI to replay, close the window to quit.")
    while p.isConnected():
        keys = p.getKeyboardEvents()
        if keys.get(ord("r"), 0) & p.KEY_WAS_TRIGGERED:
            planner.set_conf(start)
            time.sleep(0.3)
            execute(planner, dense, args.speed)
        time.sleep(0.02)


if __name__ == "__main__":
    main()
