"""Bridge: drive the real printer from the simulation GUI, and keep the sim in sync.

Run next to the simulation, in its own terminal (needs pyserial):
    python RoboCup/Simulation/simulation_gui.py
    python RoboCup/Translation/sim_printer_bridge.py /dev/ttyUSB0 [--home]

What it does:
  * startup   : connects (optionally homes with G28), reads the printer position (M114)
                and moves the sim there, so both start from the same pose.
  * sim input : a slider target (once the slider has been still for --debounce s) or a
                planned move ("Plan & move to goal") from simulation_gui.py is converted with
                sim_to_printer and sent with WaypointSender, one waypoint at a time,
                each checked against the printer's M114 report.
  * on error  : endstop hit, firmware error, position mismatch or a target outside the
                printer's range -> the run stops, the real position is read back and
                the sim is reset to it (its planned move is aborted).

Only the newest request is kept: input that arrives while the printer is moving
replaces any earlier request still waiting, and is run once the current one is done.
"""

from __future__ import annotations

import argparse
import queue
import time

from pose_to_gcode import FEEDRATE
from sim_link import bridge_endpoint
from sim_to_printer import DEFAULT, SimToPrinter
from waypoint_sender import PrinterConnection, WaypointError, WaypointSender


class SimPrinterBridge:
    def __init__(self, sender: WaypointSender, sim, transform: SimToPrinter = DEFAULT,
                 debounce_s: float = 0.4, log=print):
        self.sender, self.sim, self.transform = sender, sim, transform
        self.debounce_s = debounce_s
        self.log = log

    # ---- printer -> sim --------------------------------------------------------------
    def sync_sim(self, reason: str) -> bool:
        """Read the printer position and move the sim there. False if unreadable."""
        try:
            pose = self.sender.read_pose()
        except WaypointError as exc:
            self.log(f"!! cannot read the printer position, sim not synced: {exc}")
            return False
        joints = self.transform.printer_to_joints(pose.x1, pose.x2, pose.y, pose.z)
        self.sim.send({"type": "sync", "joints": list(joints), "reason": reason})
        self.log(f"sim synced ({reason}): X1={pose.x1:.2f} X2={pose.x2:.2f} "
                 f"Y={pose.y:.2f} Z={pose.z:.2f} mm")
        return True

    # ---- sim -> printer --------------------------------------------------------------
    def execute(self, msg: dict) -> None:
        kind = msg.get("type")
        try:
            if kind == "path":
                goal_gap = msg.get("goal_gap")
                self.log(f"planned move: {len(msg['waypoints'])} waypoints, "
                         f"gap {msg['gap'] * 1000:.1f} mm"
                         + (f", then gap {goal_gap * 1000:.1f} mm at the goal"
                            if goal_gap is not None else ""))
                self.sender.run_sim_path(msg["waypoints"], msg["gap"], goal_gap)
            elif kind == "goal":
                try:
                    pose = self.transform.joints_to_printer(*msg["joints"])
                except ValueError as exc:
                    raise WaypointError(f"not started: {exc}") from None
                self.log(f"slider target: X1={pose.x1:.2f} X2={pose.x2:.2f} "
                         f"Y={pose.y:.2f} Z={pose.z:.2f} mm")
                self.sender.run([pose])
            else:
                return
        except WaypointError as exc:
            self.log(f"!! stopped: {exc}")
            self.sync_sim("endstop hit" if "endstops hit" in str(exc) else "printer error")
        except (KeyError, TypeError, ValueError) as exc:
            self.log(f"!! malformed message from the sim ignored: {exc}")

    def loop(self) -> None:
        pending, last_rx = None, 0.0
        while True:
            for msg in self.sim.receive(timeout=0.1):
                if msg.get("type") in ("goal", "path"):
                    pending, last_rx = msg, time.monotonic()
            if pending is None:
                continue
            settled = time.monotonic() - last_rx >= self.debounce_s
            if pending["type"] == "path" or settled:
                msg, pending = pending, None
                self.execute(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="printer serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--home", action="store_true", help="home all axes (G28) at startup")
    ap.add_argument("--feedrate", type=int, default=FEEDRATE, help="G1 feedrate [mm/min]")
    ap.add_argument("--tol", type=float, default=0.05, help="position check tolerance [mm]")
    ap.add_argument("--debounce", type=float, default=0.4,
                    help="slider must be still this long before it is sent [s]")
    args = ap.parse_args()

    try:
        sim = bridge_endpoint()
    except OSError as exc:
        raise SystemExit(f"cannot open the sim link (bridge already running?): {exc}")

    rx = queue.Queue()
    link = PrinterConnection(rx)
    link.connect(args.port)
    sender = WaypointSender(link, rx, feedrate=args.feedrate, tol_mm=args.tol)
    bridge = SimPrinterBridge(sender, sim, debounce_s=args.debounce)
    try:
        if args.home:
            print("homing (G28) ...")
            try:
                sender.home()
            except WaypointError as exc:
                print(f"!! homing failed: {exc}")
        bridge.sync_sim("startup")
        print("bridge running, Ctrl+C to stop")
        bridge.loop()
    except KeyboardInterrupt:
        pass
    finally:
        link.disconnect()
        sim.close()


if __name__ == "__main__":
    main()
