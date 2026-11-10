"""Local UDP link between the simulation GUI (Simulation/simulation_gui.py) and the printer
bridge (Translation/sim_printer_bridge.py). Standard library only, so the simulation
venv needs nothing extra. One JSON object per datagram, simulation units (metres).

sim -> bridge
    {"type": "goal", "joints": [x1, x2, y, z]}                  slider target (settled)
    {"type": "path", "waypoints": [[x, y, z], ...], "gap": g,   planned move (x = jaw midpoint)
     "goal_gap": g2}                                            optional: gap set at the goal
bridge -> sim
    {"type": "sync", "joints": [x1, x2, y, z], "reason": str}   real printer position:
                                                                 the sim jumps there
"""

from __future__ import annotations

import json
import select
import socket

HOST = "127.0.0.1"
BRIDGE_PORT = 47811  # the bridge listens here
SIM_PORT = 47812     # the simulation listens here


class Endpoint:
    """Non-blocking UDP endpoint. Raises OSError if listen_port is taken."""

    def __init__(self, listen_port: int, peer_port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((HOST, listen_port))
        except OSError:
            self.sock.close()
            raise
        self.sock.setblocking(False)
        self.peer = (HOST, peer_port)

    def send(self, msg: dict) -> bool:
        """Fire and forget; False if it could not be sent (nobody listening is fine)."""
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.peer)
            return True
        except OSError:
            return False

    def receive(self, timeout: float = 0.0) -> list[dict]:
        """All messages waiting now; waits up to `timeout` s for the first one."""
        if timeout > 0:
            select.select([self.sock], [], [], timeout)
        msgs = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                return msgs
            except OSError:  # e.g. ICMP "port unreachable" from an earlier send
                continue
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            if isinstance(msg, dict):
                msgs.append(msg)

    def close(self) -> None:
        self.sock.close()


def sim_endpoint() -> Endpoint:
    return Endpoint(SIM_PORT, BRIDGE_PORT)


def bridge_endpoint() -> Endpoint:
    return Endpoint(BRIDGE_PORT, SIM_PORT)
