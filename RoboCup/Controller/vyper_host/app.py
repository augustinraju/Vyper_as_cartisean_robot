"""Tkinter control window: wires the panels to PrinterConnection and the G-code builders."""

from __future__ import annotations

import queue
import tkinter as tk
from tkinter import messagebox
from typing import Iterable

import serial

from . import commands
from .config import POSITION_INTERVAL_MS
from .connection import PrinterConnection, available_ports
from .widgets import (PAD, CommandPanel, ConnectionPanel, GotoPanel, GripperPanel, HomePanel,
                      JogPanel, LogPanel, PositionPanel, ReferencePanel, XMotorPanel)

QUEUE_POLL_MS = 100


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Vyper Axis Control")
        self.resizable(False, False)

        self.rx_queue: queue.Queue = queue.Queue()
        self.printer = PrinterConnection(self.rx_queue)

        self._build_widgets()
        self.refresh_ports()
        self.after(QUEUE_POLL_MS, self._poll_queue)
        self.after(POSITION_INTERVAL_MS, self._position_tick)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_widgets(self) -> None:
        self.conn_panel = ConnectionPanel(self, self.refresh_ports, self.toggle_connection)
        self.pos_panel = PositionPanel(self)
        self.jog_panel = JogPanel(self, self.jog)
        self.gripper_panel = GripperPanel(self, self.step_gap, self.set_gap)
        self.goto_panel = GotoPanel(self, self.goto_position)
        self.log_panel = LogPanel(self)
        panels = [
            self.conn_panel,
            self.pos_panel,
            self.jog_panel,
            XMotorPanel(self, self.jog_x_motor),
            self.gripper_panel,
            HomePanel(self, self.home_axis),
            ReferencePanel(self, self.set_zero),
            self.goto_panel,
            CommandPanel(self, self.send_typed_command),
        ]
        for row, panel in enumerate(panels):
            panel.grid(row=row, column=0, sticky="ew", **PAD)
        # Log on the right, as tall as the whole control column
        self.log_panel.grid(row=0, column=1, rowspan=len(panels), sticky="nsew", **PAD)

    # ---------- Connection ----------
    def refresh_ports(self) -> None:
        self.conn_panel.set_ports(available_ports())

    def _disconnected(self, message: str) -> None:
        """Close the port and reset the UI (button, position panel)."""
        self.printer.disconnect()
        self.conn_panel.set_connected(False)
        self.pos_panel.clear()
        self.log_line(message)

    def toggle_connection(self) -> None:
        if self.printer.connected:
            self._disconnected("Disconnected")
            return
        port = self.conn_panel.port
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            self.printer.connect(port)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return
        self.conn_panel.set_connected(True)
        self.log_line(f"Connected to {port} @ {self.printer.baudrate}")
        # Check endstops on every move, not only during G28 (lost on board reset,
        # so it is re-sent on each connect). Queued; goes out once the board has booted.
        self.send_gcode(commands.ENABLE_ENDSTOPS)

    def _require_connection(self) -> bool:
        if not self.printer.connected:
            messagebox.showwarning("Not connected", "Connect to the printer first.")
            return False
        return True

    @property
    def printer_free(self) -> bool:
        """No command batch running (sent, acknowledged and its motion finished)."""
        return self.printer.connected and not self.printer.busy

    def send_gcode(self, lines: str | Iterable[str], log: bool = True) -> bool:
        """Send one command batch, but only while the printer is free; otherwise it is
        dropped (never queued) and False is returned.

        The batch ends with a silent M400, so the printer counts as busy until its
        moves have finished.
        """
        if isinstance(lines, str):
            lines = [lines]
        lines = list(lines)
        if not lines or not self._require_connection():
            return False
        if not self.printer_free:
            self.log_line("!! Printer busy, command ignored")
            return False
        try:
            self.printer.send_lines(lines, log)
            self.printer.send(commands.WAIT_FOR_MOVES, log=False)
        except RuntimeError:
            messagebox.showwarning("Not connected", "Connect to the printer first.")
            return False
        except ValueError as exc:
            messagebox.showwarning("Invalid command", str(exc))
            return False
        return True

    # ---------- Actions ----------
    def send_typed_command(self, text: str) -> str | None:
        """Send the typed line (anything after ';' is a comment). Returns it if sent."""
        line = commands.strip_comment(text)
        if not line or not self._require_connection():
            return None
        bad = commands.non_ascii_chars(line)
        if bad:
            # Keep the text in the box so it can be corrected
            messagebox.showwarning("Invalid command", f"G-code is ASCII only. Remove: {bad}")
            return None
        return line if self.send_gcode(line) else None

    def jog(self, axis: str, direction: int) -> None:
        self.send_gcode(commands.jog(axis, self.jog_panel.step * direction))

    def jog_x_motor(self, motor: int, direction: int) -> None:
        self.send_gcode(commands.jog_x_motor(motor, self.jog_panel.step * direction))

    def step_gap(self, direction: int) -> None:
        """Open (+1) or close (-1) the gripper by the jog step."""
        self.send_gcode(commands.change_gap(self.jog_panel.step * direction))

    def set_gap(self) -> None:
        """Move both jaws at once to the typed gap (firmware M606 S2 knows the current gap)."""
        if not self._require_connection():
            return
        try:
            target = self.gripper_panel.target()
        except ValueError:
            messagebox.showwarning("Invalid value", "Enter the gap as a number >= 0 (mm).")
            return
        self.send_gcode(commands.set_gap(target))

    def home_axis(self, axis: str) -> None:
        if self._require_connection():
            self.send_gcode(commands.home(axis))

    def goto_position(self) -> None:
        """Move to typed X mid / Y / Z targets (empty field = axis untouched)."""
        if not self._require_connection():
            return
        try:
            targets = self.goto_panel.targets()
        except ValueError:
            messagebox.showwarning("Invalid value", "Enter numbers only (mm).")
            return
        self.send_gcode(commands.goto(targets))  # empty if no field is filled in

    def set_zero(self) -> None:
        """Declare the current physical position as X0 Y0 Z0 (G92). No motion."""
        if not self._require_connection():
            return
        if messagebox.askyesno(
            "Set zero",
            "Treat the CURRENT physical position as X0 Y0 Z0?\n\n"
            "Only do this with the grippers at your chosen reference "
            "(a small margin inside the mechanical stop). "
            "Software limits are measured from this point.",
        ):
            self.send_gcode(commands.set_zero())

    # ---------- Log / queue ----------
    def log_line(self, text: str) -> None:
        self.log_panel.append(text)

    def _handle_event(self, item) -> None:
        if not isinstance(item, tuple):
            self.log_line(item)
            return
        kind, *values = item
        if kind == "lost":  # ("lost", ser, error): port failed, e.g. USB unplugged
            ser, error = values
            if ser is self.printer.ser:  # ignore a late report from an old connection
                self._disconnected(f"!! Connection lost: {error}")
                messagebox.showerror("Connection lost",
                                     f"The serial port stopped working:\n{error}\n\n"
                                     "Check the USB cable, then connect again.")
        elif kind == "fault":  # ("fault", text): rest of the batch dropped by the connection
            self.log_line("!! Printer fault, remaining commands of this batch dropped")
        elif kind == "pos":  # ("pos", x, y, z): only Y and Z, X comes from dualx
            self.pos_panel.show_yz(*values[1:])
        elif kind == "dualx":  # ("dualx", x1, x2)
            self.pos_panel.show_dual_x(*values)

    def _poll_queue(self) -> None:
        while True:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_event(item)
        if not self.printer.connected:
            self.conn_panel.set_status("Offline")
        else:
            self.conn_panel.set_status("Ready" if self.printer_free else "Busy")
        self.after(QUEUE_POLL_MS, self._poll_queue)

    def _position_tick(self) -> None:
        """Periodically ask Marlin for its current position (M114) while connected.
        Background polls do not make the printer busy for the buttons."""
        if self.printer.connected and self.printer.idle:
            try:
                self.printer.send(commands.REPORT_POSITION, log=False, background=True)
            except RuntimeError:
                pass
        self.after(POSITION_INTERVAL_MS, self._position_tick)

    def on_close(self) -> None:
        self.printer.disconnect()
        self.destroy()


def main() -> None:
    App().mainloop()
