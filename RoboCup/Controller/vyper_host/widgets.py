"""Tk panels for the control GUI. Each panel only builds widgets and calls back;
the App decides what to send."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Any, Callable

from .config import DEFAULT_JOG_STEP_MM, JOG_STEPS_MM

PAD: dict[str, Any] = {"padx": 6, "pady": 4}
MONO = ("Consolas", 10)
POS_LABELS = {"X": "X mid:", "X1": "X1:", "X2": "X2:", "GAP": "Gap:", "Y": "Y:", "Z": "Z:"}


class ConnectionPanel(ttk.LabelFrame):
    def __init__(self, master, on_refresh: Callable[[], None], on_toggle: Callable[[], None]):
        super().__init__(master, text="Connection")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=0, **PAD)
        ttk.Button(self, text="Refresh", command=on_refresh).grid(row=0, column=1, **PAD)
        self.connect_btn = ttk.Button(self, text="Connect", command=on_toggle)
        self.connect_btn.grid(row=0, column=2, **PAD)
        self.status_var = tk.StringVar(value="Offline")
        ttk.Label(self, textvariable=self.status_var, width=8, font=MONO).grid(row=0, column=3, **PAD)

    def set_status(self, text: str) -> None:
        if self.status_var.get() != text:
            self.status_var.set(text)

    @property
    def port(self) -> str:
        return self.port_var.get()

    def set_ports(self, ports: list[str]) -> None:
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def set_connected(self, connected: bool) -> None:
        self.connect_btn.config(text="Disconnect" if connected else "Connect")


class PositionPanel(ttk.LabelFrame):
    def __init__(self, master):
        super().__init__(master, text="Position Commanded (mm)")
        self.vars = {key: tk.StringVar(value="--") for key in POS_LABELS}
        for i, (key, label) in enumerate(POS_LABELS.items()):
            ttk.Label(self, text=label).grid(row=0, column=2 * i, sticky="e", **PAD)
            ttk.Label(self, textvariable=self.vars[key], width=8, font=MONO).grid(
                row=0, column=2 * i + 1, sticky="w", **PAD)

    def show_yz(self, y: float, z: float) -> None:
        """X comes from show_dual_x (M114 X: is the logical X, not both motors)."""
        self.vars["Y"].set(f"{y:.2f}")
        self.vars["Z"].set(f"{z:.2f}")

    def show_dual_x(self, x1: float, x2: float) -> None:
        self.vars["X1"].set(f"{x1:.2f}")
        self.vars["X2"].set(f"{x2:.2f}")
        self.vars["X"].set(f"{(x1 + x2) / 2:.2f}")  # midpoint of the grippers
        self.vars["GAP"].set(f"{x2 - x1:.2f}")

    def clear(self) -> None:
        for var in self.vars.values():
            var.set("--")


class JogPanel(ttk.LabelFrame):
    """Y/X/Z jog buttons plus the step size (also used by XMotorPanel)."""

    def __init__(self, master, on_jog: Callable[[str, int], None]):
        super().__init__(master, text="Axis jog")
        self.step_var = tk.StringVar(value=DEFAULT_JOG_STEP_MM)
        ttk.Label(self, text="Step (mm):").grid(row=0, column=0, **PAD)
        ttk.Combobox(self, textvariable=self.step_var, width=6, state="readonly",
                     values=list(JOG_STEPS_MM)).grid(row=0, column=1, **PAD)

        buttons = [  # (text, axis, direction, row, column)
            ("Y+", "Y", 1, 1, 1), ("X-", "X", -1, 2, 0), ("X+", "X", 1, 2, 2),
            ("Y-", "Y", -1, 3, 1), ("Z+", "Z", 1, 1, 3), ("Z-", "Z", -1, 3, 3),
        ]
        for text, axis, direction, row, col in buttons:
            ttk.Button(self, text=text, width=6,
                       command=lambda a=axis, d=direction: on_jog(a, d)).grid(row=row, column=col, **PAD)

    @property
    def step(self) -> float:
        return float(self.step_var.get())


class XMotorPanel(ttk.LabelFrame):
    """Jog X1 or X2 alone. Calls on_jog(motor, direction) with motor 1 or 2."""

    def __init__(self, master, on_jog: Callable[[int, int], None]):
        super().__init__(master, text="Single X motor (X1 = T0, X2 = T1)")
        for col, (motor, direction) in enumerate(((1, -1), (1, 1), (2, -1), (2, 1))):
            text = f"X{motor}{'+' if direction > 0 else '-'}"
            ttk.Button(self, text=text, width=6,
                       command=lambda m=motor, d=direction: on_jog(m, d)).grid(row=0, column=col, **PAD)


class GripperPanel(ttk.LabelFrame):
    """Gripper gap (X2 - X1) around a fixed midpoint.

    on_step(direction): +1 open / -1 close by the jog step.
    on_set(): move to the gap typed in the entry (read it with target()).
    """

    def __init__(self, master, on_step: Callable[[int], None], on_set: Callable[[], None]):
        super().__init__(master, text="Gripper gap (both jaws at once, midpoint kept)")
        ttk.Button(self, text="Close", width=6, command=lambda: on_step(-1)).grid(row=0, column=0, **PAD)
        ttk.Button(self, text="Open", width=6, command=lambda: on_step(1)).grid(row=0, column=1, **PAD)
        ttk.Label(self, text="Gap (mm):").grid(row=0, column=2, sticky="e", **PAD)
        self.var = tk.StringVar()
        entry = ttk.Entry(self, textvariable=self.var, width=8)
        entry.grid(row=0, column=3, **PAD)
        entry.bind("<Return>", lambda _e: on_set())
        ttk.Button(self, text="Set gap", command=on_set).grid(row=0, column=4, **PAD)

    def target(self) -> float:
        """Typed gap in mm (',' accepted). Raises ValueError if empty, invalid or negative."""
        value = float(self.var.get().strip().replace(",", "."))
        if value < 0:
            raise ValueError("negative gap")
        return value


class HomePanel(ttk.LabelFrame):
    def __init__(self, master, on_home: Callable[[str], None], axes=("X1", "X2", "Y", "Z")):
        super().__init__(master, text="Home axis (G28)")
        for i, axis in enumerate(axes):
            ttk.Button(self, text=f"Home {axis}", width=9,
                       command=lambda a=axis: on_home(a)).grid(row=0, column=i, **PAD)


class ReferencePanel(ttk.LabelFrame):
    def __init__(self, master, on_set_zero: Callable[[], None]):
        super().__init__(master, text="Reference (no endstops)")
        ttk.Button(self, text="Set zero here (G92 X0 Y0 Z0)",
                   command=on_set_zero).grid(row=0, column=0, sticky="ew", **PAD)


class GotoPanel(ttk.LabelFrame):
    """Absolute target entries for X mid / Y / Z."""

    def __init__(self, master, on_send: Callable[[], None]):
        super().__init__(master, text="Go to position (mm, relative to zero)")
        self.vars = {key: tk.StringVar() for key in ("X", "Y", "Z")}
        for i, (key, var) in enumerate(self.vars.items()):
            label = "X mid:" if key == "X" else f"{key}:"
            ttk.Label(self, text=label).grid(row=0, column=2 * i, sticky="e", **PAD)
            entry = ttk.Entry(self, textvariable=var, width=8)
            entry.grid(row=0, column=2 * i + 1, **PAD)
            entry.bind("<Return>", lambda _e: on_send())
        ttk.Button(self, text="Send", command=on_send).grid(row=0, column=6, **PAD)
        ttk.Label(self, text="Leave a field empty to keep that axis unchanged.").grid(
            row=1, column=0, columnspan=7, sticky="w", **PAD)

    def targets(self) -> dict[str, float]:
        """Filled-in fields as floats (',' accepted as decimal point). Raises ValueError."""
        targets = {}
        for axis, var in self.vars.items():
            text = var.get().strip().replace(",", ".")
            if text:
                targets[axis] = float(text)
        return targets


class LogPanel(ttk.LabelFrame):
    def __init__(self, master, width: int = 44, height: int = 10):
        super().__init__(master, text="Log")
        self.text = tk.Text(self, width=width, height=height, state="disabled")
        self.text.pack(fill="both", expand=True, padx=4, pady=4)

    def append(self, line: str) -> None:
        self.text.config(state="normal")
        self.text.insert("end", line + "\n")
        self.text.see("end")
        self.text.config(state="disabled")


class CommandHistory:
    """Shell-like history: Up/Down walk through earlier entries, past the end is empty."""

    def __init__(self):
        self.items: list[str] = []
        self.pos = 0

    def add(self, line: str) -> None:
        if not self.items or self.items[-1] != line:
            self.items.append(line)
        self.pos = len(self.items)

    def step(self, delta: int) -> str | None:
        """Move by delta and return the entry there ('' past the end), None if empty."""
        if not self.items:
            return None
        self.pos = max(0, min(len(self.items), self.pos + delta))
        return self.items[self.pos] if self.pos < len(self.items) else ""


class CommandPanel(ttk.LabelFrame):
    """Free G-code entry. on_submit(text) returns the line actually sent (it goes
    into the history and the box is cleared), or None to keep the text for editing."""

    def __init__(self, master, on_submit: Callable[[str], str | None]):
        super().__init__(master, text="Send command")
        self.on_submit = on_submit
        self.history = CommandHistory()
        self.columnconfigure(0, weight=1)
        self.var = tk.StringVar()
        entry = ttk.Entry(self, textvariable=self.var, font=MONO)
        entry.grid(row=0, column=0, sticky="ew", **PAD)
        entry.bind("<Return>", lambda _e: self.submit())
        entry.bind("<Up>", lambda _e: self._recall(-1))
        entry.bind("<Down>", lambda _e: self._recall(1))
        ttk.Button(self, text="Send", command=self.submit).grid(row=0, column=1, **PAD)

    def submit(self) -> None:
        sent = self.on_submit(self.var.get())
        if sent is not None:
            self.history.add(sent)
            self.var.set("")

    def _recall(self, delta: int) -> str:
        text = self.history.step(delta)
        if text is not None:
            self.var.set(text)
        return "break"
