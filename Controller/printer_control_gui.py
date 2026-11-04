"""
Minimal axis control GUI for the Anycubic Vyper (Marlin firmware) over USB serial.

Requirements:
    pip install pyserial

Usage:
    python printer_control_gui.py
"""

import queue
import re
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUDRATE = 115200  # matches BAUDRATE in Marlin/Configuration.h
FEEDRATE = 1200    # mm/min for jog movesW
POSITION_INTERVAL_MS = 500  # how often M114 (current position) is polled
POSITION_RE = re.compile(
    r"X:(-?\d+\.?\d*)\s+Y:(-?\d+\.?\d*)\s+Z:(-?\d+\.?\d*)\s+E:(-?\d+\.?\d*)"
)
DUAL_X_RE = re.compile(r"X1:(-?\d+\.?\d*)\s+X2:(-?\d+\.?\d*)")  # per-motor X (mm)
POS_LABELS = {"X": "X mid:", "X1": "X1:", "X2": "X2:", "Y": "Y:", "Z": "Z:"}


class PrinterConnection:
    """Handles the serial connection and reads responses in a background thread."""

    def __init__(self, rx_queue: queue.Queue):
        self.ser = None
        self.rx_queue = rx_queue
        self._running = False
        self._lock = threading.Lock()
        self._silent_acks = 0  # "ok" replies belonging to silent (polling) commands

    def connect(self, port: str) -> None:
        self.ser = serial.Serial(port, BAUDRATE, timeout=0.5)
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def disconnect(self) -> None:
        self._running = False
        if self.ser:
            self.ser.close()
            self.ser = None

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send(self, line: str, log: bool = True) -> None:
        if not self.connected:
            raise RuntimeError("Not connected")
        if not log:
            with self._lock:
                self._silent_acks += 1
        self.ser.write((line.strip() + "\n").encode("ascii"))
        if log:
            self.rx_queue.put(f">> {line.strip()}")

    def _reader(self) -> None:
        while self._running and self.ser:
            try:
                raw = self.ser.readline()
            except serial.SerialException as exc:
                self.rx_queue.put(f"!! Serial error: {exc}")
                break
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            match = POSITION_RE.search(text)
            if match:
                # Position report from M114: forward to the GUI instead of the log.
                self.rx_queue.put(("pos", *map(float, match.group(1, 2, 3))))
                continue
            dual = DUAL_X_RE.search(text)
            if dual:
                self.rx_queue.put(("dualx", *map(float, dual.group(1, 2))))
                continue
            if text.startswith("Count "):
                continue  # M114 step-count line, sent on every poll: keep it out of the log
            if text.startswith("ok"):
                with self._lock:
                    if self._silent_acks:
                        self._silent_acks -= 1  # this ack belongs to a silent command
                        continue
            self.rx_queue.put(f"<< {text}")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Vyper Axis Control")
        self.resizable(False, False)

        self.rx_queue: queue.Queue = queue.Queue()
        self.printer = PrinterConnection(self.rx_queue)

        self.port_var = tk.StringVar()
        self.step_var = tk.StringVar(value="10")

        self._build_widgets()
        self.refresh_ports()
        self.after(100, self._poll_queue)
        self.after(POSITION_INTERVAL_MS, self._position_tick)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI ----------
    def _build_widgets(self) -> None:
        pad = {"padx": 6, "pady": 4}

        # Connection row
        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=0, **pad)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(row=0, column=1, **pad)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, **pad)

        # Current position monitor
        pos = ttk.LabelFrame(self, text="Position Commanded (mm)")
        pos.grid(row=1, column=0, sticky="ew", **pad)
        self.pos_vars = {key: tk.StringVar(value="--") for key in POS_LABELS}
        for i, (key, label) in enumerate(POS_LABELS.items()):
            ttk.Label(pos, text=label).grid(row=0, column=2 * i, sticky="e", **pad)
            ttk.Label(pos, textvariable=self.pos_vars[key], width=8,
                      font=("Consolas", 10)).grid(row=0, column=2 * i + 1, sticky="w", **pad)

        # Jog controls
        jog = ttk.LabelFrame(self, text="Axis jog")
        jog.grid(row=2, column=0, sticky="ew", **pad)

        ttk.Label(jog, text="Step (mm):").grid(row=0, column=0, **pad)
        ttk.Combobox(jog, textvariable=self.step_var, width=6, state="readonly",
                     values=["0.1", "1", "10", "50"]).grid(row=0, column=1, **pad)

        ttk.Button(jog, text="Y+", width=6, command=lambda: self.jog("Y", 1)).grid(row=1, column=1, **pad)
        ttk.Button(jog, text="X-", width=6, command=lambda: self.jog("X", -1)).grid(row=2, column=0, **pad)
        ttk.Button(jog, text="X+", width=6, command=lambda: self.jog("X", 1)).grid(row=2, column=2, **pad)
        ttk.Button(jog, text="Y-", width=6, command=lambda: self.jog("Y", -1)).grid(row=3, column=1, **pad)

        ttk.Button(jog, text="Z+", width=6, command=lambda: self.jog("Z", 1)).grid(row=1, column=3, **pad)
        ttk.Button(jog, text="Z-", width=6, command=lambda: self.jog("Z", -1)).grid(row=3, column=3, **pad)

        # ttk.Button(jog, text="Home all", command=lambda: self.send("G28")).grid(row=4, column=0, columnspan=4, sticky="ew", **pad)

        # Individual X motors (dual X carriage)
        x_motors = ttk.LabelFrame(self, text="Single X motor (X1 = T0, X2 = T1)")
        x_motors.grid(row=3, column=0, sticky="ew", **pad)
        ttk.Button(x_motors, text="X1-", width=6, command=lambda: self.jog_x_motor(0, -1)).grid(row=0, column=0, **pad)
        ttk.Button(x_motors, text="X1+", width=6, command=lambda: self.jog_x_motor(0, 1)).grid(row=0, column=1, **pad)
        ttk.Button(x_motors, text="X2-", width=6, command=lambda: self.jog_x_motor(1, -1)).grid(row=0, column=2, **pad)
        ttk.Button(x_motors, text="X2+", width=6, command=lambda: self.jog_x_motor(1, 1)).grid(row=0, column=3, **pad)

        # Reference (no endstops: position is set by hand)
        ref = ttk.LabelFrame(self, text="Reference (no endstops)")
        ref.grid(row=4, column=0, sticky="ew", **pad)
        ttk.Button(ref, text="Set zero here (G92 X0 Y0 Z0)",
                   command=self.set_zero).grid(row=0, column=0, sticky="ew", **pad)

        # Log
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=5, column=0, sticky="nsew", **pad)
        self.log = tk.Text(log_frame, width=44, height=10, state="disabled")
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

    # ---------- Serial ----------
    def refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.printer.connected:
            self.printer.disconnect()
            self.connect_btn.config(text="Connect")
            for var in self.pos_vars.values():
                var.set("--")
            self.log_line("Disconnected")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            self.printer.connect(port)
        except serial.SerialException as exc:
            messagebox.showerror("Connection failed", str(exc))
            return
        self.connect_btn.config(text="Disconnect")
        self.log_line(f"Connected to {port} @ {BAUDRATE}")

    def send(self, line: str) -> None:
        try:
            self.printer.send(line)
        except RuntimeError:
            messagebox.showwarning("Not connected", "Connect to the printer first.")

    def jog(self, axis: str, direction: int) -> None:
        distance = float(self.step_var.get()) * direction
        # Relative move, then restore absolute positioning mode.
        # With X_DUAL_STEPPER_DRIVERS, one G1 X moves X1 and X2 together.
        self.send("G91")
        self.send(f"G1 {axis}{distance:.2f} F{FEEDRATE}")
        self.send("G90")

    def set_zero(self) -> None:
        """Declare the current physical position as X0 Y0 Z0 (G92). No motion."""
        if not self.printer.connected:
            messagebox.showwarning("Not connected", "Connect to the printer first.")
            return
        if not messagebox.askyesno(
            "Set zero",
            "Treat the CURRENT physical position as X0 Y0 Z0?\n\n"
            "Only do this with the grippers at your chosen reference "
            "(a small margin inside the mechanical stop). "
            "Software limits are measured from this point.",
        ):
            return
        self.send("G92 X0 Y0 Z0")  # also resets the X1/X2 counters in the firmware
        self.send("M114")

    def jog_x_motor(self, motor: int, direction: int) -> None:
        """Move one X motor only. motor: 0 = X1, 1 = X2 (firmware M606 S1 X1|X2)."""
        distance = float(self.step_var.get()) * direction
        self.send(f"M606 S1 X{motor + 1}")  # Lock the other motor
        self.send("G91")
        self.send(f"G1 X{distance:.2f} F{FEEDRATE}")
        self.send("G90")
        self.send("M606 S0")  # Back to both motors; waits for the move to finish

    # ---------- Log / queue ----------
    def log_line(self, text: str) -> None:
        self.log.config(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _poll_queue(self) -> None:
        while True:
            try:
                item = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, tuple):
                kind, *values = item
                if kind == "pos":  # ("pos", x, y, z): only Y and Z, X comes from dualx
                    for key, value in zip("YZ", values[1:]):
                        self.pos_vars[key].set(f"{value:.2f}")
                elif kind == "dualx":  # ("dualx", x1, x2): X shown as midpoint of the grippers
                    x1, x2 = values
                    self.pos_vars["X1"].set(f"{x1:.2f}")
                    self.pos_vars["X2"].set(f"{x2:.2f}")
                    self.pos_vars["X"].set(f"{(x1 + x2) / 2:.2f}")
            else:
                self.log_line(item)
        self.after(100, self._poll_queue)

    def _position_tick(self) -> None:
        """Periodically ask Marlin for its current position (M114) while connected."""
        if self.printer.connected:
            try:
                self.printer.send("M114", log=False)
            except (RuntimeError, serial.SerialException):
                pass
        self.after(POSITION_INTERVAL_MS, self._position_tick)

    def on_close(self) -> None:
        self.printer.disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
