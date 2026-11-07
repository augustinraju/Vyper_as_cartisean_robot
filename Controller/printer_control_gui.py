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
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUDRATE = 115200  # matches BAUDRATE in Marlin/Configuration.h
BOOT_DELAY_S = 3.0  # wait after opening the port (board resets) before sending
FEEDRATE = 1200   # mm/min for jog movesW
POSITION_INTERVAL_MS = 500  # how often M114 (current position) is polled
POSITION_RE = re.compile(
    r"X:(-?\d+\.?\d*)\s+Y:(-?\d+\.?\d*)\s+Z:(-?\d+\.?\d*)\s+E:(-?\d+\.?\d*)"
)
DUAL_X_RE = re.compile(r"X1:(-?\d+\.?\d*)\s+X2:(-?\d+\.?\d*)")  # per-motor X (mm)
RESEND_RE = re.compile(r"^(?:Resend|rs)[:\s]+N?(\d+)", re.IGNORECASE)
ACK_TIMEOUT_S = 10.0  # no "ok" (or "busy:") for this long -> retransmit the same line
POS_LABELS = {"X": "X mid:", "X1": "X1:", "X2": "X2:", "Y": "Y:", "Z": "Z:"}


def frame(n: int, line: str) -> str:
    """Marlin host protocol: 'N<n> <cmd>*<xor checksum>'."""
    body = f"N{n} {line}"
    checksum = 0
    for ch in body.encode("ascii"):
        checksum ^= ch
    return f"{body}*{checksum}"


class PrinterConnection:
    """Handles the serial connection and reads responses in a background thread.

    Every line is sent with a line number and checksum, one at a time (stop-and-wait):
      * corrupted line  -> Marlin answers "Error:checksum mismatch" + "Resend: N" -> resent
      * lost line / lost "ok" -> after ACK_TIMEOUT_S the same numbered line is resent.
        If Marlin already ran it, it rejects the duplicate number and asks for N+1,
        so a move is never executed twice.
    """

    def __init__(self, rx_queue: queue.Queue):
        self.ser = None
        self.rx_queue = rx_queue
        self._stop = threading.Event()
        self._threads: list = []
        self._ready_at = 0.0
        self._lock = threading.Lock()
        self._tx_queue: queue.Queue = queue.Queue()
        self._ack = threading.Event()  # set when Marlin answers "ok"
        self._resend = None  # line number from the last "Resend:" request
        self._last_rx = 0.0  # time of the last "busy:" keepalive

    @property
    def busy(self) -> bool:
        return not self._tx_queue.empty()

    def connect(self, port: str) -> None:
        # Fresh state per connection; each connection's threads have their own stop
        # event, so threads of an earlier connection can never run alongside new ones.
        self._tx_queue = queue.Queue()
        self._ack = threading.Event()
        self._resend = None
        self.ser = serial.Serial(port, BAUDRATE, timeout=0.5, write_timeout=5)
        self._stop = threading.Event()
        self._ready_at = time.monotonic() + BOOT_DELAY_S
        self._threads = [
            threading.Thread(target=self._reader, args=(self.ser, self._stop), daemon=True),
            threading.Thread(target=self._writer, args=(self.ser, self._stop), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def _write_raw(self, ser, data: str) -> bool:
        try:
            ser.write(data.encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self.rx_queue.put(f"!! Serial error: {exc}")
            return False

    def _transact(self, ser, stop: threading.Event, n: int, line: str):
        """Send numbered line n until Marlin accepts it.

        Returns True when accepted, False on stop/serial failure.
        """
        attempt = 0
        while not stop.is_set():
            with self._lock:
                self._resend = None
            self._ack.clear()
            if not self._write_raw(ser, frame(n, line) + "\n"):
                if stop.is_set() or not ser.is_open:
                    return False
                time.sleep(0.5)
                continue
            self._last_rx = time.monotonic()
            # Wait for "ok". M606 / full planner can block for a while; "busy:"
            # keepalives (HOST_KEEPALIVE_FEATURE) push the deadline out.
            while not stop.is_set() and not self._ack.wait(timeout=0.2):
                if time.monotonic() - self._last_rx > ACK_TIMEOUT_S:
                    break
            if stop.is_set():
                return False
            if not self._ack.is_set():
                attempt += 1
                self.rx_queue.put(f"!! No reply to N{n} {line}, resending ({attempt})")
                continue
            with self._lock:
                resend = self._resend
            if resend is None or resend == n + 1:
                return True  # accepted (n+1 = duplicate of an already-run line)
            attempt += 1
            self.rx_queue.put(f"!! Printer asked to resend N{resend}, resending {line} ({attempt})")
            if resend != n:
                # Marlin expects an earlier/later number than ours: resync the counter
                # instead of replaying old moves (only this line is retransmitted).
                self._resync(ser, n - 1)
        return False

    def _resync(self, ser, last_n: int) -> None:
        """Tell Marlin the last line number was last_n (unnumbered M110 is always accepted)."""
        self._ack.clear()
        if self._write_raw(ser, f"M110 N{last_n}\n"):
            self._ack.wait(timeout=2)

    def _writer(self, ser, stop: threading.Event) -> None:
        """Send one numbered line at a time and wait for Marlin's "ok"."""
        # Opening the port may reset the board (DTR); wait for it to boot.
        while not stop.is_set() and time.monotonic() < self._ready_at:
            time.sleep(0.1)
        ser.reset_input_buffer()
        # A bare newline terminates any partial line left in Marlin's RX buffer
        # (that leftover is what produced "MM114"), then reset the line counter.
        self._write_raw(ser, "\n")
        time.sleep(0.2)
        ser.reset_input_buffer()
        n = 0
        self._resync(ser, n)
        while not stop.is_set():
            try:
                line, log = self._tx_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            n += 1
            if log:
                self.rx_queue.put(f">> {line}")
            if not self._transact(ser, stop, n, line):
                break
            if log:
                self.rx_queue.put("<< ok")

    def disconnect(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)  # reader wakes within its 0.5 s read timeout
        self._threads = []
        if self.ser:
            self.ser.close()
            self.ser = None

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def send(self, line: str, log: bool = True) -> None:
        if not self.connected:
            raise RuntimeError("Not connected")
        self._tx_queue.put((line.strip(), log))

    def _reader(self, ser, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as exc:
                if not stop.is_set():
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
                self._ack.set()  # the writer logs "<< ok" for non-silent commands
                continue
            resend = RESEND_RE.match(text)
            if resend:
                with self._lock:
                    self._resend = int(resend.group(1))  # acted on when the "ok" follows
                continue
            if "busy:" in text:
                self._last_rx = time.monotonic()  # Marlin is alive, just still working
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

        # Manual absolute positioning
        goto = ttk.LabelFrame(self, text="Go to position (mm, relative to zero)")
        goto.grid(row=5, column=0, sticky="ew", **pad)
        self.goto_vars = {key: tk.StringVar() for key in ("X", "Y", "Z")}
        for i, (key, var) in enumerate(self.goto_vars.items()):
            label = "X mid:" if key == "X" else f"{key}:"
            ttk.Label(goto, text=label).grid(row=0, column=2 * i, sticky="e", **pad)
            entry = ttk.Entry(goto, textvariable=var, width=8)
            entry.grid(row=0, column=2 * i + 1, **pad)
            entry.bind("<Return>", lambda _e: self.goto_position())
        ttk.Button(goto, text="Send", command=self.goto_position).grid(row=0, column=6, **pad)
        ttk.Label(goto, text="Leave a field empty to keep that axis unchanged.").grid(
            row=1, column=0, columnspan=7, sticky="w", **pad)

        # Log
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=6, column=0, sticky="nsew", **pad)
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

    def goto_position(self) -> None:
        """Move to typed X mid / Y / Z targets (empty field = axis untouched)."""
        if not self.printer.connected:
            messagebox.showwarning("Not connected", "Connect to the printer first.")
            return
        targets = {}
        try:
            for axis, var in self.goto_vars.items():
                text = var.get().strip().replace(",", ".")
                if text:
                    targets[axis] = float(text)
        except ValueError:
            messagebox.showwarning("Invalid value", "Enter numbers only (mm).")
            return
        if not targets:
            return

        words = []
        relative_x = False
        if "X" in targets:
            try:
                mid = float(self.pos_vars["X"].get())
                # Move both motors by the distance from the current midpoint, so a
                # previous single-motor offset between X1 and X2 is preserved.
                words.append(f"X{targets['X'] - mid:.2f}")
                relative_x = True
            except ValueError:
                words.append(f"X{targets['X']:.2f}")  # no midpoint known: absolute X
        if "Y" in targets:
            words.append(f"Y{targets['Y']:.2f}")
        if "Z" in targets:
            words.append(f"Z{targets['Z']:.2f}")

        if relative_x:
            # G91 would also make Y/Z relative, so send X relative, then Y/Z absolute.
            self.send("G91")
            self.send(f"G1 {words[0]} F{FEEDRATE}")
            self.send("G90")
            words = words[1:]
        if words:
            self.send(f"G1 {' '.join(words)} F{FEEDRATE}")
        self.send("M114")

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
        if self.printer.connected and not self.printer.busy:
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
