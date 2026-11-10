"""Serial link to Marlin with line numbers, checksums and retransmission.

Messages posted to rx_queue (consumed by the GUI or any other host):
  * str                      -> log line
  * ("pos", x, y, z)         -> M114 position report
  * ("dualx", x1, x2)        -> M114 per-motor X line
  * ("lost", ser, error_str) -> port failed (e.g. USB unplugged); threads stopped
  * ("fault", text)          -> endstop hit / firmware error (protocol.is_fault); every
                                line queued before it and not yet sent is dropped
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Iterable

import serial
import serial.tools.list_ports

from . import protocol
from .config import ACK_TIMEOUT_S, BAUDRATE, BOOT_DELAY_S


def available_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


class PrinterConnection:
    """Handles the serial connection and reads responses in a background thread.

    Every line is sent with a line number and checksum, one at a time (stop-and-wait):
      * corrupted line  -> Marlin answers "Error:checksum mismatch" + "Resend: N" -> resent
      * lost line / lost "ok" -> after ACK_TIMEOUT_S the same numbered line is resent.
        If Marlin already ran it, it rejects the duplicate number and asks for N+1,
        so a move is never executed twice.
    """

    def __init__(self, rx_queue: queue.Queue, baudrate: int = BAUDRATE,
                 ack_timeout_s: float = ACK_TIMEOUT_S, boot_delay_s: float = BOOT_DELAY_S):
        self.ser = None
        self.rx_queue = rx_queue
        self.baudrate = baudrate
        self.ack_timeout_s = ack_timeout_s
        self.boot_delay_s = boot_delay_s
        self._stop = threading.Event()
        self._threads: list = []
        self._ready_at = 0.0
        self._lock = threading.Lock()
        self._tx_queue: queue.Queue = queue.Queue()
        self._ack = threading.Event()  # set when Marlin answers "ok"
        self._resend = None  # line number from the last "Resend:" request
        # Lines queued or in flight: all of them / without background polls
        self._pending = 0
        self._pending_fg = 0
        self._last_rx = 0.0  # time of the last "busy:" keepalive
        # Bumped on every fault; queued lines from an older epoch are dropped unsent
        self._epoch = 0
        # Drain after a timeout (see _drain): replies are discarded until the marker returns
        self._drain_seq = 0
        self._drain_token = None
        self._drain_marker_seen = False
        self._drain_done = threading.Event()

    # ---------- Public API ----------
    @property
    def busy(self) -> bool:
        """Lines other than background polls are queued or not yet acknowledged.

        Marlin acks a G1 once it is planned, not finished; end a batch with M400
        so that "not busy" also means "not moving".
        """
        with self._lock:
            return self._pending_fg > 0

    @property
    def idle(self) -> bool:
        """Nothing at all queued or in flight (background polls included)."""
        with self._lock:
            return self._pending == 0

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str) -> None:
        # Fresh state per connection; each connection's threads have their own stop
        # event, so threads of an earlier connection can never run alongside new ones.
        self._tx_queue = queue.Queue()
        self._ack = threading.Event()
        self._resend = None
        self._drain_token = None
        self._reset_pending()
        self.ser = serial.Serial(port, self.baudrate, timeout=0.5, write_timeout=5)
        self._stop = threading.Event()
        self._ready_at = time.monotonic() + self.boot_delay_s
        self._threads = [
            threading.Thread(target=self._reader, args=(self.ser, self._stop), daemon=True),
            threading.Thread(target=self._writer, args=(self.ser, self._stop), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def disconnect(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)  # reader wakes within its 0.5 s read timeout
        self._threads = []
        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass  # port already gone (unplugged)
            self.ser = None
        self._reset_pending()  # unsent lines are dropped with the connection

    def send(self, line: str, log: bool = True, background: bool = False) -> None:
        """Queue one line. Raises RuntimeError if not connected, ValueError if not ASCII.

        background=True (position polls) does not count towards `busy`.
        """
        if not self.connected:
            raise RuntimeError("Not connected")
        line = line.strip()
        if not line.isascii():
            # The checksum and the port only take ASCII ("ü", "°", ... are not G-code)
            raise ValueError(f"Non-ASCII character in command: {line}")
        with self._lock:
            self._pending += 1
            if not background:
                self._pending_fg += 1
            epoch = self._epoch
        self._tx_queue.put((line, log, background, epoch))

    def send_lines(self, lines: Iterable[str], log: bool = True) -> None:
        """Queue several lines; all are checked first, so none are queued on error."""
        lines = [line.strip() for line in lines]
        if not self.connected:
            raise RuntimeError("Not connected")
        for line in lines:
            if not line.isascii():
                raise ValueError(f"Non-ASCII character in command: {line}")
        for line in lines:
            self.send(line, log)

    def _reset_pending(self) -> None:
        with self._lock:
            self._pending = 0
            self._pending_fg = 0

    def _line_done(self, background: bool) -> None:
        with self._lock:
            self._pending = max(0, self._pending - 1)
            if not background:
                self._pending_fg = max(0, self._pending_fg - 1)

    # ---------- Writer side ----------
    def _lost(self, ser, stop: threading.Event, exc) -> None:
        """The port failed (e.g. USB unplugged): stop both threads and tell the GUI once."""
        if not stop.is_set():
            stop.set()
            self.rx_queue.put(("lost", ser, str(exc)))

    def _write_raw(self, ser, stop: threading.Event, data: str) -> bool:
        try:
            ser.write(data.encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self._lost(ser, stop, exc)  # a failed write means the port is gone
            return False

    def _wait(self, event: threading.Event, stop: threading.Event) -> bool:
        """Wait for event; "busy:" keepalives push the deadline out. True if it was set."""
        while not stop.is_set() and not event.wait(timeout=0.2):
            if time.monotonic() - self._last_rx > self.ack_timeout_s:
                return False
        return event.is_set()

    def _drain(self, ser, stop: threading.Event) -> None:
        """Discard replies still owed to earlier transmissions after a timeout.

        After a timeout, line n was sent twice. Marlin may answer both: the original
        with a late "ok", the copy with "Resend: n+1" + "ok" (duplicate rejected
        on reading, so this can even come first). Unconsumed, these would ack the
        following lines one step early. Marlin answers in order, so everything
        before the echo of an unnumbered M118 marker is stale.
        """
        self._drain_seq += 1
        token = f"HOSTSYNC{self._drain_seq}"
        self._drain_marker_seen = False
        self._drain_done.clear()
        self._drain_token = token  # from here on the reader discards ok/Resend
        try:
            if not self._write_raw(ser, stop, f"M118 {token}\n"):
                return
            self._last_rx = time.monotonic()
            if not self._wait(self._drain_done, stop) and not stop.is_set():
                self.rx_queue.put("!! No reply to the resync marker, continuing")
        finally:
            self._drain_token = None

    def _transact(self, ser, stop: threading.Event, n: int, line: str) -> bool:
        """Send numbered line n until Marlin accepts it.

        Returns True when accepted, False on stop/serial failure.
        """
        attempt = 0
        timed_out = False
        while not stop.is_set():
            with self._lock:
                self._resend = None
            self._ack.clear()
            if not self._write_raw(ser, stop, protocol.frame(n, line) + "\n"):
                return False  # port lost (_write_raw stopped the threads)
            self._last_rx = time.monotonic()
            # Wait for "ok". M606 / full planner can block for a while.
            acked = self._wait(self._ack, stop)
            if stop.is_set():
                return False
            if not acked:
                attempt += 1
                timed_out = True
                self.rx_queue.put(f"!! No reply to N{n} {line}, resending ({attempt})")
                continue
            with self._lock:
                resend = self._resend
            if resend is None or resend == n + 1:
                # Accepted (n+1 = duplicate of an already-run line). After a timeout,
                # drop the replies still owed to the extra transmissions.
                if timed_out:
                    self._drain(ser, stop)
                return True
            attempt += 1
            self.rx_queue.put(f"!! Printer asked to resend N{resend}, resending {line} ({attempt})")
            if resend != n:
                # Marlin expects an earlier/later number than ours: resync the counter
                # instead of replaying old moves (only this line is retransmitted).
                self._resync(ser, stop, n - 1)
        return False

    def _resync(self, ser, stop: threading.Event, last_n: int) -> None:
        """Tell Marlin the last line number was last_n (unnumbered M110 is always accepted)."""
        self._ack.clear()
        if self._write_raw(ser, stop, f"M110 N{last_n}\n"):
            self._ack.wait(timeout=2)

    def _writer(self, ser, stop: threading.Event) -> None:
        """Send one numbered line at a time and wait for Marlin's "ok"."""
        try:
            self._writer_loop(ser, stop)
        except (serial.SerialException, OSError) as exc:
            self._lost(ser, stop, exc)

    def _writer_loop(self, ser, stop: threading.Event) -> None:
        # Opening the port may reset the board (DTR); wait for it to boot.
        while not stop.is_set() and time.monotonic() < self._ready_at:
            time.sleep(0.1)
        if stop.is_set():
            return
        ser.reset_input_buffer()
        # A bare newline terminates any partial line left in Marlin's RX buffer
        # (that leftover is what produced "MM114"), then reset the line counter.
        if not self._write_raw(ser, stop, "\n"):
            return
        time.sleep(0.2)
        ser.reset_input_buffer()
        n = 0
        self._resync(ser, stop, n)
        while not stop.is_set():
            try:
                line, log, background, epoch = self._tx_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            with self._lock:
                stale = epoch != self._epoch
            if stale:
                # Queued before a fault (endstop hit / error): the batch is aborted
                if log:
                    self.rx_queue.put(f"!! Dropped after fault: {line}")
                self._line_done(background)
                continue
            if not line.isascii():
                # send() rejects these already; never let one kill this thread
                self.rx_queue.put(f"!! Skipped non-ASCII command: {line}")
                self._line_done(background)
                continue
            n += 1
            if log:
                self.rx_queue.put(f">> {line}")
            if not self._transact(ser, stop, n, line):
                break  # stopped / port lost: connect() and disconnect() reset the counts
            self._line_done(background)  # "ok" received: only now may busy drop
            if log:
                self.rx_queue.put("<< ok")

    # ---------- Reader side ----------
    def _reader(self, ser, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as exc:
                self._lost(ser, stop, exc)  # USB unplugged: stop the writer too
                break
            if raw:
                self._handle_line(raw.decode("ascii", errors="replace").strip())

    def _handle_line(self, text: str) -> None:
        token = self._drain_token
        if token is not None:
            # Draining after a timeout (_drain): drop stale ok/Resend replies
            # until the marker echo, then take the marker's own "ok".
            if token in text:
                self._drain_marker_seen = True
                return
            if protocol.is_ok(text):
                if self._drain_marker_seen:
                    self._drain_done.set()
                return
            if protocol.parse_resend(text) is not None:
                return
        position = protocol.parse_position(text)
        if position:
            # Position report from M114: forward to the GUI instead of the log.
            self.rx_queue.put(("pos", *position))
            return
        dual = protocol.parse_dual_x(text)
        if dual:
            self.rx_queue.put(("dualx", *dual))
            return
        if protocol.is_step_count(text):
            return  # sent on every poll: keep it out of the log
        if protocol.is_ok(text):
            self._ack.set()  # the writer logs "<< ok" for non-silent commands
            return
        resend = protocol.parse_resend(text)
        if resend is not None:
            with self._lock:
                self._resend = resend  # acted on when the "ok" follows
            return
        if protocol.is_fault(text):
            with self._lock:
                self._epoch += 1  # writer drops everything queued so far
            self.rx_queue.put(f"<< {text}")
            self.rx_queue.put(("fault", text))
            return
        if protocol.is_busy(text):
            self._last_rx = time.monotonic()  # Marlin is alive, just still working
        self.rx_queue.put(f"<< {text}")
