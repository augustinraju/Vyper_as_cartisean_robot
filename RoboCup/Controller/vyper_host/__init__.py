"""Host-side control of the Vyper gantry gripper (Marlin over USB serial).

Modules:
  config      - baud rate, timeouts, feedrate, poll interval
  protocol    - line framing (N/checksum) and reply parsing, pure functions
  connection  - PrinterConnection: threaded serial link with retransmission
  commands    - G-code sequences (jog, single X motor, home, goto, set zero)
  widgets     - Tk panels
  app         - the control window (needs tkinter)

Non-GUI use:
    import queue
    from vyper_host import PrinterConnection, commands
    link = PrinterConnection(queue.Queue())
    link.connect("COM3")
    link.send_lines(commands.home("Y"))
"""

from ...Controller.vyper_host import commands, protocol
from .connection import PrinterConnection, available_ports

__all__ = ["PrinterConnection", "available_ports", "commands", "protocol"]
