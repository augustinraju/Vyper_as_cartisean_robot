"""
Minimal axis control GUI for the Anycubic Vyper (Marlin firmware) over USB serial.

Requirements:
    pip install pyserial

Usage:
    python printer_control_gui.py

The code lives in the vyper_host package next to this file.
"""

from vyper_host.app import main

if __name__ == "__main__":
    main()
