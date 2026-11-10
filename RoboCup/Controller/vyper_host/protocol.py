"""Marlin host protocol: line framing and parsing of the printer's replies.

Pure functions only (no serial, no GUI), so they can be reused and tested on their own.
"""

from __future__ import annotations

import re

POSITION_RE = re.compile(
    r"X:(-?\d+\.?\d*)\s+Y:(-?\d+\.?\d*)\s+Z:(-?\d+\.?\d*)\s+E:(-?\d+\.?\d*)"
)
DUAL_X_RE = re.compile(r"X1:(-?\d+\.?\d*)\s+X2:(-?\d+\.?\d*)")  # per-motor X (mm)
RESEND_RE = re.compile(r"^(?:Resend|rs)[:\s]+N?(\d+)", re.IGNORECASE)

# Replies after which the rest of a command batch must not run (see is_fault)
FAULT_RE = re.compile(
    r"endstops hit"                      # endstop hit outside homing (M120): move cut short
    r"|^Error:"                          # firmware error (kill, homing failed, ...)
    r"|Unknown command"                  # typo / unsupported G-code
    r"|M606 \w+ (?:must|needs)",         # M606 rejected: X mode not changed (M606.cpp)
)
# Line-transport errors: repaired by retransmission ("Resend:"), not faults
TRANSPORT_ERRORS = ("checksum mismatch", "Line Number is not Last Line Number",
                    "No Checksum with line number", "No Line Number with checksum")


def checksum(text: str) -> int:
    """XOR of all bytes, as Marlin expects after '*'."""
    value = 0
    for ch in text.encode("ascii"):
        value ^= ch
    return value


def frame(n: int, line: str) -> str:
    """Marlin host protocol: 'N<n> <cmd>*<xor checksum>'."""
    body = f"N{n} {line}"
    return f"{body}*{checksum(body)}"


def is_ok(text: str) -> bool:
    return text.startswith("ok")


def is_busy(text: str) -> bool:
    """HOST_KEEPALIVE_FEATURE message: Marlin is alive, just still working."""
    return "busy:" in text


def is_step_count(text: str) -> bool:
    """M114 step-count line ('Count X:... '), sent on every poll."""
    return text.startswith("Count ")


def is_fault(text: str) -> bool:
    """A reply after which the remaining lines of the batch must be dropped."""
    return bool(FAULT_RE.search(text)) and not any(e in text for e in TRANSPORT_ERRORS)


def parse_resend(text: str) -> int | None:
    """Line number from 'Resend: N' / 'rs N', else None."""
    match = RESEND_RE.match(text)
    return int(match.group(1)) if match else None


def parse_position(text: str) -> tuple[float, float, float] | None:
    """(X, Y, Z) from an M114 report, else None."""
    match = POSITION_RE.search(text)
    return tuple(map(float, match.group(1, 2, 3))) if match else None


def parse_dual_x(text: str) -> tuple[float, float] | None:
    """(X1, X2) in mm from the M114 'X1:<mm> X2:<mm>' line, else None."""
    match = DUAL_X_RE.search(text)
    return tuple(map(float, match.group(1, 2))) if match else None
