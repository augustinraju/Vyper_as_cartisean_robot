"""Host-side settings for talking to the Vyper (Marlin) over USB serial."""

BAUDRATE = 115200  # matches BAUDRATE in Marlin/Configuration.h
BOOT_DELAY_S = 3.0  # wait after opening the port (board resets) before sending
ACK_TIMEOUT_S = 10.0  # no "ok" (or "busy:") for this long -> retransmit the same line
FEEDRATE = 1200  # mm/min for jog and go-to moves
POSITION_INTERVAL_MS = 500  # how often M114 (current position) is polled
JOG_STEPS_MM = ("0.1", "1", "10", "50")
DEFAULT_JOG_STEP_MM = "10"
