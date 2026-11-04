# Vyper Gantry Robot

A 3D printer turned into a small **XYZ gantry robot**. This repository holds a modified Marlin firmware for the Anycubic Vyper, plus a desktop GUI for moving it from a PC.

The extruder and hotend are removed. The printer's motion system (two X motors, Y, Z) is kept and driven over USB serial with ordinary G-code. Anything that can send G-code can move it, whether that is the included GUI, a script, or a robotics stack.

> **Status: work in progress.**

## Why this exists

A Vyper has a rigid frame, stepper drivers and a capable controller board at a low price. Without the hotend it makes a cheap positioning gantry. The main change is that the **X axis has two motors**, one on each side of the frame. The firmware can move them together or one at a time, which stock Marlin does not offer in this form.

## How it works

```
 PC / script / GUI  --USB serial, G-code-->  Vyper mainboard (modified Marlin)
                                               |-- X1 motor (X driver)
                                               |-- X2 motor (E0 driver, repurposed)
                                               |-- Y motor
                                               '-- Z motors (Z1, and Z2 on the E1 driver)
```

- **Base:** Marlin 2.x, from the `vyper` branch of ANYCUBIC-3D/Vyper firmware (release 2.4.5).
- **Board:** `BOARD_AC_TRI_F1_V1`, a TriGorilla-style STM32F103RE board (PlatformIO env `ac_tri_f1`).
- **No heaters, no extruder.** Temperature sensing is replaced by a dummy sensor.
- **Two X motors.** X1 uses the X driver. X2 uses the E0 driver pins through an extension cable. The Z2 motor uses the E1 driver (the ZR connector).

### Firmware features added

- **Independent X motors** (`X_DUAL_STEPPER_DRIVERS`) with a new G-code, `M606`:
  - `M606 S0`: both X motors move together (default).
  - `M606 S1 X1` or `M606 S1 X2`: only that motor moves.
  - After a single-motor move the two motors are physically apart. Send `M606 S0` before combined moves.
- **Per-motor position tracking.** `M114` prints an extra line `X1:<mm> X2:<mm>`.
- **X2 direction handling.** X2 shares its direction pin with the E0 driver, so the firmware rewrites it after each direction update. `INVERT_X2_VS_X_DIR` flips the X2 motor if it runs backwards.
- **Fixed work area.** `X_BED_SIZE` is 130 mm and `Y_BED_SIZE` is 255 mm. Both are unverified.
- The Anycubic touchscreen UI still compiles, but its probe, leveling and flow pages do not work.

## Safety: read before powering the printer

- **Never send `M104`, `M109`, `M140` or `M190`.** The heater pins are still wired and there is no real temperature feedback.
- **Never send extruder moves** (`E` moves). The E0 and E1 drivers move X2 and Z2.
- **Do not home (`G28`) until the X endstops are confirmed.** The firmware enables X min and max endstop inputs, but it is not yet confirmed that physical switches exist at the X ends. The GUI's "Home all" button sends `G28`.
- Be ready to cut power the first time you flash and move the printer.

## Quick start

### 1. Build the firmware

Requires [PlatformIO](https://platformio.org/).

```
pio run -e ac_tri_f1
```

The image is written to `.pio/build/ac_tri_f1/debug/firmware.bin`. The environment builds in `debug` mode, so use a release build for a normal flashing image. If the build fails with SCons file-write errors, run a clean build.

Flash the image with the vendor tool (`go-firmware-release_0.2.0_Windows_x86_64/`) or your usual STM32 flashing method.

### 2. Talk to the printer

Connect over USB at **115200 baud**. The port is usually COM3 or COM4 on Windows and can change when the USB adapter is replugged. Then try:

```
M115                 ; firmware info
M114                 ; position, including the X1:<mm> X2:<mm> line
G1 Z5 F300           ; move Z only
G1 X5 F1200          ; both X motors should move in the same direction
```

If one X motor runs the wrong way, flip `INVERT_X2_VS_X_DIR` in `Marlin/Configuration_adv.h` and rebuild.

### 3. Use the jogging GUI

```
pip install pyserial
python Controller/printer_control_gui.py
```

The GUI offers jog buttons for X, Y and Z (step sizes 0.1, 1, 10 and 50 mm), buttons for each X motor on its own (X1 and X2), and a live position panel (X mid, X1, X2, Y, Z). It runs on the PC only and does not affect the firmware.

## Project status

| Item | State |
|------|-------|
| Firmware builds (`ac_tri_f1`) | Done |
| Hotend, bed and leveling features disabled for the robot setup | Done |
| `M606` single-motor X control | Done |
| Jogging GUI | Done |
| Physical X endstops | Unverified |
| Bed size, steps/mm, LCD settings | Unverified |


## Where things are

| Path | What it is |
|------|------------|
| `Marlin/Configuration.h` | Board, axis limits, endstops, disabled heater and leveling features |
| `Marlin/Configuration_adv.h` | X2 and Z2 pin mapping, dual X driver setup, `INVERT_X2_VS_X_DIR` |
| `Marlin/src/module/stepper.cpp` | Step and direction logic, per-motor counters, motor locks |
| `Marlin/src/gcode/control/M606.cpp` | The `M606` command |
| `Marlin/src/gcode/host/M114.cpp` | Position report with X1/X2 |
| `Marlin/src/lcd/extui/lib/anycubic_dgus/` | Touchscreen UI |
| `Controller/printer_control_gui.py` | Jogging GUI |
| `platformio.ini` | Build environment `ac_tri_f1` |
| `buildroot/`, `docker/`, `docs/`, `config/`, `data/www/` | Marlin support material |

## Contributing

Open an issue or pull request. Please test changes on the hardware where you can, and state in the description whether they were flashed or only compiled. The project does not yet have a hardware test routine beyond the checks in [Quick start](#2-talk-to-the-printer).

## Credits and license

Built on [Marlin](https://marlinfw.org/) by the Marlin team and the [Anycubic Vyper firmware](https://github.com/ANYCUBIC-3D/Vyper). Marlin is published under the [GPL](LICENSE).
