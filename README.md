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

### Wiring in detail

| Motor | Driver on the board | Firmware pins | Notes |
|-------|--------------------|---------------|-------|
| X1 | X driver | `X_STEP/DIR/ENABLE` | Unchanged from the stock printer. |
| X2 | E0 driver (extension cable) | `X2_*_PIN` = `E0_*_PIN` | The extruder output is reused to drive the second X motor. |
| Y | Y driver | unchanged | |
| Z1 | Z driver | unchanged | Z homing uses Z1 only (`Z_MULTI_ENDSTOPS` is off). |
| Z2 | E1 driver (ZR connector) | `Z2_*_PIN` = `E1_*_PIN` | Follows Z1. |

The two X motors sit opposite each other, with their top portions facing each other. Because of this mirrored mounting, one motor must turn the opposite way to the other for the carriage to move in a straight line. `INVERT_X2_VS_X_DIR` handles this.

Since the E0 and E1 drivers now belong to X2 and Z2, the firmware has no working extruder. This is why `E` moves must never be sent (see Safety).

### Firmware features added

- **Independent X motors** (`X_DUAL_STEPPER_DRIVERS`) with a new G-code, `M606`:
  - `M606 S0`: both X motors move together (default).
  - `M606 S1 X1` or `M606 S1 X2`: only that motor moves.
  - After a single-motor move the two motors are physically apart. Send `M606 S0` before combined moves.
  - `M606` waits for all queued moves to finish (`planner.synchronize()`) before it changes the mode, so earlier moves are not affected.
  - Why it is useful: moving one motor alone lets you square the gantry, test each motor, or take up slack. Locking a motor only skips its step pulses. The firmware still thinks the carriage moved, so the planner position and the real position differ until you return to `M606 S0` and correct the offset.
  - It uses `X_DUAL_STEPPER_DRIVERS`, not `DUAL_X_CARRIAGE`. The two options exclude each other in Marlin, and `M605` (duplication mode) is therefore not available.
- **Per-motor position tracking.** `M114` prints an extra line `X1:<mm> X2:<mm>`. Each motor has its own step counter, updated in the step-pulse path and skipped while that motor is locked. Homing and `G92 X` reset both counters. The "commanded" X position in the GUI (`X mid`) is the average of the two.
- **X2 direction handling.** X2 shares its direction pin (PB3) with the E0 driver, so a pure X move would otherwise leave the pin in the wrong state. The firmware rewrites the X2 direction at the end of `Stepper::set_directions()` and XORs it with `INVERT_X2_VS_X_DIR`. If one X motor runs backwards, flip that option. This change compiles but has not yet been tested on the printer.
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

#### Install the Vyper STM32 framework package

The board needs the Vyper variant of the Arduino STM32 core, `framework-arduinoststm32_vyper-main/`. It contains the `AC_TRI_F103RE` variant, `boards.txt` and `ac_tri_f103re.json`, which the stock PlatformIO `framework-arduinoststm32` package does not have. Its `package.json` declares the name `framework-arduinoststm32` (version 4.10900.200819), so it replaces the stock package under that name.

1. Get the `framework-arduinoststm32_vyper-main/` folder. It is untracked in git and may not be in your clone.
2. Copy its contents into the PlatformIO packages folder, replacing the existing `framework-arduinoststm32` folder:
   - Windows: `C:\Users\<you>\.platformio\packages\framework-arduinoststm32\`
   - Linux/macOS: `~/.platformio/packages/framework-arduinoststm32/`

   The target folder must directly contain `package.json`, `boards.txt`, `cores/`, `variants/` and `system/`, not a nested subfolder. Run PlatformIO once first (`pio run -e ac_tri_f1`) so it creates the folder, then overwrite it. Close VS Code or PlatformIO while copying.
3. Build again. If PlatformIO later re-downloads the stock package and overwrites your copy, copy the folder again.

The pre-build script `buildroot/share/PlatformIO/scripts/generic_create_variant.py` also copies the board JSON and the `AC_TRI_F103RE` variant from this repository into the framework on every build.

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

How it works: the GUI opens the serial port at 115200 baud and polls `M114` every 500 ms. A reader thread parses the replies. It reads X, Y and Z from the standard position line and X1 and X2 from the extra line. It hides the `Count ...` line and the `ok` replies to its own polls so the log stays readable. Jogs run at 1200 mm/min. Combined X jogs send a single relative `G1 X` move, and single-motor buttons wrap that move in the `M606` sequence described above. The panel is labelled "Position Commanded", meaning it shows what the firmware was told, not a measured position. "Home all" sends `G28`, so leave it alone until the X endstops are verified.

### G-code you can use

| Command | Effect |
|---------|--------|
| `G1 X.. Y.. Z.. F..` | Move. `F` is in mm/min. |
| `G90` / `G91` | Absolute / relative positioning. |
| `G92 X0` | Set the current X position; also resets the X1/X2 counters. |
| `M606 S0` | Both X motors together (default). |
| `M606 S1 X1` / `M606 S1 X2` | Move only X1 or only X2. |
| `M114` | Report positions, including `X1:` and `X2:`. |
| `M115` | Firmware info. |
| `G28` | Home. **Use with care**, see Safety. |

A single-motor jog with the GUI is the sequence `M606 S1 X1` (or `X2`), a relative `G1 X..` move, `G90`, then `M606 S0`. You can send the same lines by hand.

### Troubleshooting

- **No reply to `M115`:** check the COM port (it can change between COM3 and COM4), the baud rate (115200), and that the firmware was flashed.
- **Build fails with SCons file-write errors:** run a clean build.
- **One X motor moves the wrong way:** flip `INVERT_X2_VS_X_DIR` and rebuild.
- **Motors drift out of step after single-motor moves:** return to `M606 S0` and compare `X1` and `X2` in `M114`.

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
