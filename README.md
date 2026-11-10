# Vyper Gantry Robot

A 3D printer turned into a small **XYZ gantry robot with a two-jaw gripper**. This repository holds:

- a modified Marlin firmware for the Anycubic Vyper,
- a desktop GUI for moving it from a PC,
- a PyBullet simulation with a collision-free motion planner,
- a translation layer that drives the real printer from the simulation.

The extruder and hotend are removed. The printer's motion system (Y, Z) is kept, two X motors replace the extruder motor to control the gripper and driven over USB serial with ordinary G-code. Each X motor carries one gripper jaw, so the two motors can move the gripper sideways together or open and close it.

> **Status: work in progress.** Several dimensions and the simulation calibration are not yet verified on the machine. See [Project status](#project-status).

## How it works

```
 Simulation GUI (PyBullet)  --UDP-->  Bridge  --.
                                                |
 Control GUI / your script  --------------------+--USB serial, G-code-->  Vyper mainboard (modified Marlin)
                                                                            |-- X1 motor (X driver)     -> right jaw
                                                                            |-- X2 motor (E0 driver)    -> left jaw
                                                                            |-- Y motor                 -> bed
                                                                            '-- Z motors (Z1, Z2 on E1) -> gantry
```

- **Base:** Marlin 2.x, from the `vyper` branch of the ANYCUBIC-3D/Vyper firmware (release 2.4.5).
- **Board:** `BOARD_AC_TRI_F1_V1`, a TriGorilla V0.0.6 board with a GD32F103 MCU. The firmware is built for STM32F103RE (PlatformIO env `ac_tri_f1`).
- **No heaters, no extruder.** Temperature sensing is replaced by a dummy sensor.
- **Two X motors.** X1 uses the X driver. X2 uses the E0 driver pins through an extension cable. The Z2 motor uses the E1 driver (the ZR connector).

### Wiring

| Motor | Driver on the board | Firmware pins | Endstop |
|-------|--------------------|---------------|---------|
| X1 | X driver | `X_STEP/DIR/ENABLE` | X min, PA7 (X connector) |
| X2 | E0 driver (extension cable) | `X2_*_PIN` = `E0_*_PIN` | X max, PA5 ("filament" connector) |
| Y | Y driver | unchanged | Y min, PC5 |
| Z1 | Z driver | unchanged | Z min, PB2 (ZL) |
| Z2 | E1 driver (ZR connector) | `Z2_*_PIN` = `E1_*_PIN` | Z2 min, PC6 (ZR) |

The two X motors sit opposite each other, so one must turn the opposite way to the other for both jaws to move in the same direction. `INVERT_X2_VS_X_DIR` handles this.

The X2 switch is on the "filament" connector, which the board pulls low. Wire the switch between the +5V pin and S (COM to +5V, NO to S). A switch to GND on this connector never registers.

Since the E0 and E1 drivers now belong to X2 and Z2, the firmware has no working extruder. This is why `E` moves must never be sent (see Safety).

### Firmware features added

- **Independent X motors** (`X_DUAL_STEPPER_DRIVERS`) with a new G-code, `M606`:

  | Command | Mode | What `G1 X` means |
  |---------|------|-------------------|
  | `M606 S0` | Both X motors together (default) | Gripper midpoint; the gap is kept |
  | `M606 S1 X1` / `M606 S1 X2` | Only that motor moves | That jaw's position |
  | `M606 S2` | Gripper mode: both motors in opposite directions | Half the gap; the midpoint is kept (`G1 X50` = 100 mm gap) |
  | `M606 H2` | Home X2 to its switch at X max | |

  `M606` waits for all queued moves to finish before it changes the mode. It uses `X_DUAL_STEPPER_DRIVERS`, not `DUAL_X_CARRIAGE`, so `M605` is not available.
- **Per-motor position tracking.** `M114` prints an extra line `X1:<mm> X2:<mm>`. Each motor has its own step counter.
- **Homing.** `G28` (or `G28 X`) homes X1 to X min, then X2 to X max, each with the other motor locked, and ends in `M606 S0`. After homing, X1 = 0, X2 = 181 and the midpoint is 90.5 mm. Z1 and Z2 each stop on their own switch (`Z_MULTI_ENDSTOPS`).
- **Soft limits for the gripper**, active once both X motors are homed: neither jaw passes its switch, and the jaws stay at least `X_GRIPPER_MIN_GAP` (2 mm) apart. Disabling the X motors (`M18`/`M84`) clears the homed state, and the limits are off until X is homed again.
- **Endstops outside homing.** With `M120`, an endstop hit during a normal move stops the move, drops the queued moves and re-reads the position from the steppers.
- **`G92` sets the raw position** instead of a workspace offset. An axis changed with `G92` counts as unhomed, so its soft limits are off until it is homed again.
- **Work area:** each jaw 0..181 mm on X, Y -125..130 mm, Z 30..255 mm. These numbers are not yet verified against the frame.
- The Anycubic touchscreen UI still compiles, but its probe, leveling and flow pages do not work.

[agents.md](agents.md) lists every firmware change against upstream in detail.

## Safety: read before powering the printer

- **Never send `M104`, `M109`, `M140` or `M190`.** The heater pins are still wired and there is no real temperature feedback.
- **Never send extruder moves** (`E` moves). The E0 and E1 drivers move X2 and Z2.
- **Check every endstop with `M119` before the first `G28`.** Homing drives each axis into its switch. A switch that is missing or wired wrongly lets the axis run into the frame.
- Right after `G28` both jaws sit on their switches, so a combined X move cannot go anywhere. Close the gap first (`M606 S2` or a single-motor move).
- The soft limits only apply after homing. Before that, nothing stops a jaw from hitting the frame or the other jaw.
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

Flash the image with the vendor tool (`go-firmware-release_0.2.0_Windows_x86_64/`) or your usual STM32 flashing method. This firmware has a new EEPROM version, so the first boot after flashing discards the stored settings and loads the defaults. Check them with `M503`.

### 2. Talk to the printer

Connect over USB at **115200 baud**. The port is usually COM3 or COM4 on Windows and `/dev/ttyUSB0` on Linux. Then try:

```
M115                 ; firmware info
M119                 ; endstop states - press each switch by hand and check it
M114                 ; position, including the X1:<mm> X2:<mm> line
G1 Z5 F300           ; move Z only
G1 X5 F1200          ; both X motors should move in the same direction
```

If one X motor runs the wrong way, flip `INVERT_X2_VS_X_DIR` in `Marlin/Configuration_adv.h` and rebuild.

### 3. Use the control GUI

Run it from the repository root, as a module:

```
pip install pyserial
python -m RoboCup.Controller.printer_control_gui
```

Running the file directly (`python RoboCup/Controller/printer_control_gui.py`) fails with `No module named 'RoboCup'`.

The GUI offers:

- **Axis jog** for X (both jaws), Y and Z, with step sizes 0.1, 1, 10 and 50 mm.
- **Single X motor** buttons (X1 and X2 on their own).
- **Gripper gap:** Open and Close by the jog step, or "Set gap" to a typed value. Both jaws move at once and the midpoint stays put.
- **Home** buttons for X1, X2, Y and Z.
- **Go to position** for the X midpoint, Y and Z.
- **Set zero here** (`G92 X0 Y0 Z0`).
- A free G-code box with history, and a live position panel (X mid, X1, X2, Gap, Y, Z).

Commands are not queued. A button or typed command is only accepted while the printer is free; otherwise it is dropped and the log shows "Printer busy, command ignored". The GUI sends `M120` on connect, so endstops are checked on every move. If the printer reports a fault (an endstop hit, an error, an unknown command), the rest of that command batch is dropped.

The G-code sequences live in `RoboCup/Controller/vyper_host/commands.py` and work without the GUI:

```python
import queue
from RoboCup.Controller.vyper_host import PrinterConnection, commands

link = PrinterConnection(queue.Queue())
link.connect("/dev/ttyUSB0")
link.send_lines(commands.home("Y"))
```

### 4. Run the simulation

The simulation needs `pybullet`, `pybullet_planning` and `xacro`. Install them in a virtual environment, then:

```
cd RoboCup/Simulation
python simulation_gui.py             # add --opengl2 if the GUI text is clipped
```

The window shows a simplified Vyper with a two-block stack on the bed. Sliders move Z, the gripper centre (`x_mid`), the gap and Y. Set `goal_x`, `goal_y`, `goal_z` and `goal_gap`, then press **Plan & move to goal**: the gripper follows a collision-free path with its current gap, then opens or closes to the goal gap. A command is refused if opening the jaws would hit something.

`plan_gripper_move.py` runs the planner on its own, from point A to point B:

```
python plan_gripper_move.py --start -0.07 0.06 0.005 --goal 0.07 0.12 0.005 --gap 0.03
```

The robot model is `vyper.urdf.xacro`. **Do not edit `vyper.urdf`**: it is generated at startup by `build_urdf.py`, which takes the joint limits from the firmware's `Configuration.h` and `Configuration_adv.h`. The simulation can therefore only reach what the printer accepts.

### 5. Drive the printer from the simulation

Start the simulation, then start the bridge in a second terminal (it needs `pyserial`):

```
python RoboCup/Translation/sim_printer_bridge.py /dev/ttyUSB0 --home
```

- At startup the bridge reads the printer's position and moves the simulation there. `--home` runs `G28` first.
- A slider target is sent once the slider has been still for 0.4 s. A planned move is sent as its waypoints.
- Each waypoint is range-checked before anything moves, sent on its own, and compared with the printer's `M114` report.
- On an endstop hit, a firmware error, a position mismatch or an out-of-range target, the run stops and the simulation jumps back to the printer's real position.

Do not connect the control GUI to the same serial port at the same time.

> **The mapping between simulation and printer coordinates is not calibrated.** The offsets and signs in `RoboCup/Translation/sim_to_printer.py` are first guesses. Check them on the machine (see the `CALIBRATE` comments there) before trusting the simulation for real moves.

### G-code you can use

| Command | Effect |
|---------|--------|
| `G1 X.. Y.. Z.. F..` | Move. `F` is in mm/min. |
| `G90` / `G91` | Absolute / relative positioning. |
| `G28` | Home all axes (X1 to min, then X2 to max, then Y and Z). |
| `G28 Y` / `G28 Z` | Home one axis. |
| `M606 S1 X1`, `G28 X`, `M606 S0` | Home X1 only. |
| `M606 H2` | Home X2 only. |
| `M606 S0` | Both X motors together (default). |
| `M606 S1 X1` / `M606 S1 X2` | Move only X1 or only X2. |
| `M606 S2` | Gripper mode: `G1 X` sets half the gap. |
| `G92 X0 Y0 Z0` | Set the current position as zero. Marks the axes unhomed. |
| `M114` | Report positions, including `X1:` and `X2:`. |
| `M119` | Report endstop states. |
| `M120` / `M121` | Check endstops on every move / only while homing. |
| `M400` | Wait until all moves have finished. |

To set the gripper gap to 40 mm by hand: `M606 S2`, `G90`, `G1 X20 F1200`, `M606 S0`.

### Troubleshooting

- **No reply to `M115`:** check the port, the baud rate (115200), and that the firmware was flashed.
- **Build fails with SCons file-write errors:** run a clean build.
- **One X motor moves the wrong way:** flip `INVERT_X2_VS_X_DIR` and rebuild.
- **The X2 switch never triggers:** it must switch +5V to S on the "filament" connector, not GND.
- **A combined X move does nothing after `G28`:** both jaws are on their switches. Close the gap first.
- **"Set gap" shifts the gripper instead of closing it:** the board runs an older firmware without `M606 S2`. Flash the current build.
- **`No module named 'RoboCup'`:** start the GUI with `python -m RoboCup.Controller.printer_control_gui` from the repository root.
- **`xacro is not installed`:** `pip install xacro` in the simulation environment.

## Project status

| Item | State |
|------|-------|
| Firmware builds (`ac_tri_f1`) | Done |
| Hotend, bed and leveling features disabled | Done |
| `M606` single-motor and gripper modes | Done |
| Endstops, homing of X1 / X2 / Y / Z, gripper soft limits | Done in firmware; Y, Z1 and Z2 switch logic not yet verified with `M119` |
| Control GUI | Done |
| Simulation and motion planner | Done |
| Simulation-to-printer bridge | Done; coordinate calibration open |
| Axis ranges, X steps/mm, `X_GRIPPER_MIN_GAP`, `X_GRIPPER_HOME_GAP` | Unverified |
| `RoboCup/Planner/path_planner.py` | Empty placeholder |
| Automated tests for the Python code | None |

## Where things are

| Path | What it is |
|------|------------|
| `Marlin/Configuration.h` | Board, axis limits, endstops, disabled heater and leveling features |
| `Marlin/Configuration_adv.h` | X2 and Z2 pin mapping, dual X driver setup, gripper geometry |
| `Marlin/src/gcode/control/M606.cpp` | The `M606` command, X2 homing, gripper soft limits |
| `Marlin/src/module/stepper.cpp` | Step and direction logic, per-motor counters, motor locks |
| `Marlin/src/gcode/calibrate/G28.cpp` | X homing split into X1 and X2 |
| `Marlin/src/pins/stm32f1/pins_AC_TRI_F1_V1.h` | Endstop pins |
| `RoboCup/Controller/` | Control GUI (`printer_control_gui.py`) and the `vyper_host` package (serial link, G-code sequences) |
| `RoboCup/Simulation/` | PyBullet simulation, planner, robot model (`vyper.urdf.xacro`) |
| `RoboCup/Translation/` | Simulation-to-printer coordinates, G-code generation, waypoint sender, bridge |
| `RoboCup/Planner/` | Placeholder |
| `platformio.ini` | Build environment `ac_tri_f1` |
| `agents.md` | Detailed engineering notes: every firmware change, hardware facts, open items |
| `buildroot/`, `docker/`, `docs/`, `config/`, `data/www/` | Marlin support material |

## Contributing

Open an issue or pull request. Please test changes on the hardware where you can, and state in the description whether they were flashed or only compiled. The project does not yet have a hardware test routine beyond the checks in [Quick start](#2-talk-to-the-printer).

## Credits and license

Built on [Marlin](https://marlinfw.org/) by the Marlin team and the [Anycubic Vyper firmware](https://github.com/ANYCUBIC-3D/Vyper). Marlin is published under the [GPL](LICENSE).
