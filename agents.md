# Vyper Project Notes

## Overview
- Marlin 2.x fork of the ANYCUBIC-3D/Vyper firmware (`vyper` branch, release 2.4.5 for hardware v0.0.6) for an Anycubic Vyper printer.
- Remotes: `upstream` = `https://github.com/ANYCUBIC-3D/Vyper.git` (branch `vyper`)
- The printer is repurposed as a **gantry robot (gripper)**: the extruder and hotend are removed, and the controller is driven over USB serial.
- Board target: `BOARD_AC_TRI_F1_V1` (TriGorilla STM32F103RE board, env `ac_tri_f1`).
- No extruders are present. `EXTRUDERS` is 2, but no extruder is driven: the E0 and E1 drivers carry X2 and Z2 (see Hardware Facts). The two X motors sit opposite each other, with their top portions facing each other.

## Build
- PlatformIO env: `ac_tri_f1` (`board = ac_tri_f103re`, `MCU_STM32F103RE`, `VECT_TAB_OFFSET=0x8000`).
- Build command: `pio run -e ac_tri_f1` (or the PlatformIO sidebar Build button).
- Changed in `platformio.ini` (`[env:ac_tri_f1]`): `board_build.core = stm32`, and `extra_scripts` adds `pre:buildroot/share/PlatformIO/scripts/generic_create_variant.py`. That script copies the board definition and the variant into the framework. Both are new files in this fork:
  - Board JSON: `buildroot/share/PlatformIO/boards/ac_tri_f103re.json` (STM32F103RE, 512k flash, 64k RAM).
  - Variant: `buildroot/share/PlatformIO/variants/AC_TRI_F103RE/` (`variant.cpp/.h`, `PeripheralPins.c`, `PinNamesVar.h`, `hal_conf_custom.h`, `ldscript.ld`).
- Output: `.pio\build\ac_tri_f1\debug\firmware.bin` (`board_build.firmware` says `ac_tri_f1.bin`, but the actual file is `firmware.bin`).
- Build type is `debug` (`build_type = debug`, unchanged from upstream). Use a release build for a normal flashing image.
- Clean builds fix earlier SCons file-write errors under `.pio\build`.
- Local framework copy: `framework-arduinoststm32_vyper-main/` (ignored in `.gitignore`).

## Key Files
- `Marlin/Configuration.h`: motherboard, extruder count, temp sensors, endstops, motion tuning, axis limits, home positions, leveling and safety features.
- `Marlin/Configuration_adv.h`: `X_DUAL_STEPPER_DRIVERS`, `INVERT_X2_VS_X_DIR`, gripper geometry (`X_GRIPPER_HOME_GAP`, `X_GRIPPER_MIN_GAP`), stepper pin mapping (`X2_*`, `Z2_*`), Z multi-endstops, disabled `DUAL_X_CARRIAGE` block.
- `Marlin/src/pins/stm32f1/pins_AC_TRI_F1_V1.h`: endstop pins (X max moved to PA5, Z2 min on PC6).
- `Marlin/src/gcode/control/M606.cpp` (new): M606 X motor selection, X2 homing to X max, gripper soft limits. Registered in `gcode.cpp` / `gcode.h` under `X_DUAL_STEPPER_DRIVERS`.
- `Marlin/src/gcode/calibrate/G28.cpp`: X homing split into X1 (min) and X2 (max).
- `Marlin/src/gcode/geometry/G92.cpp`: G92 sets the raw position (no workspace shift) with `X_DUAL_STEPPER_DRIVERS`.
- `Marlin/src/gcode/host/M114.cpp`: position report, extended with `X1:<mm> X2:<mm>`.
- `Marlin/src/module/stepper.cpp` / `stepper.h`: direction and step logic; per-motor X counters; X motor locks; `report_dual_x_positions()`, `dual_x_logical_steps()`.
- `Marlin/src/module/motion.cpp` / `motion.h`: X soft limits via `dual_x_motion_limits()`; declarations of the `dual_x_*` helpers.
- `Marlin/src/module/endstops.cpp`: `X_MIN_TEST` / `X_MAX_TEST` tied to the active X motor.
- `Marlin/src/module/settings.cpp` / `settings.h`: `EEPROM_VERSION` bumped; last-position save/restore code present but commented out.
- `Marlin/src/lcd/extui/lib/anycubic_dgus/dgus_tft.cpp` and `Marlin/src/lcd/extui/ui_api.cpp`: DGUS touchscreen UI, guarded for missing heater/probe/runout/mesh; flow functions stubbed.
- `Controller/printer_control_gui.py` (new): tkinter control GUI over serial.
- `RoboCup/Simulation/` (new): PyBullet simulation (`vyper.urdf`, `run_gui.py`, `plan_gripper_move.py`). `RoboCup/Planner/path_planner.py` is an empty placeholder.
- `buildroot/`, `docker/`, `docs/`, `config/`, `data/www/`: Marlin support material.

## Hardware Facts
- Two X motors on opposite sides of the X frame. Wiring:
  - X1 on the X driver (`X_STEP/DIR/ENABLE`).
  - X2 on the **E0** driver pins (extension cable). `X2_*_PIN` = `E0_*_PIN`.
  - Z1 is unchanged. ZR connector is wired to the E1 driver, so `Z2_*_PIN` = `E1_*_PIN`.
- E0 direction pin (PB3) is shared with X2 direction. `Stepper::set_directions()` therefore must not write the E0 direction in this config (it would glitch X2); X2 direction is set only through `X_APPLY_DIR`.
- E0 is also the X2 step/enable pin: **do not send E moves on T0**. The E1 driver is the Z2 driver: **do not send E moves on T1** either.
- Board is a TriGorilla V0.0.6 (stock Vyper board; pin map matches `pins_AC_TRI_F1_V1.h`).
- Endstops, by real pin in `pins_AC_TRI_F1_V1.h`:
  - X1: `X_MIN_PIN` PA7 (X connector). Homes to min: `M606 S1 X1`, `G28 X`, `M606 S0`.
  - X2: `X_MAX_PIN` PA5 ("filament" connector; upstream had X max on PC6), switch physically at X max. Homes with `M606 H2` (X1 locked, ends in `M606 S0`). The board pulls PA5 low (made for an active-high runout sensor), so the switch goes between the +5V pin and S (COM to +5V, NO to S), `X_MAX_ENDSTOP_INVERTING false`. A switch to GND on this connector never registers.
  - `X_DUAL_ENDSTOPS` is off because X1 and X2 home in opposite directions. `G28` / `G28 X` (any source: LCD, host, `M1000`) homes X1 to min, then X2 to max, each with the other motor locked (`G28.cpp`, calls `home_x2_to_max()` in `M606.cpp`). An active `M606 S1 X1` / `M606 S1 X2` limits `G28 X` to that motor.
  - X min only counts while X1 is active and X max only while X2 is active (`X_MIN_TEST` / `X_MAX_TEST` in `endstops.cpp`, `Stepper::x1_active()` / `x2_active()`).
  - Y: `Y_MIN_PIN` PC5. Z1: `Z_MIN_PIN` PB2 (ZL). Z2: `Z2_MIN_PIN` PC6 (ZR), via `Z_MULTI_ENDSTOPS`. `Z2_USE_ENDSTOP _ZMAX_` (with `USE_ZMAX_PLUG`, `Z_MAX_PIN` -1) is a placeholder Marlin 2.0's SanityCheck requires (upstream used `_XMAX_`).
  - Inverting: X min and Y min `true`, Z min, Z2 min and X max `false`. X1's switch is normally open to GND (verified with `M119`). Others not yet verified.
- X1 and X2 share one X frame: X1's switch is `X_MIN_POS` (0), X2's switch is `X_MIN_POS + X_GRIPPER_HOME_GAP` (182 mm). After `G28`: X1 = 0, X2 = 182, logical X = 91.
- Logical X (`G1 X`, M114 `X:`) is the gripper midpoint in `M606 S0`, and the active gripper's position in `M606 S1 X1` / `S1 X2`. `dual_x_select()` (`M606.cpp`) switches the mode and re-derives the logical X from the X1/X2 counters. `G28` always ends in `M606 S0`.
- Position resets (`_set_position`, `set_axis_position`) shift the active X motor counter(s) by the change of the logical X. Y/Z homing and `G92 Y/Z` leave X1/X2 alone.
- Soft limits (`dual_x_motion_limits()`, used in `apply_motion_limits`), active once both X motors are homed: X1 >= its switch, X2 <= its switch, X2 - X1 >= `X_GRIPPER_MIN_GAP` (placeholder 0, measure it). Right after `G28` combined X cannot move, since both grippers sit on their switches. Narrow the gap with single-motor moves first.
- No heater is fitted. **Do not send M104/M109/M140/M190.** Heater pins are still wired and there is no real temperature feedback.

## Firmware Changes vs. upstream `vyper`
1. **Thermal sensing and heaters disabled** (`Configuration.h`):
   - `TEMP_SENSOR_0` 5 -> 998, `TEMP_SENSOR_1` 0 -> 998 (dummy sensor, always 25 °C; `EXTRUDERS 2` needs a sensor on both slots).
   - `TEMP_SENSOR_BED` 1 -> 0.
   - Commented out: `PIDTEMP`, `PIDTEMPBED`, `PREVENT_COLD_EXTRUSION`, `EXTRUDE_MINTEMP`, `PREVENT_LENGTHY_EXTRUDE`, `EXTRUDE_MAXLENGTH`, `THERMAL_PROTECTION_HOTENDS/BED/CHAMBER`.
   - `E0_DRIVER_TYPE` commented out (E0 is the X2 driver).
2. **Extruder count**: `EXTRUDERS` 1 -> 2.
3. **Leveling, probe, runout and pause disabled** to get through SanityCheck:
   - `Configuration.h`: `AUTO_BED_LEVELING_BILINEAR`, `ENABLE_LEVELING_AFTER_G28`, `PREHEAT_BEFORE_LEVELING`, `Z_MIN_PROBE_PIN`, `NOZZLE_AS_PROBE`, `Z_MIN_PROBE_REPEATABILITY_TEST`, `FILAMENT_RUNOUT_SENSOR`.
   - `Configuration_adv.h`: `ADVANCED_PAUSE_FEATURE`, `BABYSTEP_ZPROBE_OFFSET`.
4. **Endstop plugs** (`Configuration.h`): `USE_XMAX_PLUG` kept (now X2's switch on PA5); `USE_ZMAX_PLUG` enabled as the `Z2_USE_ENDSTOP` placeholder.
5. **Motion tuning for X** (`Configuration.h`, X / Y / Z / E):
   - `DEFAULT_AXIS_STEPS_PER_UNIT` `{ 80, 100, 400, 405 }` -> `{ 400, 100, 400, 405 }`.
   - `DEFAULT_MAX_FEEDRATE` `{ 300, 300, 40, 80 }` -> `{ 40, 300, 40, 80 }`.
   - `DEFAULT_MAX_ACCELERATION` `{ 1500, 1000, 50, 3000 }` -> `{ 50, 1000, 50, 3000 }`; `MAX_ACCEL_EDIT_VALUES` X 6000 -> 200.
   - `DEFAULT_XJERK` 5 -> 0.3; `MAX_JERK_EDIT_VALUES` X 20 -> 0.6.
   - `HOMING_FEEDRATE_MM_M` `{ 50*60, 50*60, 4*60 }` -> `{ 10*60, 25*60, 4*60 }`.
6. **Axis limits and home positions** (`Configuration.h`):
   - `X_BED_SIZE` 246 -> 180 (`Y_BED_SIZE` still 255). Not yet verified against the frame.
   - `Y_MIN_POS` 0 -> -125, `Y_MAX_POS` `Y_BED_SIZE` -> `Y_BED_SIZE-MANUAL_Y_HOME_POS` (= 380, written without parentheses).
   - `Z_MIN_POS` 0 -> 30, `Z_MAX_POS` 265 -> 255.
   - `MANUAL_X_HOME_POS` +0.5 -> +0.0, `MANUAL_Y_HOME_POS` -4.2 -> -125, `MANUAL_Z_HOME_POS` -4 -> +35.
7. **Host keepalive** enabled: `HOST_KEEPALIVE_FEATURE`, `DEFAULT_KEEPALIVE_INTERVAL 3` (the GUI uses `busy:` messages to extend its ack timeout).
8. **Pins** (`pins_AC_TRI_F1_V1.h`): `X_MAX_PIN` PC6 -> PA5, new `Z2_MIN_PIN` PC6, comments for the gantry wiring.
9. **Park point** (`Configuration.h`): `NOZZLE_PARK_POINT` Z 20 -> `(Z_MIN_POS + 20)`.
10. **Independent X motors via `X_DUAL_STEPPER_DRIVERS` + M606** (not `DUAL_X_CARRIAGE`):
    - `Configuration_adv.h`: `X_DUAL_STEPPER_DRIVERS` and `INVERT_X2_VS_X_DIR` enabled, `X_DUAL_ENDSTOPS` off, `X_GRIPPER_HOME_GAP 182`, `X_GRIPPER_MIN_GAP 0`, `X2_*` = `E0_*` and `Z2_*` = `E1_*` pins (Z2 pins are defined in both the X-dual and the Z block, same values).
    - `DUAL_X_CARRIAGE` stays disabled (mutually exclusive in `SanityCheck.h`). Its block was edited anyway (X2 pins, `INVERT_X2_VS_X_DIR`, default mode `DXC_DUPLICATION_MODE`), and the `DUAL_X_CARRIAGE` branch of `X_APPLY_DIR` in `stepper.cpp` honours `INVERT_X2_VS_X_DIR`.
    - `M606 S0`: both X motors (default). `M606 S1 X1` / `M606 S1 X2`: only that motor. `M606 H2`: home X2 to X max. M606 calls `planner.synchronize()` first.
    - M605 needs `DUAL_X_CARRIAGE`, so it is not compiled.
11. **Stepper changes** (`stepper.cpp` / `stepper.h`):
    - Lock plumbing: `locked_X_motor` / `locked_X2_motor` and `separate_multi_axis` compiled with `X_DUAL_STEPPER_DRIVERS`; `X_APPLY_STEP` uses `DUAL_SEPARATE_APPLY_STEP`; new `x1_active()` / `x2_active()`.
    - X2 direction fix: `set_directions()` does not write the E0 direction (shared with X2 DIR); it only sets `count_direction.e`. X2 direction comes only from `X_APPLY_DIR` (`X2_DIR_WRITE(v ^ INVERT_X2_VS_X_DIR)`).
    - Per-motor counters `count_position_x1` / `count_position_x2`, advanced in `PULSE_PREP` (`COUNT_DUAL_X_STEP`) unless that motor is locked. `_set_position()` / `set_axis_position()` shift the active counters by the change in logical X, unless `dual_x_hold_counts` is set (used by `dual_x_select()`).
    - `report_dual_x_positions()` (M114 `X1:` / `X2:` line), `dual_x_logical_steps()`, `x1_steps()` / `x2_steps()`.
12. **Homing and motion** (`G28.cpp`, `M606.cpp`, `motion.cpp`, `endstops.cpp`): G28 X homes X1 to min, then X2 to max, and G28 ends in `dual_x_select(0)`; `apply_motion_limits` uses `dual_x_motion_limits()` instead of the X soft endstops; endstop tests follow the active X motor.
13. **G92** (`G92.cpp`): with `X_DUAL_STEPPER_DRIVERS`, G92 sets the raw position (Marlin 1.0 behaviour) instead of a workspace offset, so the X1/X2 counters and soft limits are re-referenced.
14. **EEPROM** (`settings.cpp`): `EEPROM_VERSION` "V82" -> "V83". This is active and makes the first boot after flashing discard the stored settings and load the defaults. Code that saved the last position (X1, Y, Z, X2) to EEPROM and restored it at boot is still in `settings.cpp/.h`, `stepper.cpp/.h` (`get/set_x2_position_mm`) and `MarlinCore.cpp` (`restore_last_position()` call), but commented out (commit "removed position storing inside the board").
15. **Anycubic DGUS UI kept** (`ANYCUBIC_LCD_DGUS`): heater max limits defined as 0 when the heater is disabled, `page201_handle` behind `HAS_BED_PROBE` (empty stub otherwise), runout/mesh/temperature-fault code behind `HAS_FILAMENT_SENSOR`, `HAS_MESH`, `HAS_TEMP_HOTEND`, `HAS_TEMP_BED`. `ui_api.cpp`: `getFlowPercentage` returns 100, `setFlow_percent` is a no-op. Probe/leveling pages and the screen flow controls do not work.
16. **Cosmetic only**: comment-encoding changes in `MarlinCore.cpp` (setup() bullet list) and `Configuration_adv.h` (`BOOT_MARLIN_LOGO_ANIMATED` comment), one blank line in `loop()`.
17. **Repo housekeeping**: `.gitignore` adds `/framework-arduinoststm32_vyper-main`, `todo.md`, `/assembly_station` (PyBullet venv).

## GUI (`Controller/printer_control_gui.py`)
- Tkinter GUI over serial at 115200 baud (`FEEDRATE` 1200 mm/min, `POSITION_INTERVAL_MS` 500 for M114 polling).
- Sends numbered lines with checksums; waits for `ok` (a `busy:` keepalive extends the 10 s `ACK_TIMEOUT_S`), retransmits on timeout or `Resend:`, and resyncs the line number with `M110`.
- Sends `M120` on connect, so endstops are checked on every move.
- Axis jog: Y+/Y-, Z+/Z-, X-/X+ (both X motors, relative `G1 X` in `G91`). Step size 0.1, 1, 10 or 50 mm. The "Home all" (`G28`) button is commented out.
- Single X motor: X1-/X1+/X2-/X2+ send `M606 S1 X1|X2`, `G91`, `G1 X…`, `G90`, `M606 S0`. The label "X1 = T0, X2 = T1" is only a naming hint; no `T` command is sent.
- Home axis: Home X1 (`M606 S1 X1`, `G28 X`, `M606 S0`), Home X2 (`M606 H2`), Home Y / Home Z (`G28 Y` / `G28 Z`).
- Reference: "Set zero here" sends `G92 X0 Y0 Z0` after a confirmation (resets the X1/X2 counters too). The frame is still labelled "no endstops", which predates the endstop wiring.
- Go to position: X (midpoint), Y, Z. X is sent as a relative move from the current midpoint (keeps the X1/X2 gap), Y/Z absolute.
- Position panel: `X mid` = `(X1 + X2) / 2`, `X1`, `X2`, `Y`, `Z` ("Position Commanded (mm)"). The reader thread drops the M114 `Count …` line and hides the `ok` replies of the silent polls.

## Simulation (`RoboCup/Simulation/`)
- `vyper.urdf`: simplified Vyper model (X1/X2 jaws on the X beam, moving bed for Y, gantry Z).
- `run_gui.py`: PyBullet GUI with joint sliders, a Lego stack on the bed, and a "Plan & move to goal" action.
- `plan_gripper_move.py`: collision-free moves with `pybullet_planning` (`birrt` + `smooth_path`) in (x, y, z) with a fixed gripper gap (M606 S0 style). Run with the `assembly_station` venv, e.g. `../../assembly_station/bin/python plan_gripper_move.py`.

## Testing Notes
- GUI setup: `pip install pyserial`, then `python Controller/printer_control_gui.py`. The GUI runs on the host only.
- Verify with `G1 Z5 F300` (Z motors only, including ZR) and `G1 X5 F1200` (both X motors same direction).
- If one X motor runs backwards, flip `INVERT_X2_VS_X_DIR`.
- Serial: Vyper on COM3/COM4 (CH340 adapter moved between ports). Try `M115` / `M114` at 115200 baud. Expected M114 output includes `X1:<mm> X2:<mm>`.
- After flashing a build with a new `EEPROM_VERSION` (or using an `*_erase_eeprom.bin` image), settings come from the defaults in `Configuration*.h`; re-check with `M503`.

## Open Items
- Confirm X direction and X2 homing behaviour on hardware after flashing.
- Measure and set `X_GRIPPER_MIN_GAP` and confirm `X_GRIPPER_HOME_GAP` (182 mm).
- Verify the Y (-125..380) and Z (30..255) ranges, `X_BED_SIZE` 180, and X steps/mm 400 against the frame.
- Verify the Y, Z1 and Z2 endstop logic with `M119`.
- Update the stale `EXTRUDERS` comment and the GUI "no endstops" label.
- Check whether the page-stepping path needs the X1/X2 counter updates.
- Decide on a release build for flashing.
