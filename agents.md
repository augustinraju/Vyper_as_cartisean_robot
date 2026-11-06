# Vyper Project Notes

## Overview
- Marlin 2.x fork (`vyper` branch of ANYCUBIC-3D/Vyper firmware, base release 2.4.5) for an Anycubic Vyper printer.
- The printer is repurposed as a **gantry robot**: the extruder and hotend are removed, and the controller is driven over USB serial.
- Board target: `BOARD_AC_TRI_F1_V1` (Kobra Go / TriGorilla-style STM32F103RE board, env `ac_tri_f1`).
- No extruders are present. `EXTRUDERS` is still set to 2 in the configuration, but no extruder is driven by it: the E0 and E1 drivers carry X2 and Z2 (see Hardware Facts). The two X motors sit opposite each other, with their top portions facing each other.

## Build
- PlatformIO env: `ac_tri_f1` (`board = ac_tri_f103re`, `MCU_STM32F103RE`, `VECT_TAB_OFFSET=0x8000`).
- Build command: `pio run -e ac_tri_f1` (or the PlatformIO sidebar Build button).
- `platformio.ini` (`[env:ac_tri_f1]`) sets `board_build.core = stm32` and runs `buildroot/share/PlatformIO/scripts/generic_create_variant.py` as an `extra_scripts` pre-build step. That script copies the board definition and the `AC_TRI_F103RE` variant into the framework:
  - Board JSON: `buildroot/share/PlatformIO/boards/ac_tri_f103re.json` (STM32F103RE, 512k flash, 64k RAM).
  - Variant: `buildroot/share/PlatformIO/variants/AC_TRI_F103RE/` (`variant.cpp/.h`, `PeripheralPins.c`, `PinNamesVar.h`, `hal_conf_custom.h`, `ldscript.ld`).
- Output: `.pio\build\ac_tri_f1\debug\firmware.bin` (`board_build.firmware` says `ac_tri_f1.bin`, but the actual file is `firmware.bin`).
- Build type is `debug`, set explicitly in the `[env:ac_tri_f1]` block. Use a release build for a normal flashing image.
- Clean builds fix earlier SCons file-write errors under `.pio\build`.
- Local framework copy: `framework-arduinoststm32_vyper-main/` (contains `variants/`, `boards.txt`, `cores/`, `system/`). Untracked in git.

## Key Files
- `Marlin/Configuration.h`: motherboard, extruder count, temp sensors, endstops, axis limits, leveling and safety features.
- `Marlin/Configuration_adv.h`: stepper pin mapping (`X2_*`, `Z2_*`), `X_DUAL_STEPPER_DRIVERS`, `INVERT_X2_VS_X_DIR`, disabled `DUAL_X_CARRIAGE` block, Z multi-endstops.
- `Marlin/src/module/stepper.cpp` / `stepper.h`: direction and step logic; per-motor position counters; X motor locks; `report_dual_x_positions()`.
- `Marlin/src/gcode/control/M606.cpp` (new, untracked): M606 single-motor selection. Registered in `gcode.cpp` / `gcode.h` under `X_DUAL_STEPPER_DRIVERS`.
- `Marlin/src/gcode/host/M114.cpp`: position report, extended with `X1:<mm> X2:<mm>` when `X_DUAL_STEPPER_DRIVERS` is enabled.
- `Marlin/src/lcd/extui/lib/anycubic_dgus/dgus_tft.cpp` and `Marlin/src/lcd/extui/ui_api.cpp`: DGUS touchscreen UI, guarded for missing heater/probe/runout/mesh; flow functions stubbed.
- `Controller/printer_control_gui.py` (new, untracked): tkinter jogging GUI over serial (M114 polling).
- `buildroot/`, `docker/`, `docs/`, `config/`, `data/www/`: Marlin support material.
- `framework-arduinoststm32_vyper-main/`: local STM32 Arduino framework and variants.
- Binaries and archives in the repo root (untracked, not source): `firmware.bin` (152 KB, Oct 28), `Vyper firmware 2.4.5.zip` (~22 MB, the stock release), `go-firmware-release_0.2.0_Windows_x86_64/` (vendor flashing tool, with `main_board_*_erase_eeprom.bin` images from 2026-11-02 and 2026-11-03).

## Hardware Facts
- Two X motors on opposite sides of the X frame. Wiring:
  - X1 on the X driver (`X_STEP/DIR/ENABLE`).
  - X2 on the **E0** driver pins (extension cable). `X2_*_PIN` = `E0_*_PIN`.
  - Z1 is unchanged. ZR connector is wired to the E1 driver, so `Z2_*_PIN` = `E1_*_PIN`.
- E0 direction pin (PB3) is shared with X2 direction. `Stepper::set_directions()` therefore must not write the E0 direction in this config (it would overwrite or glitch X2); X2 direction is set only through `X_APPLY_DIR`.
- E0 is also the X2 step/enable pin: **do not send E moves on T0**. The E1 driver is the Z2 driver: **do not send E moves on T1** either.
- Config enables the X min/max endstop plugs (`X_STOP_PIN` PA7, `X_MAX_PIN` PC6), but physical switches at the X boundaries are not confirmed. Do not home before X moves until this is verified.
- Z homes on Z1 only (`Z_MULTI_ENDSTOPS` is off). The Z2 endstop (ZR_STOP) would share the X-MAX pin (PC6), per the comment in `Configuration_adv.h`.
- No heater is fitted. **Do not send M104/M109/M140/M190.** Heater pins are still wired and there is no real temperature feedback.

## Firmware Changes Made
1. **Hotend and bed thermal sensing disabled** for a no-hotend setup (`Configuration.h`):
   - `TEMP_SENSOR_0` and `TEMP_SENSOR_1` are 998 (dummy sensor, always reads 25 °C).
   - `TEMP_SENSOR_BED` is 0 (was 1).
   - `PIDTEMP`, `PIDTEMPBED`, `THERMAL_PROTECTION_*`, `PREVENT_COLD_EXTRUSION`, `EXTRUDE_MINTEMP`, `PREVENT_LENGTHY_EXTRUDE`, `EXTRUDE_MAXLENGTH` are commented out.
   - `E0_DRIVER_TYPE` is commented out; E0 is now the X2 driver.
2. **Bed leveling, probe and related features disabled** to get the config through SanityCheck (`Configuration.h`):
   - `AUTO_BED_LEVELING_BILINEAR`, `ENABLE_LEVELING_AFTER_G28`, `PREHEAT_BEFORE_LEVELING`.
   - `Z_MIN_PROBE_PIN`, `NOZZLE_AS_PROBE`, `Z_MIN_PROBE_REPEATABILITY_TEST`.
   - `FILAMENT_RUNOUT_SENSOR` (in `Configuration.h`), `ADVANCED_PAUSE_FEATURE` and `BABYSTEP_ZPROBE_OFFSET` (in `Configuration_adv.h`).
3. **Anycubic DGUS UI kept** (`ANYCUBIC_LCD_DGUS`): guarded heater max limits (0 when disabled), `page201_handle` behind `HAS_BED_PROBE` (empty stub otherwise), and filament-runout, mesh and temperature-fault code behind `HAS_FILAMENT_SENSOR`, `HAS_MESH`, `HAS_TEMP_HOTEND`, `HAS_TEMP_BED`. Flow functions stubbed in `ui_api.cpp` (`getFlowPercentage` returns 100, `setFlow_percent` no-op). Compiled only, not flashed. Probe/leveling pages and the screen flow controls do not work.
4. **Independent X motor control via `X_DUAL_STEPPER_DRIVERS` + M606** (not `DUAL_X_CARRIAGE`):
   - `X_DUAL_STEPPER_DRIVERS` is enabled in `Configuration_adv.h`; `DUAL_X_CARRIAGE` is disabled. The two are mutually exclusive (`SanityCheck.h`).
   - `M606 S0` moves both X motors together (default). `M606 S1 X1` or `M606 S1 X2` locks the other motor for single-motor moves. `M606` calls `planner.synchronize()` first, so it waits for queued motion.
   - M605 (duplication mode) requires `DUAL_X_CARRIAGE`, so it is not compiled in this config.
   - Caveat: while a motor is locked, its step count is skipped, so after a single-motor move the two motors are physically apart by that distance. Return to `M606 S0` for combined motion.
5. **X2 direction fix** (`stepper.cpp`): `E0_DIR_PIN` is the same pin as `X2_DIR_PIN`, so `set_directions()` no longer writes the E0 direction when `X_DUAL_STEPPER_DRIVERS` is on. It only sets `count_direction.e`. The earlier fix re-wrote X2 after the E0 write at the end of `set_directions()`, which glitched the X2 DIR line right before stepping, and it was removed. X2 direction now comes only from `X_APPLY_DIR`, which writes `X2_DIR_WRITE(v ^ INVERT_X2_VS_X_DIR)` (`INVERT_X2_VS_X_DIR` is enabled).
6. **Per-motor position counters**: `count_position_x1` and `count_position_x2` in `stepper.cpp`/`stepper.h`. They advance in the step-pulse path (`COUNT_DUAL_X_STEP` inside `PULSE_PREP`, around line 1564), skipped when that motor is locked by M606. `_set_position()` and `set_axis_position()` reset them, so homing and `G92 X` reset them. M114 prints `X1:<mm> X2:<mm>` through `Stepper::report_dual_x_positions()`.
7. **Fixed-size bed**: `X_BED_SIZE` is set to 130 (was 246). `Y_BED_SIZE` is still 255. Not yet verified against the physical frame.
8. **Stepper X-motor lock plumbing**: `locked_X_motor` / `locked_X2_motor` and `separate_multi_axis` are now compiled when `X_DUAL_STEPPER_DRIVERS` is on (previously only with `X_DUAL_ENDSTOPS`). `X_APPLY_STEP` uses `DUAL_SEPARATE_APPLY_STEP` for X in this mode.

## GUI (`Controller/printer_control_gui.py`)
- Tkinter GUI for jogging X, Y, Z over serial at 115200 baud (`FEEDRATE` 1200 mm/min, `POSITION_INTERVAL_MS` 500).
- Jog buttons: Y+/Y-, Z+/Z-, X-/X+ (both X motors together, one `G1 X` move in `G91` mode), and "Home all" (sends `G28`).
- Single-motor buttons X1-/X1+/X2-/X2+ send `M606 S1 X1` or `M606 S1 X2`, the relative `G1 X` move, `G90`, then `M606 S0`. The group label says "X1 = T0, X2 = T1", but no `T` command is sent; the label is only a naming hint.
- Step size is selectable: 0.1, 1, 10, 50 mm.
- Position panel shows `X mid` = `(X1 + X2) / 2`, `X1`, `X2`, `Y`, `Z` (labelled "Position Commanded (mm)"). Y and Z come from the M114 `X:… Y:… Z:… E:…` line; X1/X2 come from the `X1:… X2:…` line.
- Reader thread drops the M114 `Count …` line so it does not flood the log, and hides the `ok` replies of the silent M114 polls.
- Homing is not tied to the X buttons (no switches). "Home all" still runs `G28` and should be used with care until the X endstops are verified.

## Testing Notes
- GUI setup: `pip install pyserial`, then `python Controller/printer_control_gui.py`. The GUI runs on the host only; it does not affect firmware builds.
- Verify with `G1 Z5 F300` (Z motors only, including ZR) and `G1 X5 F1200` (both X motors same direction).
- If one X motor runs backwards, flip `INVERT_X2_VS_X_DIR`.
- Serial: Vyper on COM3/COM4 (CH340 adapter moved between ports). Try `M115` / `M114` at 115200 baud. Expected M114 output includes a line `X1:<mm> X2:<mm>`.
- Everything compiles (`ac_tri_f1`), but the latest direction and counter changes are **not yet flashed or verified on the printer**.

## Open Items
- Confirm X direction behaviour on hardware after flashing.
- Confirm the Vyper mainboard matches `AC_TRI_F1_V1` (check MCU marking).
- Verify bed size, steps/mm, and LCD settings for the gantry use case.
- Check whether the page-stepping path needs the X1/X2 counter updates.
- Decide on the release build for flashing.
