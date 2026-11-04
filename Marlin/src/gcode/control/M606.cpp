/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "../../inc/MarlinConfig.h"

#if ENABLED(X_DUAL_STEPPER_DRIVERS)

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/stepper.h"

/**
 * M606: Select which X motor(s) move (X_DUAL_STEPPER_DRIVERS)
 *
 *   M606 S0       : Both X motors move together (default). Clears any single-motor lock.
 *   M606 S1 X1    : Only X1 moves. X2 is locked (its steps are skipped).
 *   M606 S1 X2    : Only X2 moves. X1 is locked.
 *
 * Caveat: while a motor is locked, Marlin still counts its steps in the position model,
 * so after a single-motor move the two X motors are physically apart by that distance.
 * Return to M606 S0 for normal combined motion.
 */
void GcodeSuite::M606() {
  planner.synchronize();  // Finish queued motion before changing the locks

  if (!parser.intval('S', 0)) {
    stepper.set_separate_multi_axis(false);
    stepper.set_x_lock(false);
    stepper.set_x2_lock(false);
    return;
  }

  if (!parser.seen('X')) {
    SERIAL_ECHO_MSG("M606 S1 needs X1 or X2");
    return;
  }

  const uint8_t motor = parser.value_byte();
  if (motor != 1 && motor != 2) {
    SERIAL_ECHO_MSG("M606 X must be 1 or 2");
    return;
  }

  stepper.set_separate_multi_axis(true);
  stepper.set_x_lock(motor == 2);   // Lock X1 when only X2 should move
  stepper.set_x2_lock(motor == 1);  // Lock X2 when only X1 should move
}

#endif // X_DUAL_STEPPER_DRIVERS
