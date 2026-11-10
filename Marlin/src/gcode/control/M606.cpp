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
#include "../../module/motion.h"
#include "../../module/endstops.h"

// Defined in motion.cpp (not declared in motion.h)
uint8_t do_homing_move(const AxisEnum axis, const float distance, const feedRate_t fr_mm_s, const bool final_approach);

/**
 * Shared X frame of the two grippers:
 *   X1's switch (X min) is X_MIN_POS, X2's switch (X max) is X2_SWITCH_POS,
 *   X_GRIPPER_HOME_GAP further on. After G28: X1 = 0, X2 = 182, logical X = 91.
 *
 * Logical X (G1 X, M114 X:) depends on the M606 mode:
 *   M606 S0    : midpoint of X1 and X2. Both move, the gap stays the same.
 *   M606 S1 X1 : X1's position. Only X1 moves.
 *   M606 S1 X2 : X2's position. Only X2 moves.
 * Stepper::_set_position shifts the active motor counters by the change of the
 * logical X, so the mapping holds until the mode changes. dual_x_select() changes
 * the mode and re-derives the logical X from the X1/X2 counters.
 */
#define X2_SWITCH_POS (X_MIN_POS + (X_GRIPPER_HOME_GAP))

static uint8_t dual_x_homed_bits = 0;   // bit 0: X1 homed, bit 1: X2 homed

void dual_x_set_homed(const uint8_t motor, const bool homed) {
  const uint8_t bit = _BV(motor - 1);
  if (homed) dual_x_homed_bits |= bit; else dual_x_homed_bits &= ~bit;
}

// Set the logical X from the X1/X2 counters for the current mode. The counters stay.
static void dual_x_sync_logical() {
  current_position.x = stepper.dual_x_logical_steps() / planner.settings.axis_steps_per_mm[X_AXIS];
  stepper.dual_x_hold_counts = true;
  sync_plan_position();
  stepper.dual_x_hold_counts = false;
}

void dual_x_select(const uint8_t motor) {
  planner.synchronize();  // Finish queued motion before changing the locks
  stepper.set_separate_multi_axis(motor != 0);
  stepper.set_x_lock(motor == 2);   // Lock X1 when only X2 should move
  stepper.set_x2_lock(motor == 1);  // Lock X2 when only X1 should move
  dual_x_sync_logical();
}

/**
 * Allowed range of the logical X for the current mode, so that
 *   X1 >= X_MIN_POS (its switch), X2 <= X2_SWITCH_POS (its switch),
 *   X2 - X1 >= X_GRIPPER_MIN_GAP.
 * Moves are clipped to this range (apply_motion_limits). After G28 the gap is at
 * its maximum, so combined X (M606 S0) cannot move until a single-motor move
 * narrows the gap.
 */
bool dual_x_motion_limits(float &lo, float &hi) {
  if (dual_x_homed_bits != 0b11) return false;

  const float spmm = planner.settings.axis_steps_per_mm[X_AXIS],
              x1 = stepper.x1_steps() / spmm,
              x2 = stepper.x2_steps() / spmm;
  const bool a1 = stepper.x1_active(), a2 = stepper.x2_active();

  if (a1 && !a2) {          // X1 only
    lo = X_MIN_POS;
    hi = x2 - (X_GRIPPER_MIN_GAP);
  }
  else if (a2 && !a1) {     // X2 only
    lo = x1 + (X_GRIPPER_MIN_GAP);
    hi = X2_SWITCH_POS;
  }
  else {                    // Both: logical X is the midpoint
    const float half_gap = (x2 - x1) * 0.5f;
    lo = X_MIN_POS + half_gap;
    hi = X2_SWITCH_POS - half_gap;
  }

  if (hi < lo) lo = hi = current_position.x;  // Already outside the range: no X motion
  return true;
}

#if HAS_X_MAX

  // Move X2 (X1 locked) towards X max until the X max switch triggers.
  // On a hit, X2's position is set to X2_SWITCH_POS; otherwise to where it stopped.
  static bool x2_move_to_max(const float distance, const feedRate_t fr_mm_s) {
    const float start = current_position.x;
    endstops.hit_on_purpose();
    // do_homing_move zeroes the X step count, then moves; +X is not the G28
    // direction, so it skips the homing-failed check (which would kill).
    do_homing_move(X_AXIS, distance, fr_mm_s, false);
    const bool hit = TEST(endstops.trigger_state(), X_MAX);
    endstops.hit_on_purpose();
    current_position.x = hit ? float(X2_SWITCH_POS) : start + planner.get_axis_position_mm(X_AXIS);
    sync_plan_position();   // Sets X2 only (X1 is locked)
    return hit;
  }

  // Home X2 to its switch at X max. X1 does not move. Ends in M606 S0 with the
  // logical X at the gripper midpoint. Used by M606 H2 and by G28 (G28.cpp).
  // The caller enables/restores endstop checking (endstops.enable / not_homing).
  bool home_x2_to_max() {
    dual_x_select(2);   // X1 stays where it is; logical X = X2

    // Fast approach, like G28. X2 can be up to the full switch-to-switch distance away.
    bool hit = x2_move_to_max(1.1f * (X_GRIPPER_HOME_GAP), homing_feedrate(X_AXIS));

    // Back off, then approach again slowly (HOMING_BUMP_MM / HOMING_BUMP_DIVISOR)
    const float bump = home_bump_mm(X_AXIS);
    if (hit && bump) {
      current_position.x = X2_SWITCH_POS - bump;
      line_to_current_position(homing_feedrate(X_AXIS));
      planner.synchronize();
      hit = x2_move_to_max(2 * bump, get_homing_bump_feedrate(X_AXIS));
    }

    dual_x_set_homed(2, hit);
    if (hit)
      SERIAL_ECHO_MSG("X2 homed to X max");
    else
      SERIAL_ERROR_MSG("X2 homing failed: X max switch not triggered");

    dual_x_select(0);   // Back to M606 S0
    return hit;
  }

#endif // HAS_X_MAX

/**
 * M606: Select which X motor(s) move (X_DUAL_STEPPER_DRIVERS)
 *
 *   M606 S0       : Both X motors move together (default). Clears any single-motor lock.
 *   M606 S1 X1    : Only X1 moves. X2 is locked (its steps are skipped).
 *   M606 S1 X2    : Only X2 moves. X1 is locked.
 *   M606 H2       : Home X2 to its switch at X max (X1 stays put). Ends in M606 S0.
 *
 * G28 / G28 X home X1 to X min and then X2 to X max, each with the other motor locked.
 * With M606 S1 X1 active, G28 X homes only X1; with M606 S1 X2, only X2.
 * G28 always ends in M606 S0.
 *
 * Logical X is the gripper midpoint in S0 and the active gripper in S1 (see above).
 * M114 reports X1/X2. Once both are homed, X moves are clipped so that neither
 * gripper passes its switch and the grippers keep X_GRIPPER_MIN_GAP apart.
 */
void GcodeSuite::M606() {
  planner.synchronize();  // Finish queued motion before changing the locks

  if (parser.seen('H')) {
    #if HAS_X_MAX
      if (parser.value_byte() == 2) {
        endstops.enable(true);   // Check the switches during the homing moves
        home_x2_to_max();
        endstops.not_homing();   // Back to the M120/M121 setting
      }
      else
        SERIAL_ECHO_MSG("M606 H must be 2 (X1 homes with M606 S1 X1 + G28 X)");
    #else
      SERIAL_ECHO_MSG("M606 H2 needs an X max endstop");
    #endif
    return;
  }

  if (!parser.intval('S', 0)) {
    dual_x_select(0);
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

  dual_x_select(motor);
}

#endif // X_DUAL_STEPPER_DRIVERS
