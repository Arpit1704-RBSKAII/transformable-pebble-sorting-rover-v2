/*
 * arm.h
 * -----
 * Semantic control of the 3-joint robotic arm (base, shoulder, elbow),
 * built on top of the generic servo driver (servos.h/.cpp) from Phase 2.
 *
 * Uses predefined/calibrated joint-angle PRESETS rather than inverse
 * kinematics, per project scope. All preset angles live in
 * configuration.h so they can be recalibrated without touching logic.
 *
 * This module owns the arm's POSITION state so other modules -
 * especially transformation.cpp (Phase 13) - can verify "arm is home
 * and settled" before performing an unsafe operation like folding.
 *
 * Reused from the existing repo (already well-designed) with one
 * addition: stopArm() was required by the original project spec
 * (section 9 - arm must have a stop/interrupt function) but did not
 * exist anywhere in the repo.
 */

#ifndef ARM_H
#define ARM_H

#include <Arduino.h>

// Which named pose the arm is currently at, moving toward, or unknown.
enum ArmPosition {
  ARM_POS_HOME,
  ARM_POS_PICKUP,
  ARM_POS_BIN_SMALL,
  ARM_POS_BIN_MEDIUM,
  ARM_POS_BIN_LARGE,
  ARM_POS_MOVING,   // in transit toward a target preset
  ARM_POS_UNKNOWN   // e.g. right after boot, or right after stopArm()
};

// Pebble size classification - shared with sorter.cpp (Phase 12) and
// the ESP32 command parser (Phase 5), since "SORT,SMALL" etc. need
// this same enum to select a bin.
enum PebbleSize {
  PEBBLE_SIZE_SMALL,
  PEBBLE_SIZE_MEDIUM,
  PEBBLE_SIZE_LARGE
};

// Call once in setup() (after servosInit()). Does NOT command a move -
// arm stays wherever servosInit() left it (safe midpoint) until a
// move*() function is explicitly called, same rationale as gripperInit().
void armInit();

// Commands the arm toward each named preset. Non-blocking - actual
// motion happens via servosUpdate() in loop(); call armUpdate() to
// track when the move completes.
void moveArmHome();
void moveArmToPickup();
void moveArmToBin(PebbleSize size);

// Immediately halts all 3 arm joints at their CURRENT angle (does not
// snap back to any preset - snapping could be more dangerous than
// holding position). Position becomes ARM_POS_UNKNOWN afterward, since
// the arm is no longer guaranteed to be at any named preset.
void stopArm();

// Current known position. Returns ARM_POS_MOVING while any of the
// 3 joints has not yet reached its target.
ArmPosition getArmPosition();

// True only when position is a named settled preset (NOT moving/unknown).
bool isArmSettled();

// Convenience check used by transformation.cpp: true only when the
// arm is BOTH settled AND at the home preset specifically.
bool isArmAtHome();

// Must be called frequently from loop() - polls all 3 arm servo
// positions and updates the internal ArmPosition.
void armUpdate();

#endif // ARM_H
