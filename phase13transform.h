// esp32/include/transformation.h
//
// Transformation mechanism: switches between SORTING mode (unfolded)
// and COMPACT ROVER mode (folded). Per project spec section 12.
//
// REDESIGNED from the repo's original version in two ways:
//
// 1. Uses the EXISTING SERVO_IDX_TRANSFORM slot in servos.h (defined
//    since Phase 2) instead of a second, independent Servo object on
//    a separate pin. See configuration.h's TRANSFORMATION section for
//    why the original's separate TRANSFORM_SERVO_PIN would have been
//    a real conflict.
//
// 2. Non-blocking, matching arm.cpp/gripper.cpp/sorter.cpp - the
//    original ran its motion inside a blocking while+delay() loop,
//    which (same as the Phase 12 sorter bug) meant an emergency stop
//    couldn't be processed until the fold/unfold motion finished on
//    its own. This is the single largest mechanical motion in the
//    whole robot - it's the LAST module that should be allowed to
//    block emergency stop.
//
// Also fixes a real safety gap: the original's "preconditions" only
// CHECKED an optional, unconfigured (-1 by default) wheels-stopped
// SENSOR pin - it never actually commanded the drive motors to stop.
// Per spec section 12: "During transformation: Stop drive motors" -
// this version calls stopRobot() directly rather than hoping a sensor
// confirms it.

#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Arduino.h>

enum TransformState {
  TRANSFORM_STATE_UNFOLDED,   // sorting mode, settled
  TRANSFORM_STATE_FOLDING,
  TRANSFORM_STATE_FOLDED,     // rover mode, settled
  TRANSFORM_STATE_UNFOLDING,
  TRANSFORM_STATE_UNKNOWN,    // boot, or interrupted mid-motion - not a named preset
};

// Call once in setup(), AFTER servosInit()/armInit()/gripperInit()/
// motorsInit() (preconditions check all of their state).
void transformInit();

// Non-blocking: validates preconditions (arm home, gripper closed,
// drive motors actively stopped, no active emergency) and starts the
// fold/unfold motion if they pass. Returns true if ACCEPTED and
// started (not completed) - check isTransformBusy()/isTransformFolded()
// for that. Returns false if a precondition failed or already busy.
bool foldTransform();
bool unfoldTransform();

// Must be called frequently from loop() - advances the servo-ramp-
// backed motion via servos.h.
void transformUpdate();

bool isTransformBusy();
bool isTransformFolded();
bool isTransformUnfolded();

// Immediately halts at the current angle. Used by safety.cpp's
// triggerEmergencyStop().
void transformEmergencyStop();

#endif // TRANSFORMATION_H
