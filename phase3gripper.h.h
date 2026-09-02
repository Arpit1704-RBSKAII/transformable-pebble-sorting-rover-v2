// esp32/include/gripper.h
//
// Semantic gripper control, built on top of the generic servos.h
// driver (specifically SERVO_IDX_GRIPPER). This module is what
// translates "open"/"close" into actual servo angles and tracks
// gripper state.
//
// Per project spec: a commanded servo angle is NOT proof of a
// successful grip. This module tracks the INTENDED state (did we
// command open/close and did the servo reach that position), not
// whether a pebble is actually held. A limit switch or force sensor
// would be needed for that — PIN_GRIPPER_CLOSED_SWITCH exists as a
// placeholder in configuration.h for future use, but is not wired in
// here yet.

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

enum GripperState {
  GRIPPER_STATE_OPEN,
  GRIPPER_STATE_CLOSED,
  GRIPPER_STATE_UNKNOWN  // mid-motion, or not yet initialized
};

// Call once in setup(), AFTER servosInit(). Commands the gripper to a
// known-safe starting state (open).
void gripperInit();

void openGripper();
void closeGripper();

// Must be called frequently from loop() (alongside servosUpdate()) so
// state tracking updates as the servo ramps toward its target.
void gripperUpdate();

// True once the gripper servo has reached its last commanded position.
bool isGripperSettled();

// Last known/intended state. GRIPPER_STATE_UNKNOWN while mid-motion.
GripperState getGripperState();

#endif // GRIPPER_H
