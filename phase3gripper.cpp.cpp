// esp32/src/gripper.cpp
//
// Implementation of the semantic gripper layer. See gripper.h for the
// public API and design notes.

#include "gripper.h"
#include "servos.h"
#include "configuration.h"

static GripperState _intendedState = GRIPPER_STATE_UNKNOWN;

void gripperInit() {
  // Start from a known-safe state. Open is the safer default: a
  // gripper that starts closed could be clamping onto nothing (fine)
  // or already under load if something was left in it (not fine).
  openGripper();
#if ENABLE_DEBUG_PRINTS
  Serial.println("[GRIPPER] Initialized, commanded to OPEN.");
#endif
}

void openGripper() {
#if ENABLE_DEBUG_PRINTS
  Serial.println("[GRIPPER] Commanding OPEN");
#endif
  setServoAngle(SERVO_IDX_GRIPPER, GRIPPER_OPEN_ANGLE);
  _intendedState = GRIPPER_STATE_UNKNOWN; // unknown until servo settles
}

void closeGripper() {
#if ENABLE_DEBUG_PRINTS
  Serial.println("[GRIPPER] Commanding CLOSE");
#endif
  setServoAngle(SERVO_IDX_GRIPPER, GRIPPER_CLOSE_ANGLE);
  _intendedState = GRIPPER_STATE_UNKNOWN; // unknown until servo settles
}

void gripperUpdate() {
  // Once the gripper servo has visually reached its target, resolve
  // the intended state based on which angle it settled at.
  if (isServoAtTarget(SERVO_IDX_GRIPPER)) {
    int angle = getServoAngle(SERVO_IDX_GRIPPER);

    // Whichever target angle is closer to the settled angle wins —
    // avoids requiring an exact match if clamping nudged the value.
    int distToOpen  = abs(angle - GRIPPER_OPEN_ANGLE);
    int distToClose = abs(angle - GRIPPER_CLOSE_ANGLE);

    if (distToOpen <= distToClose) {
      _intendedState = GRIPPER_STATE_OPEN;
    } else {
      _intendedState = GRIPPER_STATE_CLOSED;
    }
  }
  // If not yet at target, state stays UNKNOWN (still mid-motion).
}

bool isGripperSettled() {
  return isServoAtTarget(SERVO_IDX_GRIPPER);
}

GripperState getGripperState() {
  return _intendedState;
}
