// esp32/src/transformation.cpp
//
// Implementation of the transformation mechanism. See transformation.h
// for the redesign rationale (reuses the shared servo slot instead of
// a duplicate Servo object; non-blocking instead of the original's
// blocking while+delay() loop; actively stops drive motors instead of
// only checking a sensor pin).

#include "transformation.h"
#include "configuration.h"
#include "servos.h"
#include "arm.h"
#include "gripper.h"
#include "motors.h"
#include "safety.h"

static TransformState _state = TRANSFORM_STATE_UNFOLDED;
static unsigned long _moveStartMs = 0;

static bool _readPinSafe(int pin) {
  if (pin < 0) return true; // not configured -> treat as OK
  pinMode(pin, INPUT_PULLUP);
  return digitalRead(pin) == HIGH;
}

// Primary preconditions use REAL tracked software state, same
// rationale as sorter.cpp (Phase 12) - unconfigured floating pins
// default to "OK" and provide no actual protection on their own.
static bool _preconditionsOk(String &errOut) {
  if (isEmergencyStopped()) {
    errOut = "EMERGENCY_STOP_ACTIVE";
    return false;
  }
  if (!isArmAtHome()) {
    errOut = "ARM_NOT_HOME";
    return false;
  }
  if (getGripperState() != GRIPPER_STATE_CLOSED) {
    errOut = "GRIPPER_NOT_CLOSED";
    return false;
  }
  if (!_readPinSafe(PIN_ARM_HOME_SWITCH)) {
    errOut = "ARM_HOME_SWITCH_NOT_CONFIRMED";
    return false;
  }
  if (!_readPinSafe(PIN_GRIPPER_CLOSED_SWITCH)) {
    errOut = "GRIPPER_CLOSED_SWITCH_NOT_CONFIRMED";
    return false;
  }
  return true;
}

void transformInit() {
  _state = TRANSFORM_STATE_UNKNOWN;
  _moveStartMs = 0;
  // Deliberately does NOT command a servo move here - servosInit()
  // already placed every servo at a safe midpoint, which is neither
  // confirmed FOLDED nor UNFOLDED. Same reasoning as arm.cpp/
  // gripper.cpp: state stays UNKNOWN until the first real
  // foldTransform()/unfoldTransform() call actually completes.
#if ENABLE_DEBUG_PRINTS
  Serial.println("[TRANSFORM] Initialized (position unknown until first command).");
#endif
}

static bool _startMove(int targetAngle, TransformState movingState) {
  if (_state == TRANSFORM_STATE_FOLDING || _state == TRANSFORM_STATE_UNFOLDING) {
#if ENABLE_DEBUG_PRINTS
    Serial.println("[TRANSFORM] Already moving, rejecting new request.");
#endif
    return false;
  }

  String err;
  if (!_preconditionsOk(err)) {
#if ENABLE_DEBUG_PRINTS
    Serial.printf("[TRANSFORM] Preconditions failed: %s\n", err.c_str());
#endif
    return false;
  }

  // Per spec section 12: "During transformation: Stop drive motors."
  // Actively commanded here, not just checked via an optional sensor.
  stopRobot();

  setServoAngle(SERVO_IDX_TRANSFORM, targetAngle);
  _moveStartMs = millis();
  _state = movingState;
#if ENABLE_DEBUG_PRINTS
  Serial.printf("[TRANSFORM] Moving to angle %d\n", targetAngle);
#endif
  return true;
}

bool foldTransform() {
  return _startMove(TRANSFORM_ANGLE_FOLDED, TRANSFORM_STATE_FOLDING);
}

bool unfoldTransform() {
  return _startMove(TRANSFORM_ANGLE_STOW, TRANSFORM_STATE_UNFOLDING);
}

void transformUpdate() {
  if (_state != TRANSFORM_STATE_FOLDING && _state != TRANSFORM_STATE_UNFOLDING) {
    return;
  }

  // Keep drive motors stopped for the ENTIRE duration of the move, not
  // just at the start - if something else tried to command a move
  // mid-transform, this stomps it back to stopped every tick.
  stopRobot();

  if (millis() - _moveStartMs > TRANSFORM_MOVE_TIMEOUT_MS) {
#if ENABLE_DEBUG_PRINTS
    Serial.println("[TRANSFORM] Move timed out - stopping in place.");
#endif
    // Timed out mid-motion: the servo did NOT confirm reaching its
    // target, so (same reasoning as transformEmergencyStop()) the
    // honest state is UNKNOWN, not a guess at whichever end it was
    // closer to.
    _state = TRANSFORM_STATE_UNKNOWN;
    return;
  }

  if (isServoAtTarget(SERVO_IDX_TRANSFORM)) {
    _state = (_state == TRANSFORM_STATE_FOLDING) ? TRANSFORM_STATE_FOLDED : TRANSFORM_STATE_UNFOLDED;

    // Optional limit-switch confirmation - warn only, per spec's
    // "TODO: ADD LIMIT SWITCHES IF REQUIRED" (not a hard requirement).
    int confirmPin = (_state == TRANSFORM_STATE_FOLDED) ? PIN_TRANSFORM_LIMIT_FOLDED : PIN_TRANSFORM_LIMIT_UNFOLDED;
    if (confirmPin >= 0) {
      pinMode(confirmPin, INPUT_PULLUP);
      if (digitalRead(confirmPin) != LOW) {
#if ENABLE_DEBUG_PRINTS
        Serial.println("[TRANSFORM] Warning: limit switch did not confirm final position.");
#endif
      }
    }

#if ENABLE_DEBUG_PRINTS
    Serial.println(_state == TRANSFORM_STATE_FOLDED ? "[TRANSFORM] Folded." : "[TRANSFORM] Unfolded.");
#endif
  }
}

bool isTransformBusy() {
  return _state == TRANSFORM_STATE_FOLDING || _state == TRANSFORM_STATE_UNFOLDING;
}

bool isTransformFolded() {
  return _state == TRANSFORM_STATE_FOLDED;
}

bool isTransformUnfolded() {
  return _state == TRANSFORM_STATE_UNFOLDED;
}

void transformEmergencyStop() {
  // Freeze in place - do not snap to either end state. We don't
  // actually know the real mechanical position when interrupted
  // mid-motion, so (same as arm.cpp's stopArm()) the correct state is
  // UNKNOWN, not a guess at either FOLDED or UNFOLDED - a caller that
  // trusted a wrong guess here could make an unsafe decision (e.g.
  // driving while the mechanism is actually still half-folded).
  int currentAngle = getServoAngle(SERVO_IDX_TRANSFORM);
  setServoAngle(SERVO_IDX_TRANSFORM, currentAngle);
  _state = TRANSFORM_STATE_UNKNOWN;
#if ENABLE_DEBUG_PRINTS
  Serial.println("[TRANSFORM] EMERGENCY STOP - halted in place, position now UNKNOWN.");
#endif
}
