// esp32/src/sorter.cpp
//
// Non-blocking implementation of the sorting mechanism. See sorter.h
// for the redesign rationale (the original was fully blocking, which
// made emergency stop non-responsive during a sort motion).
//
// PRECONDITION CHECK CHANGE vs. the original: the original checked
// raw GPIO pins (PIN_ARM_HOME_SWITCH, PIN_GRIPPER_CLOSED_SWITCH) that
// default to -1 (unconfigured) and are therefore ALWAYS treated as
// "safe" out of the box - meaning the original preconditions provided
// NO actual protection until someone wires physical switches. This
// version checks the REAL software state we already track and have
// tested (arm.h's isArmAtHome(), gripper.h's getGripperState()) as the
// PRIMARY check, with the physical switches (if wired) as an
// additional secondary layer, not a replacement.

#include "sorter.h"
#include "configuration.h"
#include "arm.h"
#include "gripper.h"
#include <ESP32Servo.h>

static Servo _servo;
static SorterState _state = SORTER_STATE_STOWED;
static int _currentAngle = SORTER_ANGLE_STOW;
static int _targetAngle = SORTER_ANGLE_STOW;
static unsigned long _lastStepMs = 0;
static unsigned long _moveStartMs = 0;
static unsigned long _atBinSinceMs = 0;

static const unsigned long AT_BIN_PAUSE_MS = 200; // brief pause at the bin angle before auto-stowing

void sorterInit() {
  _servo.setPeriodHertz(50);
  _servo.attach(SORTER_SERVO_PIN, 500, 2400);
  _currentAngle = SORTER_ANGLE_STOW;
  _targetAngle = SORTER_ANGLE_STOW;
  _servo.write(_currentAngle);
  _state = SORTER_STATE_STOWED;
#if ENABLE_DEBUG_PRINTS
  Serial.println("[SORTER] Initialized, stowed.");
#endif
}

// Optional secondary check: pin < 0 means "not wired", always passes.
static bool _readPinSafe(int pin) {
  if (pin < 0) return true;
  pinMode(pin, INPUT_PULLUP);
  return digitalRead(pin) == HIGH;
}

// Primary preconditions use REAL tracked software state (arm/gripper),
// not unconfigured floating pins. Physical switches, if wired, are
// checked as an additional layer on top.
static bool _preconditionsOk(String &errOut) {
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
  if (!_readPinSafe(PIN_WHEELS_STOPPED_SIGNAL)) {
    errOut = "WHEELS_NOT_STOPPED";
    return false;
  }
  if (PIN_EMERGENCY_STOP >= 0) {
    pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);
    if (digitalRead(PIN_EMERGENCY_STOP) == LOW) {
      errOut = "EMERGENCY_STOP_ACTIVE";
      return false;
    }
  }
  return true;
}

static bool _startMove(int targetAngle) {
  if (_state == SORTER_STATE_MOVING_TO_BIN || _state == SORTER_STATE_STOWING) {
#if ENABLE_DEBUG_PRINTS
    Serial.println("[SORTER] Already moving, rejecting new request.");
#endif
    return false;
  }

  String err;
  if (!_preconditionsOk(err)) {
#if ENABLE_DEBUG_PRINTS
    Serial.printf("[SORTER] Preconditions failed: %s\n", err.c_str());
#endif
    return false;
  }

  _targetAngle = targetAngle;
  _moveStartMs = millis();
  _state = SORTER_STATE_MOVING_TO_BIN;
#if ENABLE_DEBUG_PRINTS
  Serial.printf("[SORTER] Moving to angle %d\n", targetAngle);
#endif
  return true;
}

bool sortSmall()  { return _startMove(SORTER_ANGLE_SMALL); }
bool sortMedium() { return _startMove(SORTER_ANGLE_MEDIUM); }
bool sortLarge()  { return _startMove(SORTER_ANGLE_LARGE); }

bool sortReset() {
  if (_state == SORTER_STATE_MOVING_TO_BIN || _state == SORTER_STATE_STOWING) {
    return false; // already busy - let the current move finish or be e-stopped
  }
  _targetAngle = SORTER_ANGLE_STOW;
  _moveStartMs = millis();
  _state = SORTER_STATE_STOWING;
  return true;
}

void sorterUpdate() {
  unsigned long now = millis();

  switch (_state) {
    case SORTER_STATE_STOWED:
      return; // nothing to do

    case SORTER_STATE_MOVING_TO_BIN:
    case SORTER_STATE_STOWING: {
      if (now - _moveStartMs > SORTER_MOVE_TIMEOUT_MS) {
#if ENABLE_DEBUG_PRINTS
        Serial.println("[SORTER] Move timed out - stopping in place.");
#endif
        _state = (_state == SORTER_STATE_MOVING_TO_BIN) ? SORTER_STATE_AT_BIN : SORTER_STATE_STOWED;
        return;
      }

      if (now - _lastStepMs < SORTER_STEP_DELAY_MS) {
        return; // not time for the next step yet
      }
      _lastStepMs = now;

      if (_currentAngle == _targetAngle) {
        // Reached target for this leg of the move.
        if (_state == SORTER_STATE_MOVING_TO_BIN) {
          _state = SORTER_STATE_AT_BIN;
          _atBinSinceMs = now;
        } else {
          _state = SORTER_STATE_STOWED;
#if ENABLE_DEBUG_PRINTS
          Serial.println("[SORTER] Stowed.");
#endif
        }
        return;
      }

      int step = (_targetAngle > _currentAngle) ? SORTER_MOVE_STEP_DEG : -SORTER_MOVE_STEP_DEG;
      int next = _currentAngle + step;
      if ((step > 0 && next > _targetAngle) || (step < 0 && next < _targetAngle)) {
        next = _targetAngle;
      }
      _currentAngle = next;
      _servo.write(_currentAngle);
      return;
    }

    case SORTER_STATE_AT_BIN: {
      // Brief pause at the bin angle (lets a pebble actually drop
      // before the chute starts moving back), then auto-stow.
      if (now - _atBinSinceMs >= AT_BIN_PAUSE_MS) {
        _targetAngle = SORTER_ANGLE_STOW;
        _moveStartMs = now;
        _state = SORTER_STATE_STOWING;
      }
      return;
    }
  }
}

bool isSorterBusy() {
  return _state != SORTER_STATE_STOWED;
}

bool isSorterSettled() {
  return _state == SORTER_STATE_STOWED;
}

void sorterEmergencyStop() {
  _targetAngle = _currentAngle; // freeze in place, don't snap anywhere
  _state = SORTER_STATE_STOWED; // treat as idle - a fresh sort command will re-check preconditions
#if ENABLE_DEBUG_PRINTS
  Serial.println("[SORTER] EMERGENCY STOP - halted in place.");
#endif
}
