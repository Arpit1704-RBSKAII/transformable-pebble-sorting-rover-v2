// esp32/src/safety.cpp
//
// Implementation of the central emergency-stop module. See safety.h
// for design notes on why this is its own module.

#include "safety.h"
#include "motors.h"
#include "arm.h"
#include "servos.h"
#include "sorter.h"
#include "transformation.h"
#include "configuration.h"

static bool _emergencyStopped = false;
static bool _hardwarePinInitialized = false;

void safetyInit() {
  _emergencyStopped = false;
  if (PIN_EMERGENCY_STOP >= 0) {
    pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);
    _hardwarePinInitialized = true;
  }
}

bool isEmergencyStopped() {
  return _emergencyStopped;
}

void triggerEmergencyStop() {
  _emergencyStopped = true;

  // Order matters: stopArm() must run before/instead of a generic
  // servosHalt()-only approach, because it ALSO updates arm.cpp's own
  // internal position state to UNKNOWN. If only servosHalt() were
  // called, armUpdate() would later see "all 3 joints stopped moving"
  // and incorrectly conclude the arm reached whatever preset it was
  // heading toward, even though it was actually interrupted mid-move.
  emergencyStopMotors();
  stopArm();

  // Freezes any OTHER servo (gripper, transformation) that stopArm()
  // doesn't touch. Redundant-but-harmless for the arm's own 3 servos,
  // since stopArm() already froze them the same way.
  servosHalt();

  // Phase 12 addition: halts the sorter in place. Safe to call even if
  // the sorter isn't mid-move - sorterEmergencyStop() is a no-op-ish
  // "freeze wherever you are" regardless of current state.
  sorterEmergencyStop();

  // Phase 13 addition: halts the transformation mechanism in place,
  // marking its position UNKNOWN rather than guessing folded/unfolded.
  transformEmergencyStop();

#if ENABLE_DEBUG_PRINTS
  Serial.println("[SAFETY] EMERGENCY STOP triggered - motors halted, arm/servos/sorter/transform frozen.");
#endif
}

void clearEmergencyStop() {
  _emergencyStopped = false;
#if ENABLE_DEBUG_PRINTS
  Serial.println("[SAFETY] Emergency stop cleared - normal commands now accepted.");
#endif
}

void safetyUpdate() {
  if (!_hardwarePinInitialized) {
    return; // PIN_EMERGENCY_STOP is -1 (not wired) - nothing to poll
  }
  if (_emergencyStopped) {
    return; // already stopped, no need to keep checking
  }

  // Assumes a normally-closed-to-HIGH button wired with INPUT_PULLUP:
  // LOW means pressed. See configuration.h's PIN_EMERGENCY_STOP comment.
  if (digitalRead(PIN_EMERGENCY_STOP) == LOW) {
#if ENABLE_DEBUG_PRINTS
    Serial.println("[SAFETY] Hardware E-STOP button pressed!");
#endif
    triggerEmergencyStop();
  }
}
