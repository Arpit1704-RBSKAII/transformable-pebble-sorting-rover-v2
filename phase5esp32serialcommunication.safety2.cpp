// esp32/src/safety.cpp
//
// Implementation of the central emergency-stop module. See safety.h
// for design notes on why this is its own module.

#include "safety.h"
#include "motors.h"
#include "arm.h"
#include "servos.h"
#include "configuration.h"

static bool _emergencyStopped = false;

void safetyInit() {
  _emergencyStopped = false;
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

#if ENABLE_DEBUG_PRINTS
  Serial.println("[SAFETY] EMERGENCY STOP triggered - motors halted, arm/servos frozen.");
#endif
}

void clearEmergencyStop() {
  _emergencyStopped = false;
#if ENABLE_DEBUG_PRINTS
  Serial.println("[SAFETY] Emergency stop cleared - normal commands now accepted.");
#endif
}
