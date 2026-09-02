// esp32/include/safety.h
//
// Central emergency-stop state. This is deliberately its OWN module
// rather than living inside motors.cpp or communication.cpp, because
// an e-stop is a cross-cutting concern: per project spec, it must
// immediately stop drive motors AND prevent normal motion commands
// (arm/gripper/sort) from being accepted until cleared. Putting it in
// motors.cpp would make it look motor-specific when it isn't.
//
// communication.cpp referenced isEmergencyStopped()/triggerEmergencyStop()/
// clearEmergencyStop() already, but none of these existed anywhere in
// the repo - this module fills that gap.

#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>

// Call once in setup(), after motors/servos/gripper/arm are initialized.
void safetyInit();

// True from the moment triggerEmergencyStop() is called until
// clearEmergencyStop() is called. Checked by communication.cpp before
// accepting any MOVE/ARM/GRIP/SORT command.
bool isEmergencyStopped();

// Immediately halts ALL actuators:
//   - drive motors (via emergencyStopMotors(), bypassing ramping)
//   - the 3 arm joints (via stopArm(), which ALSO correctly marks the
//     arm's position as UNKNOWN rather than falsely completing
//     whatever preset it was moving toward)
//   - gripper + transformation servos (via servosHalt(), freezing them
//     at their current position)
// Sets the emergency-stop flag so subsequent commands are rejected
// until clearEmergencyStop() is called.
void triggerEmergencyStop();

// Clears the emergency-stop flag ONLY. Does not re-home or resume any
// previous action - the next command from Python must explicitly
// re-command whatever state is needed (e.g. ARM,HOME).
void clearEmergencyStop();

#endif // SAFETY_H
