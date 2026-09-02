// esp32/include/servos.h
//
// Generic, low-level multi-servo driver.
//
// This module does NOT know what "arm home" or "gripper open" means -
// that semantic mapping is added in Phase 3 (gripper) and Phase 4 (arm),
// which will call setServoAngle(index, angle) with values from
// configuration.h constants like GRIPPER_OPEN_ANGLE.
//
// Responsibilities of this module only:
//  - Attach all servos defined in configuration.h
//  - Clamp every angle write to a safe per-servo range
//  - Optionally ramp toward a target angle instead of jumping instantly
//
// Reused as-is from the existing repo — this module was already well
// designed. The only thing missing was the configuration.h section it
// depends on (SERVO_COUNT, SERVO_IDX_*, pins, angle/timing constants),
// which has now been added.

#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>

// Call once in setup(). Attaches all servos and moves them to a
// neutral/current position WITHOUT assuming any semantic meaning.
void servosInit();

// Request a servo move to targetAngle (degrees). Internally clamped
// to that servo's configured safe min/max. index must be one of the
// SERVO_IDX_* constants from configuration.h.
void setServoAngle(int index, int targetAngle);

// Returns the last commanded (post-clamp) angle for a servo.
int getServoAngle(int index);

// Returns true once the servo has visually reached its target
// (only meaningful if ramping is enabled - otherwise always true
// immediately after setServoAngle).
bool isServoAtTarget(int index);

// Must be called frequently from loop() to advance ramped movement.
void servosUpdate();

// Emergency stop: halts ramping in place (does NOT snap to any angle -
// snapping could be more dangerous than holding position).
void servosHalt();

#endif // SERVOS_H
