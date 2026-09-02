// esp32/include/motors.h
//
// Four-wheel drive subsystem: 4 real DC motors (front-left, rear-left,
// front-right, rear-right), driven by 2x L298N dual H-bridge modules.
// Software commands them in PAIRED left/right groups — front+rear on
// the same side always move identically — while each motor still gets
// its own dedicated PWM + IN1/IN2 pins at the hardware level.
//
// This module owns all direct pin access for driving. Nothing outside
// motors.cpp should touch the motor pins directly.

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "configuration.h"

// ---- Lifecycle ----

// Configures all 4 motors' pins. Safe to call even if pins are still -1
// (TODO/unconfigured) — those channels simply run in no-op mode.
void motorsInit();

// Call this frequently (every loop() iteration). Advances speed ramping
// toward whatever target speed was last requested for each channel.
void motorsUpdate();

// ---- High-level movement API (required by project spec) ----
// speedPercent is 0-100. Values outside that range are clamped.

void moveForward(int speedPercent);
void moveBackward(int speedPercent);
void turnLeft(int speedPercent);   // In-place turn: left side back, right side forward
void turnRight(int speedPercent);  // In-place turn: left side forward, right side back
void stopRobot();

// Immediate stop, bypassing ramping. Used by the emergency-stop path.
void emergencyStopMotors();

// True if any of the 4 motors currently has nonzero PWM applied.
bool motorsAreMoving();

// ---- Low-level per-side control ----
// Sets BOTH motors on a side (front + rear) together, since they are
// paired in software. forward=true -> spin forward, false -> backward.
//
// SAFETY NOTE: if a side is currently spinning and asked to reverse
// direction, current PWM is forced to 0 immediately before the IN1/IN2
// pins flip, rather than reversing through a live PWM value.
void setLeftSide(int speedPercent, bool forward);
void setRightSide(int speedPercent, bool forward);

#endif // MOTORS_H
