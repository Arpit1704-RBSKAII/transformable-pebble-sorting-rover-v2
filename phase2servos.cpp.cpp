/*
 * servos.cpp
 * ----------
 * Implements the generic multi-servo driver declared in servos.h.
 *
 * Uses the ESP32Servo library (NOT the stock Arduino Servo.h, which
 * is unreliable on ESP32 because of timer/PWM conflicts with other
 * peripherals like LEDC used by motors.cpp).
 *
 * Install via Library Manager: "ESP32Servo" by Kevin Harrington / John K. Bennett.
 *
 * Reused as-is from the existing repo. It previously would not compile
 * because configuration.h was missing SERVO_COUNT, the SERVO_IDX_*
 * table, servo pins, and the angle/timing constants below — that
 * section has now been added to configuration.h.
 *
 * KNOWN RISK TO RE-CHECK AT INTEGRATION (Phase 14, not now):
 * ESP32PWM::allocateTimer(0..3) below claims all 4 hardware PWM timers
 * for servo use. motors.cpp uses analogWrite() for motor PWM, which on
 * the ESP32 Arduino core also allocates LEDC timers automatically.
 * Running motors + servos together for the first time should include a
 * check that neither glitches/jitters the other.
 */

#include <ESP32Servo.h>
#include "servos.h"
#include "configuration.h"

// Pin table - ORDER MUST MATCH the SERVO_IDX_* defines in configuration.h
static const int SERVO_PINS[SERVO_COUNT] = {
  SERVO_ARM_BASE_PIN,     // SERVO_IDX_ARM_BASE
  SERVO_ARM_SHOULDER_PIN, // SERVO_IDX_ARM_SHOULDER
  SERVO_ARM_ELBOW_PIN,    // SERVO_IDX_ARM_ELBOW
  SERVO_GRIPPER_PIN,      // SERVO_IDX_GRIPPER
  SERVO_TRANSFORM_PIN     // SERVO_IDX_TRANSFORM
};

static Servo servos[SERVO_COUNT];
static int currentAngle[SERVO_COUNT];
static int targetAngle[SERVO_COUNT];
static unsigned long lastTickMs = 0;

// Per-servo min/max. All default to the same conservative range for now;
// Phase 4 can override individual entries here once real mechanical
// limits are measured on the physical arm.
static int servoMinAngle[SERVO_COUNT];
static int servoMaxAngle[SERVO_COUNT];

static int clampAngle(int index, int angle) {
  if (angle < servoMinAngle[index]) return servoMinAngle[index];
  if (angle > servoMaxAngle[index]) return servoMaxAngle[index];
  return angle;
}

void servosInit() {
  // ESP32Servo needs timer allocation set up before attaching servos.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < SERVO_COUNT; i++) {
    servoMinAngle[i] = SERVO_MIN_ANGLE_DEFAULT;
    servoMaxAngle[i] = SERVO_MAX_ANGLE_DEFAULT;

    servos[i].setPeriodHertz(50); // standard 50Hz servo signal
    servos[i].attach(SERVO_PINS[i], 500, 2400); // typical pulse width range, adjust if servo datasheet differs

    // Start at the midpoint of the safe range rather than assuming
    // 0 or 90 is safe - midpoint is the least likely to hit a hard
    // mechanical stop on an unknown/uncalibrated build.
    int startAngle = (servoMinAngle[i] + servoMaxAngle[i]) / 2;
    currentAngle[i] = startAngle;
    targetAngle[i] = startAngle;
    servos[i].write(startAngle);
  }

  lastTickMs = millis();
  Serial.println("[servos] Initialized all servos to safe midpoint.");
}

void setServoAngle(int index, int requestedAngle) {
  if (index < 0 || index >= SERVO_COUNT) {
    Serial.println("[servos] ERROR: invalid servo index.");
    return;
  }
  targetAngle[index] = clampAngle(index, requestedAngle);
}

int getServoAngle(int index) {
  if (index < 0 || index >= SERVO_COUNT) return -1;
  return currentAngle[index];
}

bool isServoAtTarget(int index) {
  if (index < 0 || index >= SERVO_COUNT) return false;
  return currentAngle[index] == targetAngle[index];
}

void servosUpdate() {
  unsigned long now = millis();
  if (now - lastTickMs < SERVO_TICK_MS) {
    return;
  }
  lastTickMs = now;

  for (int i = 0; i < SERVO_COUNT; i++) {
    if (currentAngle[i] == targetAngle[i]) continue;

    if (currentAngle[i] < targetAngle[i]) {
      currentAngle[i] = min(currentAngle[i] + SERVO_MAX_STEP_PER_TICK, targetAngle[i]);
    } else {
      currentAngle[i] = max(currentAngle[i] - SERVO_MAX_STEP_PER_TICK, targetAngle[i]);
    }
    servos[i].write(currentAngle[i]);
  }
}

void servosHalt() {
  // Freeze all targets at their current position - do NOT move further,
  // but also do not release/detach (that could let an arm drop under gravity).
  for (int i = 0; i < SERVO_COUNT; i++) {
    targetAngle[i] = currentAngle[i];
  }
  Serial.println("[servos] Halted - all targets frozen at current position.");
}
