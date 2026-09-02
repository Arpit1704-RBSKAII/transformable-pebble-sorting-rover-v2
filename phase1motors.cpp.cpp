// esp32/src/motors.cpp
//
// Implementation of the four-motor, two-L298N-driver, paired-L/R
// differential drive subsystem. See motors.h for the public API and
// configuration.h for pin/speed configuration.
//
// Each of the 4 real motors (front-left, rear-left, front-right,
// rear-right) has its own PWM pin + IN1/IN2 direction pins (L298N
// style). setLeftSide()/setRightSide() drive both motors on a side
// identically, so in software the rover still behaves like a 2-channel
// differential drive — it's just backed by 4 real motor channels
// instead of 2.

#include "motors.h"

// ---- One struct per physical motor channel ----
struct MotorChannel {
  int pwmPin;
  int in1Pin;
  int in2Pin;
  int currentPWM;
  int targetPWM;
  bool forward;
};

static MotorChannel _frontLeft  = { MOTOR_FRONT_LEFT_PWM_PIN,  MOTOR_FRONT_LEFT_IN1_PIN,  MOTOR_FRONT_LEFT_IN2_PIN,  0, 0, true };
static MotorChannel _rearLeft   = { MOTOR_REAR_LEFT_PWM_PIN,   MOTOR_REAR_LEFT_IN1_PIN,   MOTOR_REAR_LEFT_IN2_PIN,   0, 0, true };
static MotorChannel _frontRight = { MOTOR_FRONT_RIGHT_PWM_PIN, MOTOR_FRONT_RIGHT_IN1_PIN, MOTOR_FRONT_RIGHT_IN2_PIN, 0, 0, true };
static MotorChannel _rearRight  = { MOTOR_REAR_RIGHT_PWM_PIN,  MOTOR_REAR_RIGHT_IN1_PIN,  MOTOR_REAR_RIGHT_IN2_PIN,  0, 0, true };

static unsigned long _lastRampMs = 0;

// ---- Helpers ----

static int percentToPWM(int speedPercent) {
  if (speedPercent < 0) speedPercent = 0;
  if (speedPercent > MOTOR_MAX_SPEED_PERCENT) speedPercent = MOTOR_MAX_SPEED_PERCENT;
  return (speedPercent * MOTOR_PWM_MAX) / MOTOR_MAX_SPEED_PERCENT;
}

static void initChannel(const MotorChannel &ch) {
  if (ch.pwmPin >= 0) pinMode(ch.pwmPin, OUTPUT);
  if (ch.in1Pin >= 0) pinMode(ch.in1Pin, OUTPUT);
  if (ch.in2Pin >= 0) pinMode(ch.in2Pin, OUTPUT);
}

// L298N direction truth table:
//   forward:  IN1=HIGH, IN2=LOW
//   backward: IN1=LOW,  IN2=HIGH
// PWM pin at 0 duty stops the motor regardless of IN1/IN2 state.
static void applyChannel(const MotorChannel &ch) {
  if (ch.in1Pin >= 0 && ch.in2Pin >= 0) {
    digitalWrite(ch.in1Pin, ch.forward ? HIGH : LOW);
    digitalWrite(ch.in2Pin, ch.forward ? LOW  : HIGH);
  }
  if (ch.pwmPin >= 0) {
    analogWrite(ch.pwmPin, ch.currentPWM);
  }
}

static void rampChannel(MotorChannel &ch) {
  if (ch.currentPWM < ch.targetPWM) {
    ch.currentPWM += MOTOR_RAMP_STEP_PWM;
    if (ch.currentPWM > ch.targetPWM) ch.currentPWM = ch.targetPWM;
  } else if (ch.currentPWM > ch.targetPWM) {
    ch.currentPWM -= MOTOR_RAMP_STEP_PWM;
    if (ch.currentPWM < ch.targetPWM) ch.currentPWM = ch.targetPWM;
  }
}

// Sets a channel's target speed/direction. If direction is reversing
// while the channel is still spinning, force PWM to 0 first before
// flipping IN1/IN2, to avoid a hard direction reversal through a live
// PWM value.
static void setChannel(MotorChannel &ch, int speedPercent, bool forward) {
  int targetPWM = percentToPWM(speedPercent);

  if (forward != ch.forward && ch.currentPWM > 0) {
    ch.currentPWM = 0;
    applyChannel(ch);
  }
  ch.forward = forward;
  ch.targetPWM = targetPWM;
}

// ---- Lifecycle ----

void motorsInit() {
  initChannel(_frontLeft);
  initChannel(_rearLeft);
  initChannel(_frontRight);
  initChannel(_rearRight);

  _frontLeft.currentPWM  = _frontLeft.targetPWM  = 0;
  _rearLeft.currentPWM   = _rearLeft.targetPWM   = 0;
  _frontRight.currentPWM = _frontRight.targetPWM = 0;
  _rearRight.currentPWM  = _rearRight.targetPWM  = 0;

  applyChannel(_frontLeft);
  applyChannel(_rearLeft);
  applyChannel(_frontRight);
  applyChannel(_rearRight);

  _lastRampMs = millis();

#if ENABLE_DEBUG_PRINTS
  bool anyConfigured = (_frontLeft.pwmPin >= 0) || (_rearLeft.pwmPin >= 0) ||
                        (_frontRight.pwmPin >= 0) || (_rearRight.pwmPin >= 0);
  if (anyConfigured) {
    Serial.println("[MOTOR] Pins configured for at least one channel.");
  } else {
    Serial.println("[MOTOR] WARNING: all motor pins are still -1 (TODO in "
                    "configuration.h). Running in no-op simulation mode.");
  }
#endif
}

void motorsUpdate() {
  unsigned long now = millis();
  if (now - _lastRampMs < MOTOR_RAMP_INTERVAL_MS) return;
  _lastRampMs = now;

  rampChannel(_frontLeft);
  rampChannel(_rearLeft);
  rampChannel(_frontRight);
  rampChannel(_rearRight);

  applyChannel(_frontLeft);
  applyChannel(_rearLeft);
  applyChannel(_frontRight);
  applyChannel(_rearRight);
}

// ---- Low-level per-side control ----

void setLeftSide(int speedPercent, bool forward) {
  setChannel(_frontLeft, speedPercent, forward);
  setChannel(_rearLeft, speedPercent, forward);
}

void setRightSide(int speedPercent, bool forward) {
  setChannel(_frontRight, speedPercent, forward);
  setChannel(_rearRight, speedPercent, forward);
}

// ---- High-level movement API ----

void moveForward(int speedPercent) {
#if ENABLE_DEBUG_PRINTS
  Serial.print("[MOTOR] moveForward speedPercent=");
  Serial.println(speedPercent);
#endif
  setLeftSide(speedPercent, true);
  setRightSide(speedPercent, true);
}

void moveBackward(int speedPercent) {
#if ENABLE_DEBUG_PRINTS
  Serial.print("[MOTOR] moveBackward speedPercent=");
  Serial.println(speedPercent);
#endif
  setLeftSide(speedPercent, false);
  setRightSide(speedPercent, false);
}

void turnLeft(int speedPercent) {
#if ENABLE_DEBUG_PRINTS
  Serial.print("[MOTOR] turnLeft speedPercent=");
  Serial.println(speedPercent);
#endif
  // In-place turn: left side backward, right side forward.
  setLeftSide(speedPercent, false);
  setRightSide(speedPercent, true);
}

void turnRight(int speedPercent) {
#if ENABLE_DEBUG_PRINTS
  Serial.print("[MOTOR] turnRight speedPercent=");
  Serial.println(speedPercent);
#endif
  // In-place turn: left side forward, right side backward.
  setLeftSide(speedPercent, true);
  setRightSide(speedPercent, false);
}

void stopRobot() {
#if ENABLE_DEBUG_PRINTS
  Serial.println("[MOTOR] stopRobot");
#endif
  _frontLeft.currentPWM  = _frontLeft.targetPWM  = 0;
  _rearLeft.currentPWM   = _rearLeft.targetPWM   = 0;
  _frontRight.currentPWM = _frontRight.targetPWM = 0;
  _rearRight.currentPWM  = _rearRight.targetPWM  = 0;

  applyChannel(_frontLeft);
  applyChannel(_rearLeft);
  applyChannel(_frontRight);
  applyChannel(_rearRight);
}

void emergencyStopMotors() {
#if ENABLE_DEBUG_PRINTS
  Serial.println("[MOTOR] EMERGENCY STOP");
#endif
  stopRobot();
}

bool motorsAreMoving() {
  return (_frontLeft.currentPWM > 0) || (_rearLeft.currentPWM > 0) ||
         (_frontRight.currentPWM > 0) || (_rearRight.currentPWM > 0);
}
