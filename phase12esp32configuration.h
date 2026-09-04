// esp32/include/configuration.h
//
// Centralized hardware configuration for the ESP32 side of the rover.
//
// PHASE 1 SCOPE: only motor-related values are defined here for now.
// Later phases (servo, sorter, transformation, sensors, communication)
// will ADD their own sections to this same file.

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// ============================================================
// SERIAL
// ============================================================
#define SERIAL_BAUD_RATE 115200

// ============================================================
// MOTOR DRIVER PINS — TODO: CONFIGURE AFTER HARDWARE SELECTION
// ============================================================
// Confirmed hardware: 4 DC geared motors (one per wheel), driven by
// TWO L298N dual H-bridge modules (2 motors per module = 4 channels
// total). Software still commands them in PAIRED left/right groups —
// front-left and rear-left always move identically, same for the
// right pair — but each physical motor has its OWN pwm+direction
// pins, matched to how L298N actually works (2 direction pins per
// channel: IN1/IN2, plus one PWM enable pin: ENA/ENB).
//
// L298N direction truth table used below:
//   IN1=HIGH, IN2=LOW  -> spin forward
//   IN1=LOW,  IN2=HIGH -> spin backward
//   (PWM pin at 0 duty stops the motor regardless of IN1/IN2 state)
//
// GPIO NOTES for when you assign real pins:
//   - Avoid ESP32 input-only pins (GPIO 34-39) — they cannot drive
//     outputs, so they cannot be used for PWM or IN1/IN2.
//   - Avoid strapping pins (GPIO 0, 2, 12, 15) if possible — they
//     affect boot mode and can cause upload/boot issues if a motor
//     driver is holding them high/low at power-on.

// --- Front Left motor (Driver #1, Channel A) ---
#define MOTOR_FRONT_LEFT_PWM_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> ENA
#define MOTOR_FRONT_LEFT_IN1_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN1
#define MOTOR_FRONT_LEFT_IN2_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN2

// --- Rear Left motor (Driver #1, Channel B) ---
#define MOTOR_REAR_LEFT_PWM_PIN   -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> ENB
#define MOTOR_REAR_LEFT_IN1_PIN   -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN3
#define MOTOR_REAR_LEFT_IN2_PIN   -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN4

// --- Front Right motor (Driver #2, Channel A) ---
#define MOTOR_FRONT_RIGHT_PWM_PIN -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> ENA
#define MOTOR_FRONT_RIGHT_IN1_PIN -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN1
#define MOTOR_FRONT_RIGHT_IN2_PIN -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN2

// --- Rear Right motor (Driver #2, Channel B) ---
#define MOTOR_REAR_RIGHT_PWM_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> ENB
#define MOTOR_REAR_RIGHT_IN1_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN3
#define MOTOR_REAR_RIGHT_IN2_PIN  -1   // TODO: CONFIGURE AFTER HARDWARE SELECTION -> IN4

// ============================================================
// PWM / SPEED CONFIGURATION
// ============================================================
// Public API speed is a PERCENT (0-100), converted internally to a
// raw 0-255 PWM duty cycle. Do not pass raw PWM values into
// moveForward/moveBackward/turnLeft/turnRight.

#define MOTOR_PWM_MAX             255  // 8-bit PWM ceiling
#define MOTOR_MAX_SPEED_PERCENT   100  // Logical speed ceiling (0-100 scale)
#define MOTOR_DEFAULT_TEST_SPEED_PERCENT 30 // Conservative bench-test default

// Speed ramping — motorsUpdate() must be called frequently (every loop
// iteration) for this to take effect.
#define MOTOR_RAMP_STEP_PWM     10   // PWM units changed per ramp step
#define MOTOR_RAMP_INTERVAL_MS  20   // Minimum ms between ramp steps

// ============================================================
// SERVOS — TODO: CONFIGURE AFTER HARDWARE SELECTION
// ============================================================
// This section was MISSING from the previous repo version, even though
// servos.cpp/servos.h already reference every constant below. Without
// this section, Phase 2 would not have compiled. Servo count/models
// still need to be confirmed (arm has 3 servos per the original spec:
// base, shoulder, elbow — the earlier repo's single ARM_SERVO_PIN
// suggests a simplified 1-servo arm was tried at some point; using the
// full 3-servo arm here since that's what servos.cpp already assumes
// via SERVO_COUNT / the SERVO_IDX_* table below. Tell me if the arm is
// actually meant to be single-servo and this gets simplified instead).

// Index table — ORDER MUST MATCH the pin array in servos.cpp.
#define SERVO_IDX_ARM_BASE     0
#define SERVO_IDX_ARM_SHOULDER 1
#define SERVO_IDX_ARM_ELBOW    2
#define SERVO_IDX_GRIPPER      3
#define SERVO_IDX_TRANSFORM    4
#define SERVO_COUNT            5

// Pins — TODO: CONFIGURE AFTER HARDWARE SELECTION
#define SERVO_ARM_BASE_PIN     -1
#define SERVO_ARM_SHOULDER_PIN -1
#define SERVO_ARM_ELBOW_PIN    -1
#define SERVO_GRIPPER_PIN      -1
#define SERVO_TRANSFORM_PIN    -1

// Safe default angle range applied to EVERY servo until per-servo
// mechanical limits are measured on the physical build (Phase 3/4).
// TODO: CALIBRATE ON HARDWARE
#define SERVO_MIN_ANGLE_DEFAULT 10
#define SERVO_MAX_ANGLE_DEFAULT 170

// Ramping: servosUpdate() must be called frequently (every loop
// iteration) for this to take effect.
#define SERVO_TICK_MS            20  // Minimum ms between ramp steps
#define SERVO_MAX_STEP_PER_TICK   2  // Degrees moved per tick (slow = gentle)

// ============================================================
// GRIPPER — TODO: CALIBRATE ON HARDWARE
// ============================================================
// These were referenced by the Phase 3 test main.cpp but never actually
// defined anywhere in the repo, and gripper.cpp/gripper.h did not exist
// at all — the two files labeled "GRIPPER" were duplicates of the
// generic servos.cpp/servos.h, not real gripper logic. Both gaps are
// filled now.
#define GRIPPER_OPEN_ANGLE      40    // TODO: CALIBRATE ON HARDWARE
#define GRIPPER_CLOSE_ANGLE     90    // TODO: CALIBRATE ON HARDWARE
#define GRIPPER_MOVE_TIMEOUT_MS 3000  // Max time to wait for gripper to settle

// ============================================================
// ARM — TODO: CALIBRATE ON HARDWARE
// ============================================================
// arm.cpp already existed and is well-designed (named-preset position
// tracking, isArmAtHome() for future use by transformation.cpp), but
// referenced all of the angle constants below, none of which were
// actually defined anywhere in the repo.
//
// Values here are PLACEHOLDER angles only, spaced out for visibly
// distinct test motion — they do NOT reflect your actual arm geometry.
// Recalibrate every one of these once the arm is physically mounted.

#define ARM_MOVE_TIMEOUT_MS 4000

// HOME - safe resting/travel position
#define ARM_HOME_BASE_ANGLE       90   // TODO: CALIBRATE ON HARDWARE
#define ARM_HOME_SHOULDER_ANGLE   90   // TODO: CALIBRATE ON HARDWARE
#define ARM_HOME_ELBOW_ANGLE      90   // TODO: CALIBRATE ON HARDWARE

// PICKUP - reaches down/out toward a detected pebble
#define ARM_PICKUP_BASE_ANGLE     90   // TODO: CALIBRATE ON HARDWARE
#define ARM_PICKUP_SHOULDER_ANGLE 60   // TODO: CALIBRATE ON HARDWARE
#define ARM_PICKUP_ELBOW_ANGLE    120  // TODO: CALIBRATE ON HARDWARE

// BIN_SMALL / BIN_MEDIUM / BIN_LARGE - one pose per sorting bin
#define ARM_BIN_SMALL_BASE_ANGLE      45   // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_SMALL_SHOULDER_ANGLE  70   // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_SMALL_ELBOW_ANGLE     100  // TODO: CALIBRATE ON HARDWARE

#define ARM_BIN_MEDIUM_BASE_ANGLE     90   // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_MEDIUM_SHOULDER_ANGLE 70   // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_MEDIUM_ELBOW_ANGLE    100  // TODO: CALIBRATE ON HARDWARE

#define ARM_BIN_LARGE_BASE_ANGLE      135  // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_LARGE_SHOULDER_ANGLE  70   // TODO: CALIBRATE ON HARDWARE
#define ARM_BIN_LARGE_ELBOW_ANGLE     100  // TODO: CALIBRATE ON HARDWARE

// ============================================================
// SORTER — TODO: CALIBRATE ON HARDWARE
// ============================================================
// Uses its OWN dedicated servo (not one of the 5 slots in the
// SERVO_IDX_* array above) since the sorting mechanism is explicitly
// meant to be independent from the arm/gripper/transform per project
// spec section 11. Merged from the repo's existing sorter config file,
// which was a fully separate/standalone configuration.h (its own
// #define SERIAL_BAUD, not merged with the rest of this file) - only
// the sorter-specific values are pulled in here.

#define SORTER_SERVO_PIN -1  // TODO: CONFIGURE AFTER HARDWARE SELECTION

// TODO: CALIBRATE ON HARDWARE - tune so each angle reliably drops into
// the correct bin. Do NOT set angles that stress the mechanism/servo.
#define SORTER_ANGLE_STOW   90   // neutral/stowed angle
#define SORTER_ANGLE_SMALL  30   // angle that directs a pebble to the SMALL bin
#define SORTER_ANGLE_MEDIUM 90   // angle that directs a pebble to the MEDIUM bin
#define SORTER_ANGLE_LARGE  150  // angle that directs a pebble to the LARGE bin

#define SORTER_MOVE_STEP_DEG   2     // degrees per ramp step (smaller = smoother)
#define SORTER_STEP_DELAY_MS   20    // ms between ramp steps
#define SORTER_MOVE_TIMEOUT_MS 5000  // max time to complete a move before aborting

// ============================================================
// SAFETY SENSORS — all optional, TODO: CONFIGURE IF WIRED
// ============================================================
// Everything in this section defaults to -1 (not configured / not
// wired) and is treated as a no-op until you actually wire something
// up. The sorter's PRIMARY preconditions use the software state we
// already track (isArmAtHome(), getGripperState()) from Phases 3-4 -
// these physical switches, if wired, are an OPTIONAL additional layer,
// not a replacement for that software state.
#define PIN_ARM_HOME_SWITCH        -1  // TODO: CONFIGURE IF WIRED - HIGH when arm is physically at home
#define PIN_GRIPPER_CLOSED_SWITCH  -1  // TODO: CONFIGURE IF WIRED - HIGH when gripper is physically closed
#define PIN_WHEELS_STOPPED_SIGNAL  -1  // TODO: CONFIGURE IF WIRED - HIGH when drive wheels are stopped

// Physical emergency-stop button, polled every loop() tick by
// safety.cpp (not just reacted to over serial) - per project spec
// section 15: "the software must not rely solely on Python to stop
// the robot." Defaults to -1 (no physical button) until wired.
#define PIN_EMERGENCY_STOP -1  // TODO: CONFIGURE IF WIRED - LOW when pressed (assumes a normally-closed-to-HIGH button wired with INPUT_PULLUP)

// ============================================================
// LOGGING
// ============================================================
#define ENABLE_DEBUG_PRINTS 1

#endif // CONFIGURATION_H
