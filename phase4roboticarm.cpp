/*
 * arm.cpp
 * -------
 * Implements semantic arm control: moves all 3 joints (base, shoulder,
 * elbow) together toward a named preset, and tracks which preset the
 * arm is currently at/moving toward.
 *
 * Reused from the existing repo with stopArm() added (see arm.h).
 */

#include "arm.h"
#include "servos.h"
#include "configuration.h"

static ArmPosition currentPosition = ARM_POS_UNKNOWN;
static ArmPosition targetPosition  = ARM_POS_UNKNOWN; // what we're moving toward

void armInit() {
  // Deliberately do NOT command a move here - see gripperInit() for
  // the same reasoning. Arm stays at the safe servo midpoint until
  // explicitly commanded.
  currentPosition = ARM_POS_UNKNOWN;
  targetPosition  = ARM_POS_UNKNOWN;
  Serial.println("[arm] Initialized (position unknown until first command).");
}

// Internal helper: commands all 3 arm joints to a given angle set and
// marks the arm as moving toward the given named target.
static void commandArmPose(int baseAngle, int shoulderAngle, int elbowAngle, ArmPosition target) {
  setServoAngle(SERVO_IDX_ARM_BASE, baseAngle);
  setServoAngle(SERVO_IDX_ARM_SHOULDER, shoulderAngle);
  setServoAngle(SERVO_IDX_ARM_ELBOW, elbowAngle);
  targetPosition = target;
  currentPosition = ARM_POS_MOVING;
}

void moveArmHome() {
  Serial.println("[arm] Moving to HOME.");
  commandArmPose(ARM_HOME_BASE_ANGLE, ARM_HOME_SHOULDER_ANGLE, ARM_HOME_ELBOW_ANGLE, ARM_POS_HOME);
}

void moveArmToPickup() {
  Serial.println("[arm] Moving to PICKUP.");
  commandArmPose(ARM_PICKUP_BASE_ANGLE, ARM_PICKUP_SHOULDER_ANGLE, ARM_PICKUP_ELBOW_ANGLE, ARM_POS_PICKUP);
}

void moveArmToBin(PebbleSize size) {
  switch (size) {
    case PEBBLE_SIZE_SMALL:
      Serial.println("[arm] Moving to BIN_SMALL.");
      commandArmPose(ARM_BIN_SMALL_BASE_ANGLE, ARM_BIN_SMALL_SHOULDER_ANGLE, ARM_BIN_SMALL_ELBOW_ANGLE, ARM_POS_BIN_SMALL);
      break;
    case PEBBLE_SIZE_MEDIUM:
      Serial.println("[arm] Moving to BIN_MEDIUM.");
      commandArmPose(ARM_BIN_MEDIUM_BASE_ANGLE, ARM_BIN_MEDIUM_SHOULDER_ANGLE, ARM_BIN_MEDIUM_ELBOW_ANGLE, ARM_POS_BIN_MEDIUM);
      break;
    case PEBBLE_SIZE_LARGE:
      Serial.println("[arm] Moving to BIN_LARGE.");
      commandArmPose(ARM_BIN_LARGE_BASE_ANGLE, ARM_BIN_LARGE_SHOULDER_ANGLE, ARM_BIN_LARGE_ELBOW_ANGLE, ARM_POS_BIN_LARGE);
      break;
  }
}

void stopArm() {
  // Freeze each joint at wherever it currently is - do NOT command a
  // new angle (that would be a move, not a stop) and do NOT detach
  // (that could let an unbalanced arm drop under gravity).
  int baseAngle     = getServoAngle(SERVO_IDX_ARM_BASE);
  int shoulderAngle = getServoAngle(SERVO_IDX_ARM_SHOULDER);
  int elbowAngle    = getServoAngle(SERVO_IDX_ARM_ELBOW);

  setServoAngle(SERVO_IDX_ARM_BASE, baseAngle);
  setServoAngle(SERVO_IDX_ARM_SHOULDER, shoulderAngle);
  setServoAngle(SERVO_IDX_ARM_ELBOW, elbowAngle);

  // No longer guaranteed to be at any named preset - a stop mid-motion
  // means we don't actually know which pose (if any) this corresponds to.
  currentPosition = ARM_POS_UNKNOWN;
  targetPosition  = ARM_POS_UNKNOWN;

  Serial.println("[arm] STOPPED - halted mid-motion, position now UNKNOWN.");
}

ArmPosition getArmPosition() {
  return currentPosition;
}

bool isArmSettled() {
  return currentPosition != ARM_POS_MOVING && currentPosition != ARM_POS_UNKNOWN;
}

bool isArmAtHome() {
  return currentPosition == ARM_POS_HOME;
}

void armUpdate() {
  if (currentPosition != ARM_POS_MOVING) {
    return; // already settled, stopped, or unknown-with-no-command
  }

  bool allJointsSettled =
      isServoAtTarget(SERVO_IDX_ARM_BASE) &&
      isServoAtTarget(SERVO_IDX_ARM_SHOULDER) &&
      isServoAtTarget(SERVO_IDX_ARM_ELBOW);

  if (allJointsSettled) {
    currentPosition = targetPosition;
    Serial.print("[arm] Reached target position: ");
    Serial.println(currentPosition);
  }
}
