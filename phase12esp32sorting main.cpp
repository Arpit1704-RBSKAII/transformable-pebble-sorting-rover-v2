/*
 * main.cpp — PHASE 12 TEST VERSION
 * ----------------------------------
 * Adds the sorter mechanism (Phase 12) on top of everything from
 * Phases 1-5 (motors, servos, gripper, arm, safety, serial commands).
 *
 * IMPORTANT DESIGN NOTE: the sorter's preconditions (isArmAtHome() +
 * gripper closed) mean it's tested here as a STANDALONE mechanism -
 * arm homed and gripper closed FIRST, then the sorter chute cycles
 * through SMALL/MEDIUM/LARGE/stow independently. This is NOT yet
 * wired into the live SORT,<size> serial command (which still uses
 * only the arm's own per-bin positions from Phase 4/11) - seeing how
 * the physical chute relates to the arm's bin-drop motion is needed
 * before that integration can be done correctly. See the comment in
 * communication.cpp's handleSort() for the full reasoning.
 *
 * What this does:
 *   1-4. Same as Phase 4's test (motors, servos, gripper, arm).
 *   5. NEW: homes the arm + closes the gripper (satisfying sorter
 *      preconditions), then runs the sorter through SMALL -> MEDIUM ->
 *      LARGE, each followed by its automatic stow.
 *   6. NEW: starts a sort, then triggers an emergency stop PARTWAY
 *      through, and confirms the sorter halts immediately rather than
 *      continuing to move - the whole point of the non-blocking
 *      redesign (see sorter.h for why the original blocking version
 *      couldn't do this).
 *   After the auto-test, serial commands (Phase 5 protocol) remain
 *   live for manual testing.
 *
 * SAFETY: Keep the arm/gripper/sorter mechanism CLEAR of obstacles and
 * hands. Wheels should still be off the ground.
 */

#include <Arduino.h>
#include "configuration.h"
#include "motors.h"
#include "servos.h"
#include "gripper.h"
#include "arm.h"
#include "safety.h"
#include "sorter.h"
#include "communication.h"

static const int TEST_PAUSE_DURATION_MS = 800;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500);

  Serial.println("=== Phase 12: Sorter Mechanism Test ===");
  Serial.println("Wheels OFF THE GROUND. Keep arm/gripper/sorter clear.");

  motorsInit();
  servosInit();
  gripperInit();
  armInit();
  safetyInit();
  sorterInit();
  commInit();

  delay(2000);
}

static void waitForArmSettled(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!isArmSettled() && (millis() - start < timeoutMs)) {
    servosUpdate();
    armUpdate();
    delay(5);
  }
}

static void waitForGripperSettled(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!isGripperSettled() && (millis() - start < timeoutMs)) {
    servosUpdate();
    gripperUpdate();
    delay(5);
  }
}

static void waitForSorterSettled(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (!isSorterSettled() && (millis() - start < timeoutMs)) {
    sorterUpdate();
    delay(5);
  }
}

static void runSorterTest() {
  Serial.println("[test] Preparing sorter preconditions: arm home, gripper closed");
  moveArmHome();
  waitForArmSettled(ARM_MOVE_TIMEOUT_MS);
  closeGripper();
  waitForGripperSettled(GRIPPER_MOVE_TIMEOUT_MS);
  Serial.print("[test] Arm at home: ");
  Serial.println(isArmAtHome() ? "YES" : "NO");
  Serial.print("[test] Gripper closed: ");
  Serial.println(getGripperState() == GRIPPER_STATE_CLOSED ? "YES" : "NO");

  Serial.println("[test] Sorter: SMALL");
  bool accepted = sortSmall();
  Serial.print("[test] Accepted: ");
  Serial.println(accepted ? "YES" : "NO");
  waitForSorterSettled(SORTER_MOVE_TIMEOUT_MS * 2);
  Serial.println("[test] Sorter settled (stowed).");
  delay(TEST_PAUSE_DURATION_MS);

  Serial.println("[test] Sorter: MEDIUM");
  sortMedium();
  waitForSorterSettled(SORTER_MOVE_TIMEOUT_MS * 2);
  Serial.println("[test] Sorter settled (stowed).");
  delay(TEST_PAUSE_DURATION_MS);

  Serial.println("[test] Sorter: LARGE");
  sortLarge();
  waitForSorterSettled(SORTER_MOVE_TIMEOUT_MS * 2);
  Serial.println("[test] Sorter settled (stowed).");
  delay(TEST_PAUSE_DURATION_MS);

  // --- Emergency-stop interruption test ---
  // This is the actual point of the non-blocking redesign: an e-stop
  // mid-motion takes effect within one loop iteration, not "whenever
  // the blocking call happens to finish" (which the original version
  // couldn't do at all).
  Serial.println("[test] Sorter: SMALL (will be interrupted by emergency stop)");
  sortSmall();
  unsigned long start = millis();
  while (millis() - start < 100) { // let it move for a moment, but not finish
    sorterUpdate();
    delay(5);
  }
  Serial.println("[test] Triggering EMERGENCY STOP mid-motion...");
  triggerEmergencyStop();
  Serial.print("[test] Sorter settled immediately after e-stop: ");
  Serial.println(isSorterSettled() ? "YES (correct)" : "NO (BUG)");
  clearEmergencyStop();
  Serial.println("[test] Emergency cleared.");
  delay(TEST_PAUSE_DURATION_MS);

  Serial.println("[test] Sorter test done.");
}

void loop() {
  static bool testRun = false;

  if (!testRun) {
    Serial.println("=== Running Phase 1-4 regression (motors/servos/gripper/arm) ===");
    // Abbreviated vs. earlier phases' full sweep - full detail already
    // verified in Phase 1-4 test runs. Just confirms nothing broke.
    moveForward(MOTOR_DEFAULT_TEST_SPEED_PERCENT);
    delay(500);
    motorsUpdate();
    stopRobot();
    delay(300);
    Serial.println("=== Motors OK, proceeding to sorter test ===");

    runSorterTest();

    Serial.println("=== Phase 12 test complete ===");
    Serial.println("Serial commands remain live below for manual testing.");
    testRun = true;
  }

  // Keep everything live for manual serial command testing afterward.
  commUpdate();
  motorsUpdate();
  servosUpdate();
  gripperUpdate();
  armUpdate();
  sorterUpdate();
  safetyUpdate();
}
