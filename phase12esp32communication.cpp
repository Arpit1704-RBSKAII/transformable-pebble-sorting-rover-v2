/*
 * communication.cpp
 * ------------------
 * Implements the line-based serial command protocol declared in
 * communication.h. Reads bytes non-blocking, splits comma-separated
 * tokens, dispatches to motors/gripper/arm, and tracks a single
 * "pending async action" so DONE can be reported once it settles.
 *
 * Reused from the existing repo (already well-designed) with one fix:
 * EMERGENCY,STOP now calls triggerEmergencyStop() (safety.cpp), which
 * did not exist anywhere in the repo despite being referenced here.
 * triggerEmergencyStop() correctly halts the arm via stopArm() (not
 * just a generic servo freeze) - see safety.h for why that distinction
 * matters.
 */

#include "communication.h"
#include "configuration.h"
#include "motors.h"
#include "servos.h"
#include "gripper.h"
#include "arm.h"
#include "safety.h"

// ---- line buffering ----
static const int LINE_BUFFER_SIZE = 64;
static char lineBuffer[LINE_BUFFER_SIZE];
static int lineBufferIndex = 0;

// ---- pending async action tracking ----
// Only ONE critical async action (arm or gripper move) is allowed in
// flight at a time. A new critical command while one is pending gets
// rejected with BUSY rather than queued - keeps behavior predictable
// and matches "Python must wait for acknowledgements" from the spec.
enum PendingAction {
  PENDING_NONE,
  PENDING_ARM,
  PENDING_GRIPPER
};

static PendingAction pending = PENDING_NONE;
static String pendingLabel = ""; // human-readable label echoed back in DONE,<label>

static bool isCriticalActionPending() {
  return pending != PENDING_NONE;
}

// ---- helpers ----

// Splits line (already null-terminated, no trailing \r\n) into up to
// 3 comma-separated tokens. Returns the number of tokens found.
// Modifies a working copy so the original lineBuffer is untouched.
static int splitTokens(char* line, char* tokens[], int maxTokens) {
  int count = 0;
  char* token = strtok(line, ",");
  while (token != NULL && count < maxTokens) {
    tokens[count++] = token;
    token = strtok(NULL, ",");
  }
  return count;
}

static void beginArmAction(const char* label) {
  pending = PENDING_ARM;
  pendingLabel = label;
}

static void beginGripperAction(const char* label) {
  pending = PENDING_GRIPPER;
  pendingLabel = label;
}

// ---- command handlers ----

static void handleMove(char* tokens[], int tokenCount) {
  if (isEmergencyStopped()) {
    Serial.println("ERROR,EMERGENCY_STOP_ACTIVE");
    return;
  }
  if (tokenCount != 3) {
    Serial.println("ERROR,MALFORMED_MOVE_COMMAND");
    return;
  }

  const char* direction = tokens[1];
  int speed = atoi(tokens[2]);

  if (strcmp(direction, "FORWARD") == 0) {
    moveForward(speed);
  } else if (strcmp(direction, "BACKWARD") == 0) {
    moveBackward(speed);
  } else if (strcmp(direction, "LEFT") == 0) {
    turnLeft(speed);
  } else if (strcmp(direction, "RIGHT") == 0) {
    turnRight(speed);
  } else {
    Serial.println("ERROR,UNKNOWN_MOVE_DIRECTION");
    return;
  }

  // MOVE is continuous, not a settle-to-target action - OK is the
  // only response; there is no DONE for an ongoing drive command.
  Serial.println("OK");
}

static void handleStop() {
  stopRobot();
  Serial.println("OK");
}

static void handleArm(char* tokens[], int tokenCount) {
  if (isEmergencyStopped()) {
    Serial.println("ERROR,EMERGENCY_STOP_ACTIVE");
    return;
  }
  if (tokenCount != 2) {
    Serial.println("ERROR,MALFORMED_ARM_COMMAND");
    return;
  }
  if (isCriticalActionPending()) {
    Serial.println("BUSY");
    return;
  }

  const char* subcommand = tokens[1];

  if (strcmp(subcommand, "HOME") == 0) {
    moveArmHome();
    beginArmAction("ARM_HOME");
    Serial.println("OK");
  } else if (strcmp(subcommand, "PICK") == 0) {
    moveArmToPickup();
    beginArmAction("ARM_PICK");
    Serial.println("OK");
  } else if (strcmp(subcommand, "PLACE") == 0) {
    // Ambiguous without a bin size - the protocol's SORT,<size>
    // command is the unambiguous way to move the arm to a bin.
    Serial.println("ERROR,USE_SORT_COMMAND");
  } else {
    Serial.println("ERROR,UNKNOWN_ARM_SUBCOMMAND");
  }
}

static void handleGrip(char* tokens[], int tokenCount) {
  if (isEmergencyStopped()) {
    Serial.println("ERROR,EMERGENCY_STOP_ACTIVE");
    return;
  }
  if (tokenCount != 2) {
    Serial.println("ERROR,MALFORMED_GRIP_COMMAND");
    return;
  }
  if (isCriticalActionPending()) {
    Serial.println("BUSY");
    return;
  }

  const char* subcommand = tokens[1];

  if (strcmp(subcommand, "OPEN") == 0) {
    openGripper();
    beginGripperAction("GRIP_OPEN");
    Serial.println("OK");
  } else if (strcmp(subcommand, "CLOSE") == 0) {
    closeGripper();
    beginGripperAction("GRIP_CLOSE");
    Serial.println("OK");
  } else {
    Serial.println("ERROR,UNKNOWN_GRIP_SUBCOMMAND");
  }
}

static void handleSort(char* tokens[], int tokenCount) {
  if (isEmergencyStopped()) {
    Serial.println("ERROR,EMERGENCY_STOP_ACTIVE");
    return;
  }
  if (tokenCount != 2) {
    Serial.println("ERROR,MALFORMED_SORT_COMMAND");
    return;
  }
  if (isCriticalActionPending()) {
    Serial.println("BUSY");
    return;
  }

  const char* size = tokens[1];

  // NOTE: this still uses the arm's own per-bin positions (Phase 4/11),
  // NOT the Phase 12 sorter module. The sorter's sortSmall()/etc.
  // require isArmAtHome() as a precondition (matching the original
  // design's intent: chute realignment happens while the arm is idle
  // at home, not mid-motion toward a bin) - calling it here, right
  // after commanding the arm AWAY from home, would always fail that
  // check. Wiring the two mechanisms together correctly needs to know
  // the real physical relationship between the chute and the arm's
  // bin positions (does the chute pre-position BEFORE the arm moves,
  // or does the arm drop INTO a chute that's already aligned?) - see
  // sorter.h/sorter.cpp and the accompanying test for the standalone,
  // independently-testable version of this mechanism until that's
  // resolved.
  if (strcmp(size, "SMALL") == 0) {
    moveArmToBin(PEBBLE_SIZE_SMALL);
    beginArmAction("SORT_SMALL");
  } else if (strcmp(size, "MEDIUM") == 0) {
    moveArmToBin(PEBBLE_SIZE_MEDIUM);
    beginArmAction("SORT_MEDIUM");
  } else if (strcmp(size, "LARGE") == 0) {
    moveArmToBin(PEBBLE_SIZE_LARGE);
    beginArmAction("SORT_LARGE");
  } else {
    Serial.println("ERROR,UNKNOWN_SORT_SIZE");
    return;
  }

  Serial.println("OK");
}

static void handleTransform(char* tokens[], int tokenCount) {
  // Not implemented until Phase 13. Accepted here as a recognized
  // command (not UNKNOWN_COMMAND) so Python's protocol layer doesn't
  // need to change once Phase 13 lands.
  if (tokenCount != 2) {
    Serial.println("ERROR,MALFORMED_TRANSFORM_COMMAND");
    return;
  }
  Serial.println("ERROR,NOT_IMPLEMENTED");
}

static void handleStatus() {
  // Single-line machine-readable snapshot. Kept simple (no nested
  // structure) since this is ASCII serial, not JSON.
  Serial.print("STATUS,ARM=");
  Serial.print((int)getArmPosition());
  Serial.print(",GRIPPER=");
  Serial.print((int)getGripperState());
  Serial.print(",ESTOP=");
  Serial.print(isEmergencyStopped() ? 1 : 0);
  Serial.print(",PENDING=");
  Serial.println((int)pending);
}

static void handleEmergency(char* tokens[], int tokenCount) {
  if (tokenCount != 2) {
    Serial.println("ERROR,MALFORMED_EMERGENCY_COMMAND");
    return;
  }

  const char* subcommand = tokens[1];

  if (strcmp(subcommand, "STOP") == 0) {
    triggerEmergencyStop(); // halts motors + arm (correctly) + other servos
    pending = PENDING_NONE; // abandon any in-flight action tracking
    Serial.println("OK");
  } else if (strcmp(subcommand, "CLEAR") == 0) {
    // NOTE: EMERGENCY,CLEAR is an addition beyond the original spec's
    // EMERGENCY,STOP - added so the robot is recoverable over serial
    // without a physical reset. Remove this branch if you want e-stop
    // clearing to require physical intervention only.
    clearEmergencyStop();
    Serial.println("OK");
  } else {
    Serial.println("ERROR,UNKNOWN_EMERGENCY_SUBCOMMAND");
  }
}

// ---- top-level dispatch ----

static void dispatchCommand(char* line) {
  char* tokens[3];
  int tokenCount = splitTokens(line, tokens, 3);

  if (tokenCount == 0) {
    Serial.println("ERROR,EMPTY_COMMAND");
    return;
  }

  const char* command = tokens[0];

  if (strcmp(command, "MOVE") == 0) {
    handleMove(tokens, tokenCount);
  } else if (strcmp(command, "STOP") == 0) {
    handleStop();
  } else if (strcmp(command, "ARM") == 0) {
    handleArm(tokens, tokenCount);
  } else if (strcmp(command, "GRIP") == 0) {
    handleGrip(tokens, tokenCount);
  } else if (strcmp(command, "SORT") == 0) {
    handleSort(tokens, tokenCount);
  } else if (strcmp(command, "TRANSFORM") == 0) {
    handleTransform(tokens, tokenCount);
  } else if (strcmp(command, "STATUS") == 0) {
    handleStatus();
  } else if (strcmp(command, "EMERGENCY") == 0) {
    handleEmergency(tokens, tokenCount);
  } else {
    Serial.println("ERROR,UNKNOWN_COMMAND");
  }
}

// ---- public API ----

void commInit() {
  lineBufferIndex = 0;
  pending = PENDING_NONE;
  Serial.println("READY");
}

void commUpdate() {
  // --- 1. Non-blocking read of incoming serial bytes into lineBuffer ---
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      lineBuffer[lineBufferIndex] = '\0';

      // Strip a trailing \r if present (Windows-style line endings)
      if (lineBufferIndex > 0 && lineBuffer[lineBufferIndex - 1] == '\r') {
        lineBuffer[lineBufferIndex - 1] = '\0';
      }

      if (lineBufferIndex > 0) {
        dispatchCommand(lineBuffer);
      }
      lineBufferIndex = 0;
    } else if (lineBufferIndex < LINE_BUFFER_SIZE - 1) {
      lineBuffer[lineBufferIndex++] = c;
    }
    // Bytes beyond LINE_BUFFER_SIZE-1 are silently dropped until the
    // next '\n' - prevents a malformed/oversized line from corrupting
    // memory. A well-behaved Python client should never hit this.
  }

  // --- 2. Check whether a pending async action has now settled ---
  if (pending == PENDING_ARM && isArmSettled()) {
    Serial.print("DONE,");
    Serial.println(pendingLabel);
    pending = PENDING_NONE;
  } else if (pending == PENDING_GRIPPER && isGripperSettled()) {
    Serial.print("DONE,");
    Serial.println(pendingLabel);
    pending = PENDING_NONE;
  }
}
