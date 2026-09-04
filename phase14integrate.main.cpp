/*
 * main.cpp — PHASE 14: COMPLETE INTEGRATION (production firmware)
 * -------------------------------------------------------------------
 * This is the FINAL, permanent main.cpp - not a "test version" to be
 * replaced by the next phase. Every module from Phases 1-13 is wired
 * together here: motors, servos, gripper, arm, safety (software +
 * hardware e-stop), sorter, transformation, and the serial command
 * protocol.
 *
 * Unlike every previous phase's main.cpp, this one does NOT run a
 * built-in demo/test sequence on boot. Phases 1-13 each had a canned
 * sequence (drive forward, sweep servos, etc.) so you could verify
 * each new module in isolation via the Serial Monitor. Now that
 * python/decision.py is the real, continuous controller (Phase 11),
 * the ESP32's only job is to boot, initialize every subsystem, print
 * READY, and then just respond to whatever commands Python sends -
 * exactly like Phase 5 already worked, just with every later
 * subsystem (safety, sorter, transformation) now wired in too.
 *
 * If you want to re-run any of the old demo sequences for hardware
 * debugging, they're preserved in each phase's own main.cpp test file
 * (not deleted, just not part of this final firmware).
 *
 * NOTE on the repo's own "Phase 14 complete integration" draft: it was
 * NOT used as the basis for this file. It was written against a
 * different, class-based API (Motors motors; Arm arm; ...) that
 * doesn't match any of the actual free-function modules built across
 * Phases 1-13, used BLOCKING delay()-based motor moves (the same bug
 * fixed in Phase 12/13, but for drive motors), had no integration with
 * safety.h at all, and even reintroduced the SERIAL_BAUD vs
 * SERIAL_BAUD_RATE bug from Phase 1. Adopting it would have meant
 * reintroducing already-fixed bugs, so this file instead finalizes the
 * main.cpp that's already been built up correctly, phase by phase,
 * since Phase 5.
 */

#include <Arduino.h>
#include "configuration.h"
#include "motors.h"
#include "servos.h"
#include "gripper.h"
#include "arm.h"
#include "safety.h"
#include "sorter.h"
#include "transformation.h"
#include "communication.h"

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500); // allow serial monitor / Python to connect

#if ENABLE_DEBUG_PRINTS
  Serial.println("[MAIN] Initializing all subsystems...");
#endif

  motorsInit();
  servosInit();
  gripperInit();
  armInit();
  safetyInit();
  sorterInit();
  transformInit();

  // commInit() prints "READY" - must be LAST so Python knows every
  // other subsystem is already initialized before it starts sending
  // commands.
  commInit();
}

void loop() {
  // Read/dispatch any incoming serial command, and report DONE for
  // whichever async action (if any) has settled since the last tick.
  commUpdate();

  // Advance every subsystem's non-blocking state. Order doesn't
  // matter for correctness (each module only touches its own state),
  // but motors first / safety last is a reasonable convention: drive
  // state settles before actuators, and the hardware e-stop check runs
  // last so it sees the most current state of everything else this tick.
  motorsUpdate();
  servosUpdate();
  gripperUpdate();
  armUpdate();
  sorterUpdate();
  transformUpdate();
  safetyUpdate();
}
