/*
 * main.cpp — PHASE 5 TEST VERSION
 * --------------------------------
 * Wires up ALL modules (motors, servos, gripper, arm, safety) behind
 * the serial command protocol from communication.cpp. Unlike Phases
 * 1-4, this is NOT an auto-run sequence - you drive it by typing
 * commands into the Serial Monitor yourself, since that's what's
 * actually being tested here: the protocol, not the motion.
 *
 * This TEMPORARILY REPLACES the Phase 4 main.cpp. Everything merges
 * into one final main.cpp at Phase 14.
 *
 * HOW TO TEST:
 *   1. Upload this, then open Serial Monitor.
 *   2. Set line ending to "Newline" (not "No line ending") so your
 *      typed commands actually terminate with \n.
 *   3. You should see "READY" printed once at boot.
 *   4. Type commands (see list below) and press Enter. You should
 *      see an immediate OK/BUSY/ERROR response, and for ARM/GRIP/SORT
 *      commands, a DONE,<label> line once the motion completes.
 *
 * SAFETY: Keep the arm/gripper/transform mechanism CLEAR and wheels
 * OFF THE GROUND, since MOVE commands will now actually drive the
 * wheels continuously until you send STOP.
 *
 * Suggested test sequence (type these one at a time):
 *   STATUS                    -> STATUS,ARM=...,GRIPPER=...,ESTOP=0,PENDING=0
 *   MOVE,FORWARD,30            -> OK   (wheels should turn - watch for it, then...)
 *   STOP                       -> OK
 *   GRIP,OPEN                  -> OK, then DONE,GRIP_OPEN shortly after
 *   ARM,HOME                   -> OK, then DONE,ARM_HOME shortly after
 *   ARM,HOME  (send again immediately, before DONE arrives)
 *                               -> BUSY, if the first hasn't settled yet
 *   SORT,SMALL                 -> OK, then DONE,SORT_SMALL
 *   TRANSFORM,FOLD              -> ERROR,NOT_IMPLEMENTED (expected - Phase 13)
 *   EMERGENCY,STOP              -> OK
 *   MOVE,FORWARD,30             -> ERROR,EMERGENCY_STOP_ACTIVE (expected)
 *   EMERGENCY,CLEAR              -> OK
 *   MOVE,FORWARD,30             -> OK again (recovered)
 *   BADCOMMAND                  -> ERROR,UNKNOWN_COMMAND
 */

#include <Arduino.h>
#include "configuration.h"
#include "motors.h"
#include "servos.h"
#include "gripper.h"
#include "arm.h"
#include "safety.h"
#include "communication.h"

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(500); // allow serial monitor to connect

  motorsInit();
  servosInit();
  gripperInit();
  armInit();
  safetyInit();

  // commInit() prints "READY" - must be LAST so Python (later phases)
  // knows every other module is already initialized before it starts
  // sending commands.
  commInit();
}

void loop() {
  // All actual work happens inside these Update functions - reading
  // serial, dispatching commands, advancing servo ramps, and tracking
  // whether the arm/gripper have settled.
  commUpdate();
  servosUpdate();
  gripperUpdate();
  armUpdate();
  motorsUpdate();
}
