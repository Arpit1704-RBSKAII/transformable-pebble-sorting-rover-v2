/*
 * communication.h
 * ---------------
 * Line-based ASCII command protocol over USB serial.
 *
 * This is the ONLY module that knows commands arrive over Serial -
 * parsing/dispatch logic below is transport-agnostic in structure,
 * so a future Wi-Fi/BLE transport can reuse dispatchCommand() and
 * just replace how raw lines are received.
 *
 * Commands (see project spec):
 *   MOVE,FORWARD,<speed>    MOVE,BACKWARD,<speed>
 *   MOVE,LEFT,<speed>       MOVE,RIGHT,<speed>
 *   STOP
 *   ARM,HOME    ARM,PICK    ARM,PLACE (rejected - use SORT,<size>)
 *   GRIP,OPEN   GRIP,CLOSE
 *   SORT,SMALL  SORT,MEDIUM  SORT,LARGE
 *   TRANSFORM,FOLD  TRANSFORM,UNFOLD (not implemented until Phase 13)
 *   STATUS
 *   EMERGENCY,STOP   EMERGENCY,CLEAR (CLEAR is a project addition, not
 *     in the original spec - see notes in communication.cpp)
 *
 * Responses:
 *   OK                  - command accepted, executing (or already done, for instant commands)
 *   BUSY                - a critical action (arm/gripper) is already in progress
 *   DONE,<what>         - a previously accepted async action has now completed
 *   ERROR,<description> - command rejected: malformed, unsafe, or not implemented
 *   READY               - sent once at boot, after all modules are initialized
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

// Call once in setup(), AFTER all other modules (motors/servos/gripper/
// arm/safety) have been initialized. Sends the "READY" line.
void commInit();

// Must be called frequently from loop(). Non-blocking:
//  - Accumulates incoming serial bytes into a line buffer.
//  - When a full line arrives, parses and dispatches it, sending an
//    immediate OK/BUSY/ERROR response.
//  - Also checks whether any previously accepted async action (arm/
//    gripper move) has now settled, and sends DONE,<what> if so.
void commUpdate();

#endif // COMMUNICATION_H
