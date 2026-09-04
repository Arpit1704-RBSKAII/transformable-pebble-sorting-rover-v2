// esp32/include/sorter.h
//
// Independent sorting mechanism (a chute/diverter servo, separate from
// the arm), per project spec section 11: "The sorting mechanism should
// be independent from the robotic arm."
//
// REDESIGNED from the repo's original version, which was entirely
// BLOCKING (its _moveToAngle() ran a while-loop calling delay()
// directly). That meant an EMERGENCY,STOP arriving over serial - or
// even a hardware E-STOP button press - during a sort motion would not
// be processed until the motion finished on its own (commUpdate()
// never got to run), and the original code's own in-loop "emergency
// abort" check could never actually fire, since nothing could set that
// flag while the main thread was stuck inside the blocking loop.
//
// This version is non-blocking, matching every other actuator module
// in this codebase (motors/servos/arm/gripper): sortSmall() etc. just
// validate preconditions and set a target, returning immediately;
// sorterUpdate() (called every loop() iteration) advances the servo
// angle a few degrees at a time. This also means an emergency stop
// takes effect within one loop iteration, not "whenever the current
// blocking call happens to finish."

#ifndef SORTER_H
#define SORTER_H

#include <Arduino.h>

enum SorterState {
  SORTER_STATE_STOWED,
  SORTER_STATE_MOVING_TO_BIN,
  SORTER_STATE_AT_BIN,
  SORTER_STATE_STOWING,
};

// Call once in setup(), AFTER armInit()/gripperInit() (preconditions
// check their state).
void sorterInit();

// Request a sort to a size. Validates preconditions (arm home, gripper
// closed - see sorter.cpp) and starts a non-blocking move if they
// pass. Returns true if the move was ACCEPTED and started (not
// completed - check isSorterBusy()/isSorterSettled() for that);
// false if a precondition failed or the ESP32 is already mid-sort.
bool sortSmall();
bool sortMedium();
bool sortLarge();

// Returns the sorter to its neutral/stowed position directly, bypassing
// the automatic post-sort stow (useful for manual testing/reset).
bool sortReset();

// Must be called frequently from loop() - advances the servo ramp and
// the automatic move-to-bin -> pause -> stow sequence.
void sorterUpdate();

// True while any motion (including the automatic post-drop stow) is
// still in progress.
bool isSorterBusy();

// True once fully stowed and idle - the real "sort completed" signal.
bool isSorterSettled();

// Immediately halts the servo at its current angle and marks the
// sorter idle. Used by safety.cpp's triggerEmergencyStop().
void sorterEmergencyStop();

#endif // SORTER_H
