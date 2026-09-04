"""
High-level decision and state machine.

Usage:
    - Create DecisionMachine(comm) where comm is an instance of BaseComm
      (e.g., MockComm for tests or SerialComm for hardware - both in
      communication.py).
    - Call dm.start_search() to transition to SEARCHING.
    - Feed detections by calling dm.process_detection(detection, image).
    - Call dm.emergency_stop() to enter EMERGENCY_STOP state.

Design:
    - The machine is intentionally single-threaded for clarity.
      Integrate into your main loop.
    - Communication with hardware is done through comm.send_command()
      and comm.wait_for().

Reused from the existing repo (already well-structured - correct state
list, correct safety gating on emergency/arm-extended) with THREE bugs
fixed. All three were invisible against the original (also buggy)
MockComm, which is why they weren't caught before - see communication.py
and the fix notes below for how the two bugs interact.

BUG 1 - premature completion on real hardware:
    _send_and_wait() used to treat an immediate "OK" response as
    COMPLETION for critical actions (ARM/GRIP/SORT). But in the real
    ESP32 protocol (Phase 5), OK means "accepted, now executing" - the
    real completion signal is a separate DONE,<label> line sent later.
    Fixed: OK is only treated as terminal for non-critical commands
    (MOVE/STOP/EMERGENCY/STATUS, which the ESP32 never sends a DONE
    for). Critical commands always wait for an actual DONE.

BUG 2 - BUSY treated as "keep waiting" instead of "rejected":
    The original code fell through to wait_for() even after a BUSY
    response. But BUSY means the ESP32 REJECTED this command outright
    because a different critical action is still pending - there is no
    DONE coming for THIS command. Fixed: BUSY now raises immediately.

BUG 3 - protocol mismatches (commands that don't exist in Phase 5):
    "ARM,TO_PICKUP" -> fixed to "ARM,PICK" (the ESP32 only recognizes
    ARM,HOME and ARM,PICK). "MOVE,APPROACH,x,y" was sent to drive
    toward the pebble before pickup, but no such command exists in the
    Phase 5 protocol, and building a real image-to-robot coordinate
    transform requires physical calibration data this project doesn't
    have yet (would violate "do not invent hardware"). Fixed: this step
    is skipped with a clear TODO, rather than sending a command
    guaranteed to fail with ERROR,UNKNOWN_MOVE_DIRECTION on real
    hardware every single time.
"""

import time
from typing import Dict, Optional

from communication import BaseComm, MockComm
from classification import classify_size
from color_classification import classify_colour
import configuration as cfg


class DecisionError(Exception):
    pass


class EmergencyException(Exception):
    pass


class DecisionMachine:
    # States
    STATE_IDLE = "IDLE"
    STATE_SEARCHING = "SEARCHING"
    STATE_PEBBLE_DETECTED = "PEBBLE_DETECTED"
    STATE_POSITION_ESTIMATED = "POSITION_ESTIMATED"
    STATE_MOVING_ARM = "MOVING_ARM"
    STATE_GRIPPING = "GRIPPING"
    STATE_PICKED = "PICKED"
    STATE_CLASSIFYING = "CLASSIFYING"
    STATE_MOVING_TO_BIN = "MOVING_TO_BIN"
    STATE_RELEASING = "RELEASING"
    STATE_RETURNING_HOME = "RETURNING_HOME"
    STATE_READY = "READY"
    STATE_FOLDING = "FOLDING"
    STATE_ROVER_MODE = "ROVER_MODE"
    STATE_UNFOLDING = "UNFOLDING"
    STATE_ERROR = "ERROR"
    STATE_EMERGENCY = "EMERGENCY_STOP"

    # Command prefixes that trigger an async DONE,<label> from the
    # ESP32 (Phase 5 protocol) rather than being complete on OK alone.
    _CRITICAL_PREFIXES = ("ARM,", "GRIP,", "SORT,", "TRANSFORM,")

    def __init__(self, comm: Optional[BaseComm] = None):
        self.state = self.STATE_IDLE
        self.comm = comm if comm is not None else MockComm()
        self.emergency = False
        self.arm_extended = False
        self.transform_folded = False
        self.last_detection = None
        self.pick_retries = 0
        self.log_level = getattr(cfg, "LOG_VERBOSITY", 1)

    def log(self, *args, level=1):
        if self.log_level >= level:
            print("[DM]", *args)

    def start_search(self):
        if self.emergency:
            self.log("Cannot start search: EMERGENCY active", level=0)
            return
        if self.state not in (self.STATE_IDLE, self.STATE_READY):
            self.log("start_search called but not in IDLE/READY", level=1)
        self.state = self.STATE_SEARCHING
        self.log("State -> SEARCHING")

    def emergency_stop(self):
        self.log("Emergency stop requested", level=0)
        self.emergency = True
        self.state = self.STATE_EMERGENCY
        try:
            self.comm.send_command("EMERGENCY,STOP")
        except Exception as e:
            self.log("Failed to send EMERGENCY to hardware:", e, level=0)

    def clear_emergency(self):
        self.log("Clearing emergency", level=0)
        self.emergency = False
        self.state = self.STATE_IDLE

    def _is_critical_command(self, cmd: str) -> bool:
        return cmd.startswith(self._CRITICAL_PREFIXES)

    def _send_and_wait(self, cmd: str, timeout: float = cfg.COMM_COMMAND_TIMEOUT):
        """
        Send a command and, for critical actions, wait for the actual
        DONE completion signal. Non-critical commands (MOVE/STOP/
        EMERGENCY) complete as soon as OK arrives, since the ESP32
        never sends a later DONE for those.

        Raises DecisionError on ERROR, on BUSY (command was rejected,
        not queued), or on timeout waiting for DONE.
        """
        if self.emergency:
            raise EmergencyException("Cannot send commands while in EMERGENCY state")

        self.log("Sending command:", cmd)
        resp = self.comm.send_command(cmd)
        self.log("Immediate response:", resp, level=2)

        if resp.startswith("ERROR"):
            raise DecisionError(f"Hardware error on command {cmd}: {resp}")

        if resp == "BUSY":
            # BUSY means REJECTED (a different critical action is still
            # pending) - there is no DONE coming for THIS command.
            raise DecisionError(f"Hardware busy, command rejected: {cmd}")

        if resp != "OK":
            raise DecisionError(f"Unexpected immediate response to {cmd}: {resp}")

        if not self._is_critical_command(cmd):
            # MOVE/STOP/EMERGENCY/STATUS are complete as soon as OK
            # arrives - no DONE follows for these.
            return resp

        # Critical action: OK means "accepted, now executing" - wait
        # for the real completion signal.
        try:
            token = self.comm.wait_for(["DONE"], timeout=timeout)
            if token.startswith("ERROR"):
                raise DecisionError(f"Hardware error while waiting for {cmd}: {token}")
            return token
        except TimeoutError as e:
            raise DecisionError(f"Timeout waiting for hardware for command {cmd}") from e

    def process_detection(self, detection: Dict, image=None):
        """
        Main entrypoint: called when the detection pipeline finds an
        object. detection should be a dict containing at least
        'detected' and 'bbox' (matches detection.PebbleDetection.to_dict()).
        """
        if self.emergency:
            self.log("Ignoring detection due to EMERGENCY", level=0)
            return

        self.last_detection = detection

        if self.state not in (self.STATE_SEARCHING, self.STATE_IDLE, self.STATE_READY):
            self.log("Received detection but state is", self.state, "\u2014 ignoring", level=2)
            return

        self.state = self.STATE_PEBBLE_DETECTED
        self.log("State -> PEBBLE_DETECTED")

        bbox = detection.get("bbox")
        if not bbox:
            self.log("No bbox in detection, cannot estimate position", level=0)
            self.state = self.STATE_ERROR
            return

        x, y, w, h = bbox
        cx, cy = int(x + w / 2), int(y + h / 2)
        detection["center"] = (cx, cy)
        self.state = self.STATE_POSITION_ESTIMATED
        self.log(f"State -> POSITION_ESTIMATED at center {cx},{cy}")

        size_result = classify_size(detection)
        color_result = classify_colour(detection, image=image)
        self.log("Size result:", size_result, level=2)
        self.log("Colour result:", color_result, level=2)

        size_conf = size_result.get("confidence", 0.0) or 0.0

        if color_result.get("detected", False):
            color_conf = color_result.get("confidence", 0.0) or 0.0
            combined_conf = (cfg.SIZE_CONFIDENCE_WEIGHT * size_conf) + (cfg.COLOR_CONFIDENCE_WEIGHT * color_conf)
        else:
            # Colour is OPTIONAL per project spec, and genuinely
            # couldn't be evaluated here (no image/roi supplied, or the
            # mask was too small). Falling back to size confidence
            # alone rather than letting an unevaluated colour reading
            # drag combined confidence toward zero for no real reason.
            color_conf = 0.0
            combined_conf = size_conf

        detection["size_result"] = size_result
        detection["color_result"] = color_result
        detection["combined_confidence"] = combined_conf
        self.log(f"Combined confidence: {combined_conf:.3f}")

        if combined_conf < cfg.PICK_CONFIDENCE_THRESHOLD:
            self.log("Combined confidence too low \u2014 skipping object", level=1)
            self.state = self.STATE_SEARCHING
            return

        if not cfg.ALLOW_PICK_WHILE_MOVING and self.arm_extended:
            self.log("Arm is extended; refusing to drive", level=0)
            self.state = self.STATE_ERROR
            return

        # NOTE: no approach/centering drive command is sent here. The
        # original code sent "MOVE,APPROACH,x,y", but that command does
        # not exist in the Phase 5 ESP32 protocol (only MOVE,FORWARD/
        # BACKWARD/LEFT/RIGHT), and building a real image-to-robot
        # coordinate transform needs physical calibration data this
        # project doesn't have yet. Assumes the pebble is already
        # within the arm's fixed pickup zone (ARM_PICKUP_* angles in
        # esp32/include/configuration.h). Revisit once a real transform
        # is calibrated.
        self.log("Skipping approach-drive step (no coordinate transform calibrated yet)", level=1)

        try:
            self.state = self.STATE_MOVING_ARM
            self.log("State -> MOVING_ARM")
            self._send_and_wait("ARM,PICK", timeout=cfg.ARM_MOVE_TIMEOUT)
            self.arm_extended = True
        except DecisionError as e:
            self.log("Arm move failed:", e, level=0)
            self.state = self.STATE_ERROR
            return

        try:
            self.state = self.STATE_GRIPPING
            self.log("State -> GRIPPING")
            self._send_and_wait("GRIP,OPEN", timeout=3.0)
            self._send_and_wait("GRIP,CLOSE", timeout=cfg.PICK_TIMEOUT)
            self.state = self.STATE_PICKED
            self.log("State -> PICKED")
        except DecisionError as e:
            self.log("Gripping failed:", e, level=0)
            self.pick_retries += 1
            if self.pick_retries <= cfg.MAX_PICK_RETRIES:
                self.log("Retrying pick", level=1)
                self.state = self.STATE_SEARCHING
                return
            else:
                self.state = self.STATE_ERROR
                return

        self.state = self.STATE_CLASSIFYING
        self.log("State -> CLASSIFYING")
        size_label = size_result.get("size_label", "UNKNOWN")

        try:
            self.state = self.STATE_MOVING_TO_BIN
            self.log("Moving to bin for size:", size_label)
            self._send_and_wait(f"SORT,{size_label}", timeout=cfg.MOVEMENT_TIMEOUT)
        except DecisionError as e:
            self.log("Moving to bin failed:", e, level=0)
            self.state = self.STATE_ERROR
            return

        try:
            self.state = self.STATE_RELEASING
            self.log("State -> RELEASING")
            self._send_and_wait("GRIP,OPEN", timeout=3.0)
            self._send_and_wait("GRIP,CLOSE", timeout=3.0)
            self._send_and_wait("ARM,HOME", timeout=cfg.ARM_MOVE_TIMEOUT)
            self.arm_extended = False
            self.state = self.STATE_RETURNING_HOME
            self.log("State -> RETURNING_HOME")
            self._send_and_wait("MOVE,BACKWARD,40", timeout=5.0)
            self.state = self.STATE_READY
            self.log("State -> READY")
            self.pick_retries = 0
        except DecisionError as e:
            self.log("Release sequence failed:", e, level=0)
            self.state = self.STATE_ERROR
            return

    def fold_transform(self):
        """
        Fold the robot transform (to compact rover mode). Enforces
        arm-home before folding. NOTE: TRANSFORM,FOLD is accepted but
        not yet implemented on the ESP32 (returns ERROR,NOT_IMPLEMENTED
        until Phase 13) - this will raise DecisionError until then,
        which is expected.
        """
        if self.arm_extended:
            self.log("Cannot fold: arm not home", level=0)
            raise DecisionError("Arm must be home before folding")
        try:
            self.state = self.STATE_FOLDING
            self.log("State -> FOLDING")
            self._send_and_wait("TRANSFORM,FOLD", timeout=15.0)
            self.transform_folded = True
            self.state = self.STATE_ROVER_MODE
            self.log("State -> ROVER_MODE")
        except DecisionError:
            self.state = self.STATE_ERROR
            raise

    def unfold_transform(self):
        if self.transform_folded is False:
            self.log("Already unfolded", level=1)
            return
        try:
            self.state = self.STATE_UNFOLDING
            self.log("State -> UNFOLDING")
            self._send_and_wait("TRANSFORM,UNFOLD", timeout=15.0)
            self.transform_folded = False
            self.state = self.STATE_READY
            self.log("State -> READY")
        except DecisionError:
            self.state = self.STATE_ERROR
            raise

    def tick(self):
        """Optional periodic processing hook - placeholder for future events."""
        if self.emergency:
            self.log("Emergency active in tick", level=0)
            return
        try:
            pass
        except Exception:
            self.log("STATUS poll failed", level=2)

    def shutdown(self):
        try:
            self.comm.close()
        except Exception:
            pass
