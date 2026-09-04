"""
communication.py
-----------------
Python-side implementation of the serial command protocol defined in
esp32/src/communication.cpp (Phase 5).

Responsibilities:
  - Open/close the serial connection, waiting for the ESP32's
    "READY" line on connect (the ESP32 resets when the port opens).
  - Send a command line and read back the IMMEDIATE response
    (OK / BUSY / ERROR,<reason> / STATUS,...).
  - Separately wait for an async DONE,<label> line for critical
    actions (ARM,* / GRIP,* / SORT,*) that take time to complete.
  - Log every command sent and every line received, for debugging.

This is the ONLY module that knows the transport is USB serial.
decision.py (Phase 11) will call methods on RoverSerialLink without
knowing or caring that it's PySerial underneath - so a future
Wi-Fi/BLE transport just means writing a new class with the same
method signatures.

Reused as-is from the existing repo - already correct and matches the
ESP32 protocol exactly. The only actual gap was configuration.py,
which this file imports and which did not exist until now.

Dependency: pip install pyserial
"""

import logging
import time
from typing import Optional

import serial
import serial.tools.list_ports

import configuration

logger = logging.getLogger("communication")


class SerialCommandError(Exception):
    """Raised when the ESP32 responds with ERROR,<reason> to a command."""

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(f"ESP32 returned ERROR,{reason}")


class SerialTimeoutError(Exception):
    """Raised when an expected response does not arrive within the timeout."""


def list_serial_ports() -> list:
    """
    Returns a list of (device, description) tuples for every serial
    port currently visible to the OS. Useful for finding the correct
    SERIAL_PORT value in configuration.py without guessing.
    """
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in ports]


class RoverSerialLink:
    """
    Manages a single serial connection to the ESP32 and implements the
    request/response command protocol from Phase 5.

    Typical usage:
        link = RoverSerialLink()
        link.connect()
        response = link.send_command("ARM,HOME")   # returns "OK"
        link.wait_for_done("ARM_HOME")              # blocks until DONE,ARM_HOME
        link.disconnect()
    """

    def __init__(
        self,
        port: str = configuration.SERIAL_PORT,
        baud_rate: int = configuration.SERIAL_BAUD_RATE,
    ):
        self.port = port
        self.baud_rate = baud_rate
        self._serial: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """
        Opens the serial port and waits for the ESP32's "READY" line.
        Returns True on success. Raises SerialTimeoutError if READY
        does not arrive in time (ESP32 not booting, wrong port, wrong
        firmware, etc.).
        """
        logger.info("Connecting to ESP32 on %s at %d baud...", self.port, self.baud_rate)

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=configuration.COMMAND_RESPONSE_TIMEOUT_SECONDS,
            )
        except serial.SerialException as exc:
            logger.error("Failed to open serial port %s: %s", self.port, exc)
            raise

        # Opening the port resets the ESP32 (standard behavior on most
        # boards via DTR toggling) - give it time to boot and print READY.
        deadline = time.monotonic() + configuration.CONNECT_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            line = self._read_line(timeout=deadline - time.monotonic())
            if line is None:
                continue
            logger.debug("Boot line: %s", line)
            if line == "READY":
                logger.info("ESP32 is READY.")
                return True

        self.disconnect()
        raise SerialTimeoutError(
            f"Did not receive READY from ESP32 within {configuration.CONNECT_TIMEOUT_SECONDS}s"
        )

    def disconnect(self):
        """Closes the serial connection if open. Safe to call multiple times."""
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            logger.info("Disconnected from ESP32.")
        self._serial = None

    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # Core send/receive
    # ------------------------------------------------------------------

    def send_command(self, command: str) -> str:
        """
        Sends a single command line and returns the ESP32's IMMEDIATE
        response (e.g. "OK", "BUSY", "ERROR,MALFORMED_MOVE_COMMAND",
        or a "STATUS,..." line).

        This does NOT wait for an async DONE - call wait_for_done()
        separately after this for critical actions (ARM/GRIP/SORT).

        Raises SerialTimeoutError if no response line arrives in time.
        """
        if not self.is_connected():
            raise RuntimeError("Not connected - call connect() first.")

        logger.debug("Sending: %s", command)
        self._serial.write((command + "\n").encode("ascii"))
        self._serial.flush()

        response = self._read_line(timeout=configuration.COMMAND_RESPONSE_TIMEOUT_SECONDS)
        if response is None:
            raise SerialTimeoutError(f"No response to command: {command}")

        logger.debug("Received: %s", response)
        return response

    def wait_for_done(self, expected_label: Optional[str] = None, timeout: Optional[float] = None) -> str:
        """
        Blocks until a "DONE,<label>" line arrives, an "ERROR,<reason>"
        line arrives (raises SerialCommandError), or the timeout
        expires (raises SerialTimeoutError).

        If expected_label is given, a DONE line for a DIFFERENT label
        is logged as a warning and ignored (kept waiting) - this
        guards against a stale DONE from a previous command confusing
        the caller, though the ESP32's one-pending-action-at-a-time
        design should make this rare.

        Returns the full DONE,<label> line on success.
        """
        if not self.is_connected():
            raise RuntimeError("Not connected - call connect() first.")

        if timeout is None:
            timeout = configuration.DONE_TIMEOUT_SECONDS

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._read_line(timeout=deadline - time.monotonic())
            if line is None:
                continue

            logger.debug("Received (while waiting for DONE): %s", line)

            if line.startswith("ERROR,"):
                reason = line.split(",", 1)[1]
                raise SerialCommandError(reason)

            if line.startswith("DONE,"):
                label = line.split(",", 1)[1]
                if expected_label is not None and label != expected_label:
                    logger.warning(
                        "Received DONE,%s while waiting for DONE,%s - ignoring.",
                        label, expected_label,
                    )
                    continue
                return line

        raise SerialTimeoutError(
            f"No DONE{',' + expected_label if expected_label else ''} within {timeout}s"
        )

    def get_status(self) -> dict:
        """
        Sends STATUS and parses the response into a dict, e.g.
        {"ARM": 0, "GRIPPER": 1, "ESTOP": 0, "PENDING": 0}.

        Raises SerialCommandError/SerialTimeoutError like send_command().
        """
        response = self.send_command("STATUS")
        if not response.startswith("STATUS,"):
            raise SerialCommandError(f"Unexpected STATUS response: {response}")

        fields = response[len("STATUS,"):].split(",")
        status = {}
        for field in fields:
            key, _, value = field.partition("=")
            try:
                status[key] = int(value)
            except ValueError:
                status[key] = value
        return status

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_line(self, timeout: float) -> Optional[str]:
        """
        Reads one line from serial with an explicit timeout, returning
        None if nothing arrived. PySerial's own timeout is set once at
        construction, so we temporarily override it per-call to support
        the deadline-based waiting used in connect()/wait_for_done().
        """
        if timeout <= 0:
            return None

        original_timeout = self._serial.timeout
        self._serial.timeout = timeout
        try:
            raw = self._serial.readline()
        finally:
            self._serial.timeout = original_timeout

        if not raw:
            return None

        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception as exc:  # pragma: no cover - defensive
            logger.warning("Failed to decode line %r: %s", raw, exc)
            return None

        return line if line else None
