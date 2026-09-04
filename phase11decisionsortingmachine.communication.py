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

    def read_line(self, timeout: float) -> Optional[str]:
        """
        Public wrapper around _read_line(), for callers (like SerialComm
        below) that need generic line-by-line reading rather than the
        DONE-specific wait_for_done().
        """
        return self._read_line(timeout)

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


# ============================================================
# BaseComm / MockComm / SerialComm — Phase 11 addition
# ============================================================
# decision.py (Phase 11) was already written against a DIFFERENT
# communication interface than RoverSerialLink above: send_command()
# returning just an immediate token, plus a generic wait_for(tokens,
# timeout). Rather than rewrite RoverSerialLink (tested and working
# since Phase 6) to match, these classes ADAPT it via composition -
# SerialComm wraps a RoverSerialLink internally. Nothing above this
# comment changes.


class CommError(Exception):
    pass


class BaseComm:
    """Abstract interface expected by decision.DecisionMachine."""

    def send_command(self, cmd: str) -> str:
        """Send a command string. Returns the IMMEDIATE response token
        (e.g. 'OK', 'BUSY', 'ERROR,...', 'READY')."""
        raise NotImplementedError()

    def wait_for(self, tokens, timeout: float) -> str:
        """Block until a line starting with one of tokens is received,
        or timeout. Returns that token. Raises TimeoutError on timeout."""
        raise NotImplementedError()

    def close(self):
        pass


class MockComm(BaseComm):
    """
    Test double used by tests/test_decision.py so DecisionMachine logic
    can be verified without any hardware attached.

    BUG FIX vs. the original repo version: send_command() used to
    return "BUSY" for every accepted command when auto_done=True. But
    in the REAL ESP32 protocol (communication.cpp, Phase 5), BUSY means
    a command was REJECTED because a previous critical action is still
    pending - it does NOT mean "accepted, now executing". The real
    "accepted, now executing" response is "OK", with completion
    reported later via a separate DONE,<label> line. Returning "BUSY"
    for accepted commands was simulating protocol semantics that don't
    match the real firmware, which happened to not matter for the
    original (also buggy) decision.py, since that code treated an
    immediate "OK" as terminal completion. With BOTH bugs fixed
    (here and in decision.py's _send_and_wait), MockComm now simulates
    the real protocol correctly: OK immediately, DONE after a delay.
    """

    def __init__(self, behavior_delay=0.05, auto_done=True):
        """
        behavior_delay: simulated time before a DONE is available
        auto_done: if True, wait_for() will eventually return 'DONE'
            for commands that expect one; if False, no DONE ever
            arrives (simulates a hung/unresponsive board).
        """
        self.behavior_delay = behavior_delay
        self.auto_done = auto_done
        self.sent_commands = []
        self._last_cmd = None
        self._cmd_sent_at = None

    def send_command(self, cmd: str) -> str:
        print(f"[MockComm] send: {cmd}")
        self.sent_commands.append(cmd)
        self._last_cmd = cmd
        self._cmd_sent_at = time.monotonic()

        if cmd.startswith("STATUS"):
            return "READY"

        # Real protocol: accepted commands get "OK" immediately, not
        # "BUSY". BUSY is reserved for rejecting a command sent while
        # another critical action is still pending - simulate that
        # separately via inject_busy() if a test needs it.
        return "OK"

    def wait_for(self, tokens, timeout: float) -> str:
        """
        Simulates waiting for an async DONE. If auto_done is True,
        returns 'DONE' once behavior_delay has elapsed (and 'DONE' is
        one of the requested tokens). If auto_done is False, always
        times out - simulating hardware that never responds.
        """
        elapsed = 0.0
        step = 0.02
        while elapsed < timeout:
            time.sleep(step)
            elapsed += step
            if self.auto_done and "DONE" in tokens and elapsed >= self.behavior_delay:
                print("[MockComm] -> DONE")
                return "DONE"
        raise TimeoutError("MockComm wait_for timed out")

    def close(self):
        pass


class SerialComm(BaseComm):
    """
    Adapts RoverSerialLink (the tested, real-hardware implementation
    from Phase 6) to the BaseComm interface DecisionMachine expects.
    Composition, not replacement - RoverSerialLink's connect()/
    send_command()/disconnect() logic is unchanged and reused as-is.
    """

    def __init__(self, link: Optional[RoverSerialLink] = None):
        self._link = link if link is not None else RoverSerialLink()

    def connect(self):
        """Not part of BaseComm, but needed before use - call this
        once before handing a SerialComm to DecisionMachine."""
        self._link.connect()

    def send_command(self, cmd: str) -> str:
        return self._link.send_command(cmd)

    def wait_for(self, tokens, timeout: float) -> str:
        """
        Generic multi-token wait, built on RoverSerialLink's public
        read_line(). Returns the matched token (e.g. 'DONE') - callers
        needing the full line (e.g. 'DONE,ARM_HOME') should use
        RoverSerialLink.wait_for_done() directly instead.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            line = self._link.read_line(timeout=remaining)
            if line is None:
                continue
            for token in tokens:
                if line == token or line.startswith(token + ","):
                    return token
            if line.startswith("ERROR,"):
                return line  # let caller's ERROR-prefix check handle it
        raise TimeoutError(f"SerialComm wait_for timed out waiting for {tokens}")

    def close(self):
        self._link.disconnect()
