"""
test_serial_smoketest.py — Phase 6 serial link smoke test
--------------------------------------------------------------
Originally main.py (Phase 6). Preserved here unchanged as a
standalone smoke test for the serial link alone - useful for quickly
checking "is the ESP32 connected and responding" without needing the
camera or the full decision state machine running. main.py (this
folder) is now the real, continuous autonomous control loop built in
Phase 14 - see that file for the production entry point.

Dependency: pip install pyserial
"""

import logging
import sys

import configuration
from communication import (
    RoverSerialLink,
    SerialCommandError,
    SerialTimeoutError,
    list_serial_ports,
)


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    setup_logging()
    logger = logging.getLogger("main")

    logger.info("Available serial ports:")
    for device, description in list_serial_ports():
        logger.info("  %s - %s", device, description)

    link = RoverSerialLink()

    try:
        link.connect()
    except SerialTimeoutError as exc:
        logger.error("Could not connect: %s", exc)
        logger.error(
            "Check configuration.SERIAL_PORT (%s) matches your ESP32's port, "
            "and that Phase 5 firmware is uploaded.",
            configuration.SERIAL_PORT,
        )
        sys.exit(1)

    try:
        # --- STATUS ---
        status = link.get_status()
        logger.info("Initial status: %s", status)

        # --- ARM,HOME and wait for completion ---
        logger.info("Sending ARM,HOME...")
        response = link.send_command("ARM,HOME")
        logger.info("Immediate response: %s", response)
        if response == "OK":
            done = link.wait_for_done("ARM_HOME")
            logger.info("Completed: %s", done)
        elif response == "BUSY":
            logger.warning("ESP32 was busy - skipping wait.")
        else:
            logger.error("Unexpected response to ARM,HOME: %s", response)

        # --- GRIP,OPEN and wait for completion ---
        logger.info("Sending GRIP,OPEN...")
        response = link.send_command("GRIP,OPEN")
        logger.info("Immediate response: %s", response)
        if response == "OK":
            done = link.wait_for_done("GRIP_OPEN")
            logger.info("Completed: %s", done)

        # --- GRIP,CLOSE and wait for completion ---
        logger.info("Sending GRIP,CLOSE...")
        response = link.send_command("GRIP,CLOSE")
        logger.info("Immediate response: %s", response)
        if response == "OK":
            done = link.wait_for_done("GRIP_CLOSE")
            logger.info("Completed: %s", done)

        # --- Final status ---
        status = link.get_status()
        logger.info("Final status: %s", status)

        logger.info("Phase 6 smoke test completed successfully.")

    except SerialCommandError as exc:
        logger.error("ESP32 reported an error: %s", exc.reason)
    except SerialTimeoutError as exc:
        logger.error("Timed out waiting for a response: %s", exc)
    finally:
        link.disconnect()


if __name__ == "__main__":
    main()
