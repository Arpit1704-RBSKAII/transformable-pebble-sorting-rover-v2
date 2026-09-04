"""
main.py — PHASE 6 TEST VERSION
--------------------------------
Standalone smoke test for the serial link only. Connects to the
ESP32, exercises STATUS/ARM/GRIP commands end-to-end (including
waiting for DONE), and disconnects. No camera, no detection, no
decision logic yet - those arrive in Phases 7-11.

Run this AFTER uploading the Phase 5 ESP32 firmware and confirming
it works from Serial Monitor - Python is just automating the same
commands you tested manually.

Reused as-is from the existing repo - already correct.

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
