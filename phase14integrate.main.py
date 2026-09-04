"""
main.py — PHASE 14: COMPLETE INTEGRATION (production entry point)
---------------------------------------------------------------------
The real, continuous autonomous control loop: camera -> detection ->
decision state machine -> real ESP32 hardware over serial. This
replaces the Phase 6 smoke test (preserved separately as
test_serial_smoketest.py) as the actual thing you run to operate the
rover.

What this does:
  1. Connects to the ESP32 (SerialComm - real hardware, not MockComm).
  2. Opens the camera.
  3. Loop: read a frame, look for the best pebble, and hand it to the
     DecisionMachine whenever it's idle/searching. The state machine
     (decision.py) owns everything from there - moving the arm,
     gripping, sorting, returning home - this loop's only job is to
     keep feeding it frames and keep the window/logging alive.
  4. Ctrl+C (or closing the preview window) shuts down cleanly:
     disconnects serial, releases the camera, closes any windows.

SAFETY: This WILL move real motors/servos once a pebble is detected
with high enough confidence. Keep the work area clear, wheels able to
move freely (or propped up for a first run), and know where Ctrl+C
is - it triggers emergency_stop() before exiting, but a physical
E-STOP button (if wired - see PIN_EMERGENCY_STOP in
esp32/include/configuration.h) is faster and doesn't depend on this
script still being responsive.

Run: python main.py
"""

import logging
import signal
import sys

import cv2

import configuration
from camera import Camera, CameraError
from detection import detect_best_pebble, build_roi_and_mask
from decision import DecisionMachine
from communication import SerialComm, CommError
from communication import SerialTimeoutError as LinkTimeoutError


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    setup_logging()
    logger = logging.getLogger("main")

    logger.info("Connecting to ESP32 on %s...", configuration.SERIAL_PORT)
    comm = SerialComm()
    try:
        comm.connect()
    except (LinkTimeoutError, CommError, Exception) as exc:
        logger.error("Could not connect to ESP32: %s", exc)
        logger.error(
            "Check configuration.SERIAL_PORT (%s), that the Phase 14 "
            "firmware is uploaded, and that no other program (Serial "
            "Monitor, etc.) is holding the port open.",
            configuration.SERIAL_PORT,
        )
        sys.exit(1)

    dm = DecisionMachine(comm=comm)

    # Ctrl+C triggers a real emergency stop on the hardware before
    # exiting, rather than just killing the Python process and leaving
    # actuators wherever they happened to be.
    def handle_sigint(signum, frame):
        logger.warning("Ctrl+C received - triggering emergency stop before exit.")
        dm.emergency_stop()
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, handle_sigint)

    dm.start_search()
    logger.info("DecisionMachine started - searching for pebbles.")
    logger.info("Press 'q' in the preview window, or Ctrl+C here, to stop.")

    try:
        with Camera() as cam:
            while True:
                try:
                    frame = cam.read_frame()
                except CameraError as exc:
                    logger.error("Frame read failed: %s", exc)
                    break

                det = detect_best_pebble(frame)

                if det.detected and dm.state == dm.STATE_SEARCHING:
                    roi, mask = build_roi_and_mask(frame, det)
                    detection_dict = det.to_dict()
                    # image=frame is passed so classify_colour() can
                    # still work even if build_roi_and_mask() returned
                    # None (degenerate bbox) - see detection.py.
                    dm.process_detection(detection_dict, image=frame if roi is None else None)

                if det.detected:
                    radius = max(4, int((det.area_px / 3.14159) ** 0.5))
                    cv2.circle(frame, (det.x, det.y), radius, (0, 255, 0), 2)

                cv2.putText(
                    frame, f"DM state: {dm.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                )

                cv2.imshow("Pebble-Sorting Rover - Live", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    logger.info("Quit key pressed.")
                    break

    except KeyboardInterrupt:
        logger.info("Shutting down after emergency stop.")
    except CameraError as exc:
        logger.error("Camera error: %s", exc)
    finally:
        cv2.destroyAllWindows()
        dm.shutdown()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
