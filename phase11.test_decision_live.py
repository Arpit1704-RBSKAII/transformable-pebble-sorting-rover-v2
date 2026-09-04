"""
test_decision_live.py — PHASE 11 LIVE TEST
--------------------------------------------
Chains camera -> detect_best_pebble() -> DecisionMachine.process_detection()
so you can watch the full state machine react to a real pebble in real
time, with the current state overlaid on screen.

By default this uses MockComm (SAFE - no hardware required, no real
motors/servos move). Pass --real to connect to actual ESP32 hardware
via SerialComm instead - only do this once Phases 1-6 have been tested
and the arm/gripper/wheels are ready to actually move.

Run: python test_decision_live.py
     python test_decision_live.py --real   (connects to real hardware - CAUTION)

Controls: s = start searching, e = emergency stop, c = clear emergency, q = quit
"""

import argparse
import logging
import time

import cv2

import configuration
from camera import Camera, CameraError
from detection import detect_best_pebble, build_roi_and_mask
from decision import DecisionMachine
from communication import MockComm, SerialComm


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--real", action="store_true",
        help="Connect to real ESP32 hardware via SerialComm instead of MockComm. "
             "CAUTION: this will actually move motors/servos.",
    )
    args = parser.parse_args()

    setup_logging()
    logger = logging.getLogger("test_decision_live")

    if args.real:
        logger.warning(
            "Connecting to REAL hardware - motors/servos WILL move. "
            "Make sure the rover is clear and wheels are safe to move."
        )
        comm = SerialComm()
        comm.connect()
    else:
        logger.info("Using MockComm - safe dry-run, no hardware will move.")
        comm = MockComm(behavior_delay=0.3, auto_done=True)

    dm = DecisionMachine(comm=comm)

    logger.info("Controls: s=start searching, e=emergency stop, c=clear emergency, q=quit")

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
                    dm.process_detection(detection_dict, image=frame if roi is None else None)

                if det.detected:
                    radius = max(4, int((det.area_px / 3.14159) ** 0.5))
                    cv2.circle(frame, (det.x, det.y), radius, (0, 255, 0), 2)

                cv2.putText(
                    frame, f"DM state: {dm.state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                )
                cv2.putText(
                    frame, "s=search  e=estop  c=clear  q=quit", (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
                )

                cv2.imshow("Phase 11: Live Decision State Machine", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    dm.start_search()
                elif key == ord("e"):
                    dm.emergency_stop()
                elif key == ord("c"):
                    dm.clear_emergency()

    except CameraError as exc:
        logger.error("Camera error: %s", exc)
    finally:
        cv2.destroyAllWindows()
        dm.shutdown()
        logger.info("Phase 11 live test finished.")


if __name__ == "__main__":
    main()
