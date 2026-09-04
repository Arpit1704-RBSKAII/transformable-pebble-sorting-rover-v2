"""
test_camera.py — PHASE 7 TEST VERSION
----------------------------------------
Standalone test of camera.py only. Opens the camera, shows a live
preview window with an FPS overlay, and exits cleanly on 'q' or
window close.

DESIGN NOTE - why this is a separate file from main.py:
Unlike the ESP32 side (where PlatformIO only builds one main.cpp per
phase, so each phase's test temporarily replaced the last), Python has
no such constraint - multiple .py files can coexist. From here on,
each phase gets its own small test_<module>.py script, and main.py is
reserved for the full integration entry point that gets built up in
later phases (Phase 11 decision state machine onward). Nothing is
being overwritten/lost between phases on the Python side.

Run: python test_camera.py
Press 'q' in the preview window to quit.

Dependency: pip install opencv-python numpy
"""

import logging
import time

import cv2

import configuration
from camera import Camera, CameraError


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    setup_logging()
    logger = logging.getLogger("test_camera")

    try:
        with Camera() as cam:
            logger.info("Camera stream started at %dx%d. Press 'q' to quit.", cam.width, cam.height)

            frame_count = 0
            fps_timer_start = time.monotonic()
            fps_display = 0.0

            while True:
                try:
                    frame = cam.read_frame()
                except CameraError as exc:
                    logger.error("Frame read failed: %s", exc)
                    break

                frame_count += 1
                elapsed = time.monotonic() - fps_timer_start
                if elapsed >= 1.0:
                    fps_display = frame_count / elapsed
                    frame_count = 0
                    fps_timer_start = time.monotonic()

                cv2.putText(
                    frame, f"FPS: {fps_display:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
                )
                cv2.imshow("Phase 7: Camera Test (press q to quit)", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Quit key pressed.")
                    break

    except CameraError as exc:
        logger.error("Camera error: %s", exc)
        logger.error(
            "Check CAMERA_INDEX in configuration.py (currently %d), and "
            "make sure no other program is using the camera.",
            configuration.CAMERA_INDEX,
        )
    finally:
        cv2.destroyAllWindows()
        logger.info("Phase 7 camera test finished.")


if __name__ == "__main__":
    main()
