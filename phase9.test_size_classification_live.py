"""
test_size_classification_live.py — PHASE 9 LIVE HARDWARE TEST
-------------------------------------------------------------
Unit tests (tests/test_classification.py) verify classify_size()'s
logic against hand-built dicts, not real pebbles. This script chains
the real pipeline together - camera -> detect_best_pebble() ->
.to_dict() -> classify_size() - so you can see actual SMALL/MEDIUM/
LARGE labels on actual pebbles in actual lighting.

Run: python test_size_classification_live.py
Controls: q = quit
"""

import logging
import time

import cv2

import configuration
from camera import Camera, CameraError
from detection import detect_pebbles
from classification import classify_size


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    setup_logging()
    logger = logging.getLogger("test_size_classification_live")

    calibrated = configuration.CALIBRATION.get("is_calibrated", False)
    if not calibrated:
        logger.warning(
            "CALIBRATION['is_calibrated'] is False - labels below are from "
            "FALLBACK_PIXEL_AREA_THRESHOLDS (pixel-area guesses), not real "
            "mm measurements. Run calibration.py first for accurate SMALL/"
            "MEDIUM/LARGE thresholds based on real pebble sizes."
        )

    try:
        with Camera() as cam:
            logger.info("Live size classification started. Press 'q' to quit.")

            frame_count = 0
            fps_timer_start = time.monotonic()
            fps_display = 0.0

            while True:
                try:
                    frame = cam.read_frame()
                except CameraError as exc:
                    logger.error("Frame read failed: %s", exc)
                    break

                detections = detect_pebbles(frame)

                for det in detections:
                    classification = classify_size(det.to_dict())
                    label = classification["size_label"]
                    conf = classification["confidence"]

                    color = {"SMALL": (0, 255, 255), "MEDIUM": (0, 255, 0), "LARGE": (255, 0, 0)}.get(
                        label, (128, 128, 128)
                    )

                    radius = max(4, int((det.area_px / 3.14159) ** 0.5))
                    cv2.circle(frame, (det.x, det.y), radius, color, 2)
                    cv2.putText(
                        frame, f"{label} ({conf:.2f})", (det.x + radius + 4, det.y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                    )

                frame_count += 1
                elapsed = time.monotonic() - fps_timer_start
                if elapsed >= 1.0:
                    fps_display = frame_count / elapsed
                    frame_count = 0
                    fps_timer_start = time.monotonic()

                cv2.putText(
                    frame, f"FPS: {fps_display:.1f}  Pebbles: {len(detections)}  "
                    f"Calibrated: {calibrated}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )

                cv2.imshow("Phase 9: Live Size Classification (q=quit)", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Quit key pressed.")
                    break

    except CameraError as exc:
        logger.error("Camera error: %s", exc)
    finally:
        cv2.destroyAllWindows()
        logger.info("Phase 9 live classification test finished.")


if __name__ == "__main__":
    main()
