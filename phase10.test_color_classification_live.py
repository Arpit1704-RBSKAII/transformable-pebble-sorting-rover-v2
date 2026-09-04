"""
test_color_classification_live.py — PHASE 10 LIVE HARDWARE TEST
--------------------------------------------------------------------
Chains the full real pipeline: camera -> detect_pebbles() ->
build_roi_and_mask() -> classify_colour(), so you can see actual
colour labels on actual pebbles under actual lighting - not just the
unit tests' synthetic uniform-colour patches.

Run: python test_color_classification_live.py
Controls: q = quit
"""

import logging
import time

import cv2

import configuration
from camera import Camera, CameraError
from detection import detect_pebbles, build_roi_and_mask
from color_classification import classify_colour


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    setup_logging()
    logger = logging.getLogger("test_color_classification_live")

    logger.info(
        "COLOR_REFERENCES_BGR are placeholder guesses - if colours look "
        "consistently wrong, sample your actual pebbles' BGR values (the "
        "'debug' -> 'mean_bgr' printed for each detection below is a good "
        "starting point) and update configuration.py."
    )

    try:
        with Camera() as cam:
            logger.info("Live colour classification started. Press 'q' to quit.")

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
                    roi, mask = build_roi_and_mask(frame, det)
                    if roi is None:
                        continue

                    result = classify_colour({"detected": True, "roi": roi, "roi_mask": mask})
                    label = result["colour"]
                    conf = result["confidence"]

                    radius = max(4, int((det.area_px / 3.14159) ** 0.5))
                    cv2.circle(frame, (det.x, det.y), radius, (255, 255, 255), 2)
                    cv2.putText(
                        frame, f"{label} ({conf:.2f})", (det.x + radius + 4, det.y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2,
                    )

                frame_count += 1
                elapsed = time.monotonic() - fps_timer_start
                if elapsed >= 1.0:
                    fps_display = frame_count / elapsed
                    frame_count = 0
                    fps_timer_start = time.monotonic()

                cv2.putText(
                    frame, f"FPS: {fps_display:.1f}  Pebbles: {len(detections)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                )

                cv2.imshow("Phase 10: Live Colour Classification (q=quit)", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    logger.info("Quit key pressed.")
                    break

    except CameraError as exc:
        logger.error("Camera error: %s", exc)
    finally:
        cv2.destroyAllWindows()
        logger.info("Phase 10 live colour classification test finished.")


if __name__ == "__main__":
    main()
