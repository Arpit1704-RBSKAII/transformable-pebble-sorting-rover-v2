"""
calibration.py — PHASE 9: pixel-to-mm calibration tool
----------------------------------------------------------
Built fresh - required by project spec (section 19: "Pixel-to-physical
mapping") and listed as a required Phase 9 file, but did not exist
anywhere in the repo (the other Phase 9 files covered classification
logic and its config, not calibration itself).

WHAT THIS DOES:
Detects a single reference object (e.g. a ruler, a coin, a printed
square of known size) placed in frame, using the SAME detection
pipeline as detection.py, and computes CALIBRATION["pixel_to_mm"] from
its measured pixel diameter and the real-world size you provide.

Deliberately simple per project spec ("controlled environment... do
not overcomplicate calibration") - one reference object, one
measurement, printed instructions for pasting the result into
configuration.py. No persistent calibration file, no multi-point
lens-distortion correction.

Run: python calibration.py --known-diameter-mm 24.0
(24.0 mm is an example - a US quarter is ~24.26mm, adjust to whatever
reference object you actually use)

Controls: c = capture/measure current frame, q = quit
"""

import argparse
import logging

import cv2

import configuration
from camera import Camera, CameraError
from detection import detect_best_pebble


def setup_logging():
    logging.basicConfig(
        level=getattr(logging, configuration.LOG_LEVEL),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main():
    parser = argparse.ArgumentParser(description="Pixel-to-mm calibration tool")
    parser.add_argument(
        "--known-diameter-mm", type=float, required=True,
        help="Real-world diameter (in mm) of the reference object you'll place in frame.",
    )
    args = parser.parse_args()

    setup_logging()
    logger = logging.getLogger("calibration")

    logger.info(
        "Place a reference object of known diameter %.2fmm in the camera's "
        "view, on the same surface/distance pebbles will actually be "
        "detected at. Press 'c' to capture a measurement, 'q' to quit.",
        args.known_diameter_mm,
    )

    measurements = []

    try:
        with Camera() as cam:
            while True:
                try:
                    frame = cam.read_frame()
                except CameraError as exc:
                    logger.error("Frame read failed: %s", exc)
                    break

                detection = detect_best_pebble(frame)

                display = frame.copy()
                if detection.detected:
                    cv2.circle(display, (detection.x, detection.y), 5, (0, 255, 0), -1)
                    diameter_px = (detection.bbox[2] + detection.bbox[3]) / 2.0
                    cv2.putText(
                        display, f"diameter: {diameter_px:.1f}px", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                    )
                else:
                    cv2.putText(
                        display, "No object detected - adjust lighting/threshold", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
                    )

                cv2.putText(
                    display, f"Measurements so far: {len(measurements)}  (c=capture, q=quit)",
                    (10, display.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
                )

                cv2.imshow("Phase 9: Calibration (c=capture, q=quit)", display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("c"):
                    if not detection.detected:
                        logger.warning("Nothing detected right now - adjust the reference "
                                        "object or detection threshold, then try again.")
                        continue
                    diameter_px = (detection.bbox[2] + detection.bbox[3]) / 2.0
                    pixel_to_mm = args.known_diameter_mm / diameter_px
                    measurements.append(pixel_to_mm)
                    logger.info(
                        "Captured: diameter=%.1fpx -> pixel_to_mm=%.4f  (%d measurement%s so far)",
                        diameter_px, pixel_to_mm, len(measurements),
                        "" if len(measurements) == 1 else "s",
                    )

    except CameraError as exc:
        logger.error("Camera error: %s", exc)
    finally:
        cv2.destroyAllWindows()

    if not measurements:
        logger.warning("No measurements captured - nothing to report.")
        return

    average = sum(measurements) / len(measurements)
    logger.info("=" * 60)
    logger.info("Average pixel_to_mm over %d measurement(s): %.4f", len(measurements), average)
    logger.info("Paste this into configuration.py's CALIBRATION dict:")
    logger.info('    "pixel_to_mm": %.4f,', average)
    logger.info('    "is_calibrated": True,')
    logger.info("=" * 60)
    logger.info(
        "Take multiple measurements (move the reference object slightly "
        "between captures) for a more reliable average - single-shot "
        "calibration is easy to throw off with one bad detection."
    )


if __name__ == "__main__":
    main()
