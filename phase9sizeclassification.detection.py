"""
detection.py
------------
Classical OpenCV pebble detection: blur -> threshold -> morphological
cleanup -> contours -> area/circularity/zone filtering -> centroid.

Built to satisfy python/tests/test_detection.py, which already existed
in the repo and fully specified this module's expected API and
behavior even though the module itself did not exist.

Deliberately does NOT use deep learning, per project spec - classical
thresholding/contour techniques only.

Pipeline (matches project spec section 18):
  CAMERA FRAME -> GRAYSCALE -> BLUR -> THRESHOLD -> MORPHOLOGY ->
  CONTOURS -> AREA FILTER -> CIRCULARITY FILTER -> ZONE FILTER ->
  CENTROID -> PebbleDetection

Size/colour classification are NOT done here - that's classification.py
(Phase 9/10). This module only answers "where are the pebble-shaped
things in this frame."
"""

import math
from dataclasses import dataclass
from typing import List

import cv2
import numpy as np

import configuration


@dataclass
class PebbleDetection:
    """
    One detected pebble-shaped blob.

    area_px and confidence are exposed here (not just in to_dict())
    because detect_best_pebble() and classification.py (Phase 9) need
    the raw numeric values, not just the serialized dict form.

    bbox and perimeter were added for Phase 9: classification.py's
    classify_size() expects a dict with 'bbox' and 'perimeter' keys
    (it was already written and tested against that exact shape before
    this module existed) - to_dict() below produces that shape directly
    so detect_best_pebble(frame).to_dict() can be passed straight into
    classify_size() with no adapter needed.
    """

    detected: bool
    x: int
    y: int
    area_px: float
    confidence: float
    bbox: tuple = (0, 0, 0, 0)   # (x, y, w, h) in pixels, from cv2.boundingRect
    perimeter: float = 0.0        # contour perimeter in pixels

    def to_dict(self) -> dict:
        """
        Shape matches BOTH:
          - project spec section 18's example output (detected/x/y/confidence)
          - classification.py's classify_size() expected input
            (detected/bbox/area/perimeter)
        size/colour keys are deliberately NOT included here - those get
        added by classification.py's own OUTPUT (Phase 9) and colour
        classification (Phase 10), not by this module.
        """
        return {
            "detected": self.detected,
            "x": self.x,
            "y": self.y,
            "confidence": round(self.confidence, 3),
            "bbox": self.bbox,
            "area": self.area_px,
            "perimeter": self.perimeter,
        }


def _empty_detection() -> PebbleDetection:
    return PebbleDetection(detected=False, x=0, y=0, area_px=0.0, confidence=0.0)


def detect_pebbles(frame: np.ndarray) -> List[PebbleDetection]:
    """
    Runs the full detection pipeline on a single BGR frame and returns
    one PebbleDetection per valid blob found (empty list if none).

    Does NOT claim real-world size/distance from this - pixel
    measurements only, per project spec (no physical dimensions
    without calibration.py, which comes later).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    blur_k = configuration.DETECTION_BLUR_KERNEL
    blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)

    thresh_type = (
        cv2.THRESH_BINARY_INV if configuration.DETECTION_INVERT_THRESHOLD else cv2.THRESH_BINARY
    )
    _, thresh = cv2.threshold(
        blurred, configuration.DETECTION_THRESHOLD_VALUE, 255, thresh_type
    )

    morph_k = configuration.DETECTION_MORPH_KERNEL
    kernel = np.ones((morph_k, morph_k), np.uint8)
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame_h, frame_w = frame.shape[:2]
    zx0f, zy0f, zx1f, zy1f = configuration.DETECTION_ZONE_FRACTION
    zone_x0, zone_y0 = zx0f * frame_w, zy0f * frame_h
    zone_x1, zone_y1 = zx1f * frame_w, zy1f * frame_h

    detections: List[PebbleDetection] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < configuration.MIN_CONTOUR_AREA_PX:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue

        # Circularity: 1.0 for a perfect circle, lower for elongated/
        # irregular shapes. Rejects smears, shadows, and reflections
        # that pass the area filter but clearly aren't round pebbles.
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < configuration.DETECTION_MIN_CIRCULARITY:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])

        if not (zone_x0 <= cx <= zone_x1 and zone_y0 <= cy <= zone_y1):
            continue

        # Circularity doubles as a simple confidence score - a near-
        # perfect circle is more likely to genuinely be a pebble than
        # a rough, irregular blob that only just cleared the threshold.
        confidence = min(circularity, 1.0)

        bbox = cv2.boundingRect(contour)  # (x, y, w, h)

        detections.append(
            PebbleDetection(
                detected=True,
                x=cx,
                y=cy,
                area_px=area,
                confidence=confidence,
                bbox=bbox,
                perimeter=perimeter,
            )
        )

    return detections


def detect_best_pebble(frame: np.ndarray) -> PebbleDetection:
    """
    Convenience wrapper for decision.py (Phase 11), which generally
    only cares about ONE pebble to act on at a time. Returns the
    highest-confidence detection, or a detected=False result if
    nothing was found.
    """
    detections = detect_pebbles(frame)
    if not detections:
        return _empty_detection()
    return max(detections, key=lambda d: d.confidence)


def build_roi_and_mask(frame: np.ndarray, detection: PebbleDetection):
    """
    Crops the region around a detection and builds an ELLIPTICAL mask
    approximating the pebble's shape within that crop, for use with
    classification.py's classify_colour().

    Why not just use the bbox as the mask: color_classification.py's
    own fallback path (used when no roi_mask is supplied) fills the
    ENTIRE bounding box as the mask, which its own code comments flag
    as "risky" - a square/rectangular mask includes background pixels
    in the box's corners, which skews the mean colour away from the
    pebble's actual colour, especially for a round pebble in a square
    bounding box. An ellipse inscribed in the bbox is a cheap
    improvement that excludes most of those corner pixels without
    needing the full contour mask.

    Returns (roi, mask) as (BGR crop, uint8 mask) - or (None, None) if
    the detection's bbox is degenerate (zero width/height).
    """
    x, y, w, h = detection.bbox
    if w <= 0 or h <= 0:
        return None, None

    roi = frame[y:y + h, x:x + w]
    mask = np.zeros((h, w), dtype=np.uint8)
    center = (w // 2, h // 2)
    axes = (max(1, w // 2 - 1), max(1, h // 2 - 1))
    cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
    return roi, mask
