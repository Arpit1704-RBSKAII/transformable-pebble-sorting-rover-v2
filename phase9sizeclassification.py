"""
classification.py
------------------
Size classification module.

API:
    classify_size(detection, calibration=CALIBRATION, thresholds=SIZE_THRESHOLDS_MM)

where detection is a dict with keys:
    - 'detected': bool
    - 'bbox': (x, y, w, h)          # pixels
    - 'area': int                   # contour area in pixels
    - 'perimeter': float            # contour perimeter in pixels (optional)
    - 'contour_solidity': float     # optional 0..1 solidity measure
    - 'pixel_diameter': float       # optional precomputed diameter in pixels

This is exactly the shape detection.py's PebbleDetection.to_dict()
produces (Phase 8/9 bridge - see detection.py's to_dict() docstring),
so detect_best_pebble(frame).to_dict() can be passed straight in.

Returns dict:
    {
        "detected": True/False,
        "size_label": "SMALL"/"MEDIUM"/"LARGE"/"UNKNOWN",
        "size_mm": <estimated diameter in mm or None>,
        "size_mm_area": <area estimate in mm^2 if computed, else None>,
        "confidence": 0..1,
        "debug": {...}
    }

Reused from the existing repo (already well-designed and matches
python/tests/test_classification.py, which existed before this file
did) with ONE bug fixed - see _diameter_pixels_to_mm() below.
"""

from typing import Dict, Tuple, Optional
import math

from configuration import (
    CALIBRATION,
    SIZE_THRESHOLDS_MM,
    FALLBACK_PIXEL_AREA_THRESHOLDS,
    MIN_CONTOUR_AREA_PIXELS,
    ROUNDNESS_BONUS,
)


def _area_pixels_to_mm2(area_px: float, calibration: Dict) -> Optional[float]:
    if calibration.get("mode") == "area_pixel_to_mm2" and calibration.get("area_pixel_to_mm2"):
        return area_px * float(calibration["area_pixel_to_mm2"])
    if calibration.get("mode") == "pixel_to_mm" and calibration.get("pixel_to_mm"):
        # approximate area using (pixel_to_mm)^2
        mm_per_px = float(calibration["pixel_to_mm"])
        return area_px * (mm_per_px ** 2)
    return None


def _diameter_pixels_to_mm(diameter_px: float, calibration: Dict, area_px: Optional[float] = None) -> Optional[float]:
    """
    BUG FIX vs. the original repo version: the original always took a
    DIAMETER in pixels as its argument, but in the "area_pixel_to_mm2"
    branch it multiplied that diameter value directly by
    calibration["area_pixel_to_mm2"] as though it were an AREA - a
    genuine unit mismatch (the original code even had a comment
    admitting "if caller passed area_px", acknowledging the confusion
    without fixing it). That path was dormant only because the default
    CALIBRATION["mode"] is "pixel_to_mm", not "area_pixel_to_mm2".

    Fixed by accepting the REAL area_px as an explicit optional
    parameter, so the area-based branch converts the actual contour
    area (not the diameter) into mm^2 before deriving a diameter from
    it via the circular-area formula.
    """
    if calibration.get("mode") == "pixel_to_mm" and calibration.get("pixel_to_mm"):
        return float(diameter_px) * float(calibration["pixel_to_mm"])

    if (
        calibration.get("mode") == "area_pixel_to_mm2"
        and calibration.get("area_pixel_to_mm2")
        and area_px is not None
    ):
        area_mm2 = float(area_px) * float(calibration["area_pixel_to_mm2"])
        # estimate diameter assuming circular shape: area = pi*(d/2)^2
        d_mm = math.sqrt(4.0 * area_mm2 / math.pi)
        return d_mm

    return None


def _estimate_diameter_from_bbox(bbox: Tuple[int, int, int, int]) -> float:
    # Use average of width and height as approximate diameter in pixels
    _, _, w, h = bbox
    return float((w + h) / 2.0)


def _compute_roundness(area: float, perimeter: Optional[float]) -> float:
    # Circularity: 4*pi*area / perimeter^2 (0..1, 1 = perfect circle)
    if not perimeter or perimeter <= 0:
        return 0.0
    circ = 4.0 * math.pi * area / (perimeter * perimeter)
    return max(0.0, min(1.0, circ))


def classify_size(detection: Dict, calibration: Dict = CALIBRATION, thresholds: Dict = SIZE_THRESHOLDS_MM) -> Dict:
    """
    Classify a single detection into SMALL/MEDIUM/LARGE.
    detection expects keys as documented in the module docstring.
    """
    result = {
        "detected": False,
        "size_label": "UNKNOWN",
        "size_mm": None,
        "size_mm_area": None,
        "confidence": 0.0,
        "debug": {},
    }

    if not detection or not detection.get("detected", False):
        result["debug"]["reason"] = "no_detection"
        return result

    area_px = float(detection.get("area", 0))
    if area_px < MIN_CONTOUR_AREA_PIXELS:
        result["debug"]["reason"] = "area_below_minimum"
        return result

    result["detected"] = True

    bbox = detection.get("bbox")
    pixel_diameter = detection.get("pixel_diameter")
    perimeter = detection.get("perimeter")
    solidity = detection.get("contour_solidity", None)

    # Estimate pixel diameter if not provided
    if not pixel_diameter:
        if bbox:
            pixel_diameter = _estimate_diameter_from_bbox(bbox)
            result["debug"]["diameter_source"] = "bbox_average"
        else:
            # fallback: approximate diameter from area assuming circular shape
            pixel_diameter = math.sqrt(4.0 * area_px / math.pi)
            result["debug"]["diameter_source"] = "area_circle_assumption"

    result["debug"]["pixel_diameter"] = pixel_diameter
    result["debug"]["area_px"] = area_px

    # Convert to mm using calibration
    size_mm = None
    size_mm_area = _area_pixels_to_mm2(area_px, calibration)

    if calibration.get("is_calibrated", False):
        # prefer linear diameter mapping if available (area_px passed
        # through for the area-based calibration mode - see bug fix note above)
        size_mm = _diameter_pixels_to_mm(pixel_diameter, calibration, area_px=area_px)
    else:
        # Not calibrated: size_mm remains None; classification falls
        # back to pixel-area thresholds instead of trusting a made-up mm value.
        size_mm = None

    result["size_mm"] = size_mm
    result["size_mm_area"] = size_mm_area

    # Compute basic confidence from area size and roundness
    confidence = 0.5  # default baseline confidence

    # bigger area normally more confident (to an extent)
    if area_px > 1000:
        confidence += 0.1
    elif area_px < 200:
        confidence -= 0.15

    # roundness
    if perimeter:
        roundness = _compute_roundness(area_px, perimeter)
        result["debug"]["roundness"] = roundness
        confidence += ROUNDNESS_BONUS * roundness
    elif solidity is not None:
        # if solidity available (0..1)
        result["debug"]["solidity"] = solidity
        confidence += (solidity - 0.5) * 0.2

    # clamp
    confidence = max(0.0, min(1.0, confidence))

    # Classification decision
    size_label = "UNKNOWN"

    if calibration.get("is_calibrated", False) and size_mm is not None:
        # use thresholds in mm
        for label, (min_mm, max_mm) in thresholds.items():
            if size_mm >= min_mm and size_mm < max_mm:
                size_label = label
                break
    else:
        # fallback: use pixel area thresholds
        for label, (min_px, max_px) in FALLBACK_PIXEL_AREA_THRESHOLDS.items():
            if area_px >= min_px and area_px < max_px:
                size_label = label
                # give lower confidence if not calibrated
                confidence *= 0.6
                break

    # If size_label still unknown, attempt to use pixel_diameter heuristics
    if size_label == "UNKNOWN":
        # heuristics on pixel diameter (very approximate)
        if pixel_diameter < 20:
            size_label = "SMALL"
            confidence *= 0.6
        elif pixel_diameter < 60:
            size_label = "MEDIUM"
            confidence *= 0.65
        else:
            size_label = "LARGE"
            confidence *= 0.7

    result["size_label"] = size_label
    result["confidence"] = round(confidence, 3)

    return result


# Simple CLI/demo helper (for quick manual testing)
if __name__ == "__main__":
    example_detection = {
        "detected": True,
        "bbox": (100, 120, 40, 35),
        "area": 1100,
        "perimeter": 120.0,
        "contour_solidity": 0.92,
    }
    print("Calibration:", CALIBRATION)
    res = classify_size(example_detection)
    print("Classification result:", res)
