"""
color_classification.py
------------------------
Colour classification module (Phase 10, optional per project spec).

API:
    classify_colour(detection, image=None, color_refs=None, max_distance=None)

detection may contain:
    - 'detected': bool
    - 'mask': numpy.ndarray same size as image, dtype=uint8 (0/255) OR boolean
    - 'roi': numpy.ndarray (BGR cropped image)
    - 'roi_mask': numpy.ndarray same size as roi
    - 'bbox': (x, y, w, h)
    - 'image': (optional) full BGR image if needed

Returns:
    {
        "detected": True/False,
        "colour": "BROWN"/.../"UNKNOWN",
        "confidence": 0..1,
        "lab": (L,a,b) mean colour (optional),
        "debug": { ... }
    }

Reused as-is from the existing repo (module was named/labeled "main"
in the repo, renamed here to match what it actually is). Traced
through all three of test_color_classification.py's test cases by
hand before reuse - no bugs found, unlike classification.py (Phase 9)
which had a real one.

Prefer calling detection.build_roi_and_mask(frame, detection) to get
a proper roi/roi_mask pair (elliptical mask, not a full bbox rectangle)
rather than relying on this module's own bbox-only fallback path
(Case 3 below), which its own comment flags as "risky".
"""

from typing import Dict, Tuple, Optional

import numpy as np
import cv2

from configuration import COLOR_REFERENCES_BGR, COLOR_MAX_DISTANCE, MIN_COLOR_MASK_PIXELS, COLOR_CONFIDENCE_FLOOR


# Convert BGR reference map to Lab at module import time for speed.
def _bgr_to_lab_array(bgr: Tuple[int, int, int]) -> np.ndarray:
    bgr_arr = np.uint8([[list(bgr)]])  # shape (1,1,3)
    lab = cv2.cvtColor(bgr_arr, cv2.COLOR_BGR2LAB)
    return lab[0, 0].astype(np.float32)


_COLOR_REFERENCES_LAB = {k: _bgr_to_lab_array(v) for k, v in COLOR_REFERENCES_BGR.items()}


def _mean_bgr_in_mask(image: np.ndarray, mask: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """
    image: BGR numpy array
    mask: single-channel mask uint8 or bool; non-zero indicates object
    Returns mean BGR tuple or None if mask too small.
    """
    if mask is None:
        return None

    mask_bool = mask.astype(bool)
    count = int(np.count_nonzero(mask_bool))
    if count < MIN_COLOR_MASK_PIXELS:
        return None

    if mask.dtype != np.uint8:
        mask_uint8 = (mask_bool.astype(np.uint8) * 255).astype(np.uint8)
    else:
        mask_uint8 = mask

    mean = cv2.mean(image, mask=mask_uint8)  # returns (b,g,r,a)
    return (mean[0], mean[1], mean[2])


def _bgr_to_lab_tuple(bgr_tuple: Tuple[float, float, float]) -> np.ndarray:
    bgr_arr = np.uint8([[list(map(int, np.clip(bgr_tuple, 0, 255)))]])
    lab = cv2.cvtColor(bgr_arr, cv2.COLOR_BGR2LAB)
    return lab[0, 0].astype(np.float32)


def _lab_distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b))


def _distance_to_confidence(dist: float, max_distance: float = COLOR_MAX_DISTANCE) -> float:
    # Simple linear mapping: 0 -> 1.0 ; max_distance -> 0.0 ; clamp below 0
    conf = 1.0 - (dist / float(max_distance))
    conf = max(0.0, min(1.0, conf))
    return conf


def classify_colour(
    detection: Dict,
    image: Optional[np.ndarray] = None,
    color_refs_lab: Optional[Dict[str, np.ndarray]] = None,
    max_distance: Optional[float] = None,
) -> Dict:
    """
    Classify the colour of a detection.

    detection: dict from the detection pipeline. Prefer 'roi' +
    'roi_mask' (see detection.build_roi_and_mask()) over 'mask'+'image'
    or bare 'bbox'+'image'.
    image: full BGR image (optional if detection contains roi)
    color_refs_lab: optional overrides for references (Lab arrays). Defaults to config.
    max_distance: override for normalization.
    """
    result = {
        "detected": False,
        "colour": "UNKNOWN",
        "confidence": COLOR_CONFIDENCE_FLOOR,
        "lab": None,
        "debug": {},
    }

    if not detection or not detection.get("detected", False):
        result["debug"]["reason"] = "no_detection"
        return result

    roi_img = None
    roi_mask = None

    # Case 1: caller provided roi & roi_mask (PREFERRED - see
    # detection.build_roi_and_mask())
    if detection.get("roi") is not None and detection.get("roi_mask") is not None:
        roi_img = detection["roi"]
        roi_mask = detection["roi_mask"]
        result["debug"]["mask_source"] = "roi_mask"

    # Case 2: detection contains a full-frame mask + a full image
    elif detection.get("mask") is not None:
        if image is None:
            image = detection.get("image")
        if image is None:
            result["debug"]["reason"] = "no_image_for_mask"
            return result
        roi_img = image
        roi_mask = detection["mask"]
        result["debug"]["mask_source"] = "full_image_mask"

    # Case 3: fallback to bbox cropping if bbox and image provided.
    # RISKY: fills the entire bounding box as the mask, including
    # background in the corners - prefer Case 1 via build_roi_and_mask().
    elif detection.get("bbox") and image is not None:
        x, y, w, h = detection["bbox"]
        roi_img = image[y:y + h, x:x + w]
        roi_mask = np.ones((roi_img.shape[0], roi_img.shape[1]), dtype=np.uint8) * 255
        result["debug"]["mask_source"] = "bbox_full"

    else:
        result["debug"]["reason"] = "insufficient_inputs_for_colour"
        return result

    mean_bgr = _mean_bgr_in_mask(roi_img, roi_mask)
    if mean_bgr is None:
        result["debug"]["reason"] = "mask_too_small_or_empty"
        return result

    lab_mean = _bgr_to_lab_tuple(mean_bgr)
    result["lab"] = (float(lab_mean[0]), float(lab_mean[1]), float(lab_mean[2]))
    result["detected"] = True

    if color_refs_lab is None:
        color_refs_lab = _COLOR_REFERENCES_LAB
    if max_distance is None:
        max_distance = COLOR_MAX_DISTANCE

    distances = {}
    for label, lab_ref in color_refs_lab.items():
        d = _lab_distance(lab_mean, lab_ref)
        distances[label] = d

    best_label = min(distances, key=distances.get)
    best_dist = distances[best_label]
    confidence = _distance_to_confidence(best_dist, max_distance)

    # If the best match is meaningfully better than the second-best,
    # boost confidence slightly - a clear winner is more trustworthy
    # than a close call between two similar reference colours.
    sorted_items = sorted(distances.items(), key=lambda kv: kv[1])
    if len(sorted_items) >= 2:
        second_dist = sorted_items[1][1]
        margin = second_dist - best_dist
        if margin > 10.0:
            confidence = min(1.0, confidence + 0.08)

    if best_dist > max_distance * 1.4:
        result["colour"] = "UNKNOWN"
        result["confidence"] = 0.0
    else:
        result["colour"] = best_label
        result["confidence"] = round(float(confidence), 3)

    result["debug"]["distances"] = {k: round(float(v), 2) for k, v in distances.items()}
    result["debug"]["best_dist"] = round(float(best_dist), 2)
    result["debug"]["mean_bgr"] = tuple(round(float(x), 1) for x in mean_bgr)

    return result


if __name__ == "__main__":
    bgr = COLOR_REFERENCES_BGR["BROWN"]
    img = np.full((100, 100, 3), bgr, dtype=np.uint8)
    mask = np.ones((100, 100), dtype=np.uint8) * 255
    detection = {"detected": True, "roi": img, "roi_mask": mask}
    out = classify_colour(detection)
    print("Colour classification result:", out)
