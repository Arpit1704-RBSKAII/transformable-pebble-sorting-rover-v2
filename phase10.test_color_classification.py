"""
Unit tests for colour classification module.

Run:
    pytest python/tests/test_color_classification.py

Reused as-is from the existing repo.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from color_classification import classify_colour
from configuration import COLOR_REFERENCES_BGR


def _make_test_detection_for_color(label):
    bgr = COLOR_REFERENCES_BGR[label]
    img = np.full((50, 50, 3), bgr, dtype=np.uint8)
    mask = np.ones((50, 50), dtype=np.uint8) * 255
    return {"detected": True, "roi": img, "roi_mask": mask}


def test_classify_brown():
    det = _make_test_detection_for_color("BROWN")
    res = classify_colour(det)
    assert res["detected"] is True
    assert res["colour"] in ("BROWN", "UNKNOWN")  # allow UNKNOWN if refs differ significantly
    assert 0.0 <= res["confidence"] <= 1.0


def test_classify_white():
    det = _make_test_detection_for_color("WHITE")
    res = classify_colour(det)
    assert res["detected"] is True
    assert res["colour"] in ("WHITE", "UNKNOWN")
    assert 0.0 <= res["confidence"] <= 1.0


def test_small_mask_rejected():
    det = {
        "detected": True,
        "roi": np.full((5, 5, 3), COLOR_REFERENCES_BGR["GREY"], dtype=np.uint8),
        "roi_mask": np.zeros((5, 5), dtype=np.uint8),
    }
    res = classify_colour(det)
    assert res["detected"] is False or res["colour"] == "UNKNOWN"
