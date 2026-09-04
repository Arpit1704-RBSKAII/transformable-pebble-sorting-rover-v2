"""
Unit tests for classification module. These are lightweight and do not
require hardware.

Run:
    pytest python/tests/test_classification.py

Reused as-is from the existing repo - this file already existed and
specified classify_size()'s expected input/output shape before
classification.py existed.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from classification import classify_size
from configuration import CALIBRATION, SIZE_THRESHOLDS_MM


def test_small_pixel_detection():
    detection = {
        "detected": True,
        "bbox": (10, 10, 10, 12),
        "area": 100,
        "perimeter": 40.0
    }
    res = classify_size(detection)
    assert res["detected"] is True
    # Without calibration, small pixel area should map to SMALL
    assert res["size_label"] in ("SMALL", "UNKNOWN", "MEDIUM")
    assert 0.0 <= res["confidence"] <= 1.0


def test_medium_pixel_detection():
    detection = {
        "detected": True,
        "bbox": (10, 10, 40, 35),
        "area": 1200,
        "perimeter": 140.0
    }
    res = classify_size(detection)
    assert res["detected"] is True
    assert 0.0 <= res["confidence"] <= 1.0


def test_requires_min_area():
    detection = {
        "detected": True,
        "bbox": (0, 0, 2, 3),
        "area": 10
    }
    res = classify_size(detection)
    assert res["detected"] is False or res["size_label"] == "UNKNOWN"
