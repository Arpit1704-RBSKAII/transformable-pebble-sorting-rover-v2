"""
configuration.py
-----------------
Centralized Python-side configuration - mirrors the same "keep every
constant in one place" principle as esp32/include/configuration.h.

This file was missing entirely, even though both communication.py and
the Phase 6 main.py already imported it (same gap pattern as the ESP32
side: modules were written assuming config values that were never
actually defined).
"""

# ============================================================
# SERIAL CONNECTION — TODO: CONFIGURE AFTER HARDWARE SELECTION
# ============================================================
# Find your ESP32's actual port once it's plugged in:
#   Windows : Device Manager -> Ports (COM & LPT) -> look for "Silicon
#             Labs CP210x" or "CH340", note the COM number
#   macOS   : run `ls /dev/cu.*` in Terminal, look for
#             /dev/cu.usbserial-XXXX or /dev/cu.SLAB_USBtoUART
#   Linux   : run `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` in a terminal
#
# You can also call communication.list_serial_ports() from a Python
# shell to print every port the OS currently sees.
SERIAL_PORT = "COM3"  # TODO: CONFIGURE AFTER HARDWARE SELECTION

# Must match SERIAL_BAUD_RATE in esp32/include/configuration.h exactly.
SERIAL_BAUD_RATE = 115200

# ============================================================
# TIMEOUTS
# ============================================================
# How long to wait for an IMMEDIATE response (OK/BUSY/ERROR/STATUS)
# to a single command sent to the ESP32.
COMMAND_RESPONSE_TIMEOUT_SECONDS = 2.0

# How long to wait for the "READY" line after opening the serial port.
# Needs to comfortably cover ESP32 boot time (motorsInit/servosInit/
# gripperInit/armInit/safetyInit all run before commInit() prints READY).
CONNECT_TIMEOUT_SECONDS = 5.0

# How long to wait for a "DONE,<label>" line after a critical action
# (ARM,*/GRIP,*/SORT,*). Should be noticeably longer than the ESP32's
# own ARM_MOVE_TIMEOUT_MS / GRIPPER_MOVE_TIMEOUT_MS (both in
# esp32/include/configuration.h) so Python doesn't give up right as
# the ESP32 is about to finish.
DONE_TIMEOUT_SECONDS = 6.0

# ============================================================
# CAMERA — TODO: CONFIGURE AFTER HARDWARE SELECTION
# ============================================================
# 0 is usually the first/only USB camera on a machine with no built-in
# webcam. If you have a laptop webcam AND a USB camera, 0 may grab the
# wrong one - the Phase 7 test script will show you what it opened.
CAMERA_INDEX = 0  # TODO: CONFIGURE AFTER HARDWARE SELECTION

# Requested resolution. Not guaranteed - some cameras/drivers ignore
# this and provide their own native resolution instead. camera.py logs
# a warning and uses the ACTUAL resolution if they differ.
CAMERA_FRAME_WIDTH = 640   # TODO: CALIBRATE ON HARDWARE
CAMERA_FRAME_HEIGHT = 480  # TODO: CALIBRATE ON HARDWARE

# ============================================================
# PEBBLE DETECTION — TODO: CALIBRATE ON HARDWARE
# ============================================================
# test_detection.py already existed in the repo (as unit tests against
# synthetic frames) and fully specifies detect_pebbles()'s expected
# behavior, but detection.py itself - and every constant below it
# depends on - did not exist anywhere. Values here are reasonable
# starting points confirmed against those tests, not calibrated
# against your actual camera/lighting/pebbles yet.

DETECTION_BLUR_KERNEL = 5  # must be odd; larger = smoother but less precise edges

# TODO: CALIBRATE ON HARDWARE - depends heavily on your lighting and
# background color. Use test_pebble_detection_live.py to tune this by
# watching detected blobs update in real time.
DETECTION_THRESHOLD_VALUE = 100

# TODO: CALIBRATE ON HARDWARE - True if pebbles appear DARKER than the
# background under your lighting, False if pebbles appear LIGHTER.
DETECTION_INVERT_THRESHOLD = False

DETECTION_MORPH_KERNEL = 5  # cleans up small noise / fills small gaps in blobs

# TODO: CALIBRATE ON HARDWARE - filters out noise specks. Depends on
# camera resolution and distance from camera to pebbles - a pebble
# that's 200px² at close range might only be 30px² further away.
MIN_CONTOUR_AREA_PX = 200

# 1.0 = perfect circle. Rejects elongated smears/shadows that pass the
# area filter but clearly aren't round pebbles.
DETECTION_MIN_CIRCULARITY = 0.75

# (x0, y0, x1, y1) as FRACTIONS of frame width/height (0.0-1.0), not
# pixels - stays valid across different camera resolutions. Restrict
# this if only part of the frame is a valid pickup/sorting area, e.g.
# (0.2, 0.3, 0.8, 0.9) to ignore the frame edges. Full frame by default.
DETECTION_ZONE_FRACTION = (0.0, 0.0, 1.0, 1.0)

# ============================================================
# SIZE CLASSIFICATION — TODO: CALIBRATE ON HARDWARE
# ============================================================
# Reused from the repo's existing "Phase 9 Size detection configuration"
# file, which already had sensible structure and placeholder values.

# Calibration mapping mode:
#   'pixel_to_mm'      - linear scale for diameter (mm per pixel)
#   'area_pixel_to_mm2' - area-based mapping (mm^2 per pixel)
# Choose depending on which calibration method you actually perform
# with calibration.py.
CALIBRATION = {
    "mode": "pixel_to_mm",  # or "area_pixel_to_mm2"

    # TODO: CALIBRATE ON HARDWARE - measure with a known-size reference
    # object using calibration.py and replace this placeholder.
    "pixel_to_mm": 0.5,

    # Alternative mapping, only used if mode is "area_pixel_to_mm2".
    "area_pixel_to_mm2": 0.25,

    # False until calibration.py has actually been run - classify_size()
    # falls back to FALLBACK_PIXEL_AREA_THRESHOLDS (below) while this is
    # False, with a lower confidence multiplier, rather than trusting
    # made-up mm values.
    "is_calibrated": False,
}

# Real-world size thresholds in millimetres. Format: (min_inclusive_mm, max_exclusive_mm)
# TODO: CALIBRATE ON HARDWARE - measure your actual pebbles and adjust.
SIZE_THRESHOLDS_MM = {
    "SMALL":  (0.0, 10.0),   # 0-10 mm
    "MEDIUM": (10.0, 25.0),  # 10-25 mm
    "LARGE":  (25.0, 999.0), # 25+ mm
}

# Used ONLY when CALIBRATION["is_calibrated"] is False - very approximate
# pixel-area-based fallback so classification still produces a usable
# (lower-confidence) label before real calibration is done.
# TODO: CALIBRATE ON HARDWARE
FALLBACK_PIXEL_AREA_THRESHOLDS = {
    "SMALL":  (0, 500),
    "MEDIUM": (500, 2500),
    "LARGE":  (2500, 9999999),
}

MIN_CONTOUR_AREA_PIXELS = 50  # below this, a detection isn't classified at all
ROUNDNESS_BONUS = 0.1         # confidence boost weight for circular contours

# ============================================================
# COLOUR CLASSIFICATION — TODO: CALIBRATE ON HARDWARE
# ============================================================
# Reused from the repo's existing colour detection configuration file.
# Colour classification is OPTIONAL per project spec - only wire this
# into decision.py if actually needed for sorting logic.

# Example colour references (BGR tuples) - initial guesses only.
# TODO: CALIBRATE ON HARDWARE - sample real pebbles with your camera
# and replace these with their actual mean BGR values. The classifier
# converts these to Lab internally for perceptually-meaningful comparison.
COLOR_REFERENCES_BGR = {
    "BROWN":  (42, 42, 165),
    "GREY":   (128, 128, 128),
    "BLACK":  (20, 20, 20),
    "WHITE":  (230, 230, 230),
    "RED":    (45, 30, 200),
    "YELLOW": (40, 220, 220),
    "GREEN":  (60, 150, 60),
    "BLUE":   (200, 60, 40),
}

# Maximum meaningful Lab distance, used to normalize confidence. If the
# average distance exceeds this, confidence will be very low.
COLOR_MAX_DISTANCE = 60.0

# Minimum mask pixel count required to attempt colour classification.
MIN_COLOR_MASK_PIXELS = 20

# Confidence floor when there's insufficient data to classify.
COLOR_CONFIDENCE_FLOOR = 0.0

# ============================================================
# DECISION STATE MACHINE — Phase 11
# ============================================================
# Reused from the repo's existing decision state machine configuration
# file, with ONE value changed - see PICK_CONFIDENCE_THRESHOLD below.

SIZE_CONFIDENCE_WEIGHT = 0.6   # weight of size confidence in combined score
COLOR_CONFIDENCE_WEIGHT = 0.4  # weight of colour confidence in combined score (only applied when colour was actually evaluated - see decision.py)

# TODO: CALIBRATE ON HARDWARE - the repo's original value here was 0.7,
# but classify_size()'s FALLBACK (uncalibrated) confidence path can
# never exceed ~0.42 (0.5 baseline + 0.1 area bonus + up to 0.1
# roundness bonus, all multiplied by the 0.6 fallback-mode penalty from
# classification.py). A 0.7 threshold would make it mathematically
# IMPOSSIBLE to ever pick anything until calibration.py has been run.
# Lowered to 0.35 so the system is actually testable/usable in
# fallback mode - raise this back up once CALIBRATION["is_calibrated"]
# is True in practice, since calibrated confidences run higher.
PICK_CONFIDENCE_THRESHOLD = 0.35

# Timeouts (seconds)
COMM_COMMAND_TIMEOUT = 8.0  # wait for DONE for a command
ARM_MOVE_TIMEOUT = 8.0
PICK_TIMEOUT = 10.0
MOVEMENT_TIMEOUT = 10.0

# Operational safety
ALLOW_PICK_WHILE_MOVING = False  # do not pick while driving
MIN_CONFIDENCE_TO_CLASSIFY = 0.4  # below this: mark classification as uncertain (not currently consumed by decision.py - reserved for future use)

# Retry counts
MAX_PICK_RETRIES = 2

# Logging level: 0=errors only, 1=info, 2=debug
LOG_VERBOSITY = 1

# ============================================================
# LOGGING
# ============================================================
# One of: "DEBUG", "INFO", "WARNING", "ERROR"
LOG_LEVEL = "INFO"
