"""
camera.py
---------
Thin wrapper around OpenCV's VideoCapture, providing a single place
that owns the camera device. Detection/classification (Phase 8/9) will
call read_frame() without knowing or caring how the frame was captured
- a future camera swap (USB webcam -> Pi Camera -> IP cam) only means
changing this file.

Built fresh for Phase 7 (no existing camera code found in the repo to
reuse - the only Phase 7 material was a zip file this tool couldn't
inspect; built following the same config-driven pattern as every
other module so far).

Dependency: pip install opencv-python numpy
"""

import logging
from typing import Optional

import cv2
import numpy as np

import configuration

logger = logging.getLogger("camera")


class CameraError(Exception):
    """Raised when the camera cannot be opened or a frame cannot be read."""


class Camera:
    """
    Usage:
        with Camera() as cam:
            frame = cam.read_frame()
            ...

        # or without the context manager:
        cam = Camera()
        cam.open()
        frame = cam.read_frame()
        cam.release()
    """

    def __init__(
        self,
        index: int = configuration.CAMERA_INDEX,
        width: int = configuration.CAMERA_FRAME_WIDTH,
        height: int = configuration.CAMERA_FRAME_HEIGHT,
    ):
        self.index = index
        self.width = width
        self.height = height
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self):
        logger.info("Opening camera index %d...", self.index)
        self._cap = cv2.VideoCapture(self.index)

        if not self._cap.isOpened():
            raise CameraError(
                f"Could not open camera at index {self.index}. Check "
                f"CAMERA_INDEX in configuration.py, and make sure no other "
                f"program (Zoom, another script, etc.) is already using it."
            )

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # Some cameras/drivers silently ignore the requested resolution -
        # read back what was ACTUALLY set rather than assuming it worked.
        # Detection/calibration in later phases must use the ACTUAL
        # resolution, not the requested one, or pixel-based calibration
        # will be wrong.
        actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_width, actual_height) != (self.width, self.height):
            logger.warning(
                "Requested %dx%d but camera actually provided %dx%d. "
                "Using the ACTUAL resolution from here on.",
                self.width, self.height, actual_width, actual_height,
            )
        self.width = actual_width
        self.height = actual_height

        logger.info("Camera opened at %dx%d.", self.width, self.height)

    def is_open(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    def read_frame(self) -> np.ndarray:
        """
        Reads a single frame as a BGR numpy array (OpenCV's default).
        Raises CameraError if the read fails - callers should NOT
        assume a frame is always available (a loose USB cable, camera
        unplugged mid-run, etc. all show up here rather than as a
        silent bad frame).
        """
        if not self.is_open():
            raise CameraError("Camera is not open - call open() first.")

        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise CameraError("Failed to read a frame from the camera.")
        return frame

    def release(self):
        if self._cap is not None:
            self._cap.release()
            logger.info("Camera released.")
        self._cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
