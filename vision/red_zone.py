# =============================================================================
# vision/red_zone.py — Team Vajra AeroTHON 2026
# Finds red no-fly zones on the ground and says how far the NEAREST edge is from
# the drone (metres, body frame). Rulebook: each violation costs marks.
#
# No "close" step here on purpose: it would melt a red-and-white checkerboard
# (e.g. a red QR code) into a solid blob. Instead small speckle is opened away,
# and only big blobs that FILL their rectangle (solid, not checkered) count.
# =============================================================================
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from config.params import CAM_IMG_W, RED_MIN_AREA, RED_MORPH_OPEN, RED_MIN_SOLIDITY, RED_MIN_FILL
from vision.color_masks import red_mask, odd_kernel
from vision.pixel_to_meters import PixelToMeters

_MAX_POINTS = 60          # contour points kept per blob (enough to find the nearest edge)


@dataclass
class RedBlob:
    area: float
    cx: float
    cy: float
    bbox: Tuple[int, int, int, int]
    points: np.ndarray            # (N, 2) pixel coordinates along the outline


@dataclass
class NearestRed:
    forward_m: float              # where the nearest red edge is, relative to the drone
    right_m: float
    distance_m: float


class RedZoneDetector:
    def detect(self, frame) -> List[RedBlob]:
        h, w = frame.shape[:2]
        mask = red_mask(frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, odd_kernel(RED_MORPH_OPEN, w, CAM_IMG_W))
        min_area = RED_MIN_AREA * (w * h) / (1280 * 720)
        blobs = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(c))
            if hull_area <= 0 or area / hull_area < RED_MIN_SOLIDITY:
                continue
            # A solid zone fills (almost) its tightest rectangle, however it is rotated.
            # A red-and-white checkerboard (QR code) fills only about half of it.
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            x, y, bw, bh = cv2.boundingRect(c)
            roi = np.zeros((bh, bw), np.uint8)
            cv2.drawContours(roi, [c - [x, y]], -1, 255, -1)
            red_pixels = cv2.countNonZero(cv2.bitwise_and(roi, mask[y:y + bh, x:x + bw]))
            if rw * rh <= 0 or red_pixels / (rw * rh) < RED_MIN_FILL:
                continue
            m = cv2.moments(c)
            pts = c.reshape(-1, 2)
            step = max(1, len(pts) // _MAX_POINTS)
            blobs.append(RedBlob(area, m["m10"] / m["m00"], m["m01"] / m["m00"],
                                 cv2.boundingRect(c), pts[::step].astype(float)))
        return blobs

    def nearest(self, frame, altitude_m: float, pitch_deg: float = 90.0) -> Optional[NearestRed]:
        """Nearest red edge on the ground, or None if no red zone is visible."""
        h, w = frame.shape[:2]
        best: Optional[NearestRed] = None
        for blob in self.detect(frame):
            for (px, py) in blob.points:
                g = PixelToMeters.offset_body(px - w / 2.0, py - h / 2.0, altitude_m, w, h,
                                              pitch_deg=pitch_deg)
                if g is None:
                    continue
                d = math.hypot(*g)
                if best is None or d < best.distance_m:
                    best = NearestRed(g[0], g[1], d)
        return best
