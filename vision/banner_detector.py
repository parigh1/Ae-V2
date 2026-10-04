# =============================================================================
# vision/banner_detector.py — Team Vajra AeroTHON 2026
# Finds the green AeroTHON banner that marks the corridor entrance.
# Pipeline (report section 4.3): green mask -> close (fill the text) -> open
# (drop speckle) -> biggest blob that is big enough AND a solid rectangle.
# The solidity/fill checks are what keep ragged grass from being mistaken for it.
# =============================================================================
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2

from config.params import (
    CAM_IMG_W, CAM_IMG_H, BANNER_MIN_AREA, BANNER_PROC_WIDTH,
    BANNER_MORPH_CLOSE, BANNER_MORPH_OPEN, BANNER_MIN_SOLIDITY, BANNER_MIN_FILL,
)
from vision.color_masks import green_mask, odd_kernel


@dataclass
class BannerFix:
    cx: float                      # centre of the banner, pixels (original frame)
    cy: float
    area: float                    # pixels² (original frame scale)
    bbox: Tuple[int, int, int, int]
    offset_px: float               # cx - image centre: + means the banner is to the RIGHT


class BannerDetector:
    def detect(self, frame) -> Optional[BannerFix]:
        h0, w0 = frame.shape[:2]
        scale = 1.0
        small = frame
        if w0 > BANNER_PROC_WIDTH:
            scale = BANNER_PROC_WIDTH / w0
            small = cv2.resize(frame, (BANNER_PROC_WIDTH, int(round(h0 * scale))),
                               interpolation=cv2.INTER_AREA)
        h, w = small.shape[:2]

        mask = green_mask(small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, odd_kernel(BANNER_MORPH_CLOSE, w, CAM_IMG_W))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, odd_kernel(BANNER_MORPH_OPEN, w, CAM_IMG_W))

        min_area = BANNER_MIN_AREA * (w * h) / (CAM_IMG_W * CAM_IMG_H)
        best, best_area = None, 0.0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area or area <= best_area:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(c))
            x, y, bw, bh = cv2.boundingRect(c)
            if hull_area <= 0 or area / hull_area < BANNER_MIN_SOLIDITY:
                continue
            if area / float(bw * bh) < BANNER_MIN_FILL:
                continue
            best, best_area = c, area
        if best is None:
            return None

        m = cv2.moments(best)
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        x, y, bw, bh = cv2.boundingRect(best)
        inv = 1.0 / scale
        return BannerFix(cx * inv, cy * inv, best_area * inv * inv,
                         (int(x * inv), int(y * inv), int(bw * inv), int(bh * inv)),
                         cx * inv - w0 / 2.0)
