# vision/corridor_detector.py — camera-only corridor perception.
# Every line segment is projected onto the GROUND (using camera tilt + height), so the
# answer comes out in metres:
#   center_offset_m    + : the corridor centre is to the RIGHT of the drone
#   heading_error_deg  + : the corridor points to the RIGHT of the nose (turn clockwise)
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from config.params import (
    CORRIDOR_WIDTH, CORRIDOR_PROC_WIDTH, CORRIDOR_CANNY_LOW, CORRIDOR_CANNY_HIGH,
    CORRIDOR_HOUGH_THRESH, CORRIDOR_MIN_LINE_PX, CORRIDOR_MAX_GAP_PX,
    CORRIDOR_MIN_GROUND_LEN_M, CORRIDOR_MAX_LINE_ANGLE_DEG,
    CORRIDOR_MIN_WALL_M, CORRIDOR_MAX_WALL_M, CORRIDOR_CLUSTER_M, CORRIDOR_MIN_SIDE_LENGTH_M,
    CORRIDOR_NEAR_PREFERENCE,
)
from vision.pixel_to_meters import PixelToMeters

Segment = Tuple[float, float, float, float]          # x1, y1, x2, y2 in ORIGINAL pixels


@dataclass
class CorridorFix:
    center_offset_m: float
    heading_error_deg: float
    left_m: Optional[float]                           # distance to the left wall (m), if seen
    right_m: Optional[float]
    walls: str                                        # "both" | "left" | "right"
    confidence: float                                 # 0..1


@dataclass
class _DebugInfo:
    left: List[Segment] = field(default_factory=list)
    right: List[Segment] = field(default_factory=list)


def _best_cluster(items):
    """items: (distance, phi, length, segment) for ONE side. Lines that agree on distance
    form a group. Of the groups with real support, the NEAREST is the wall base: the top
    edge of a wall shorter than the camera projects farther away than its base."""
    groups = []
    for d0, _, _, _ in items:
        g = [it for it in items if abs(it[0] - d0) <= CORRIDOR_CLUSTER_M]
        total = sum(it[2] for it in g)
        d = sum(it[0] * it[2] for it in g) / total
        phi = sum(it[1] * it[2] for it in g) / total
        groups.append((d, phi, total, [it[3] for it in g]))
    strongest = max(grp[2] for grp in groups)
    supported = [grp for grp in groups
                 if grp[2] >= max(CORRIDOR_MIN_SIDE_LENGTH_M, CORRIDOR_NEAR_PREFERENCE * strongest)]
    return min(supported, key=lambda grp: grp[0]) if supported else max(groups, key=lambda grp: grp[2])


class CorridorDetector:
    def __init__(self):
        self.debug = _DebugInfo()

    def detect(self, frame, altitude_m: float, pitch_deg: float) -> Optional[CorridorFix]:
        h0, w0 = frame.shape[:2]
        scale = min(1.0, CORRIDOR_PROC_WIDTH / w0)
        small = frame if scale >= 1.0 else cv2.resize(
            frame, (int(round(w0 * scale)), int(round(h0 * scale))), interpolation=cv2.INTER_AREA)
        pw = small.shape[1]

        # Canny runs on each colour channel: a wall with the same BRIGHTNESS as the grass
        # but a different COLOUR is invisible to a grey-scale Canny.
        blurred = cv2.GaussianBlur(small, (5, 5), 0)
        edges = np.zeros(blurred.shape[:2], np.uint8)
        for ch in range(3):
            edges |= cv2.Canny(blurred[:, :, ch], CORRIDOR_CANNY_LOW, CORRIDOR_CANNY_HIGH)
        k = pw / 640.0
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, CORRIDOR_HOUGH_THRESH,
                                minLineLength=CORRIDOR_MIN_LINE_PX * k,
                                maxLineGap=CORRIDOR_MAX_GAP_PX * k)
        self.debug = _DebugInfo()
        if lines is None:
            return None

        left, right = [], []
        inv = 1.0 / scale
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            seg = (x1 * inv, y1 * inv, x2 * inv, y2 * inv)
            g1 = PixelToMeters.offset_body(seg[0] - w0 / 2, seg[1] - h0 / 2, altitude_m, w0, h0, pitch_deg)
            g2 = PixelToMeters.offset_body(seg[2] - w0 / 2, seg[3] - h0 / 2, altitude_m, w0, h0, pitch_deg)
            if g1 is None or g2 is None:
                continue
            df, dr = g2[0] - g1[0], g2[1] - g1[1]
            length = math.hypot(df, dr)
            if length < CORRIDOR_MIN_GROUND_LEN_M:
                continue
            if df < 0:                                  # direction of travel doesn't matter
                df, dr = -df, -dr
            phi = math.degrees(math.atan2(dr, df))      # + = line heads to the right as it goes ahead
            if abs(phi) > CORRIDOR_MAX_LINE_ANGLE_DEG:
                continue
            lateral0 = g1[1] - math.tan(math.radians(phi)) * g1[0]     # where it crosses "level with us"
            dist = abs(lateral0) * math.cos(math.radians(phi))          # perpendicular distance
            if not (CORRIDOR_MIN_WALL_M <= dist <= CORRIDOR_MAX_WALL_M):
                continue
            (left if lateral0 < 0 else right).append((dist, phi, length, seg))

        L = _best_cluster(left) if left else None
        R = _best_cluster(right) if right else None
        if L and L[2] < CORRIDOR_MIN_SIDE_LENGTH_M:
            L = None
        if R and R[2] < CORRIDOR_MIN_SIDE_LENGTH_M:
            R = None
        if L is None and R is None:
            return None

        # Two walls that are not about one corridor-width apart cannot both be walls:
        # keep the better-supported one.
        if L and R and abs((L[0] + R[0]) - CORRIDOR_WIDTH) > 0.35 * CORRIDOR_WIDTH:
            if L[2] >= R[2]:
                R = None
            else:
                L = None

        if L:
            self.debug.left = L[3]
        if R:
            self.debug.right = R[3]

        if L and R:
            width_err = abs((L[0] + R[0]) - CORRIDOR_WIDTH) / CORRIDOR_WIDTH
            return CorridorFix((R[0] - L[0]) / 2.0,
                               (L[1] * L[2] + R[1] * R[2]) / (L[2] + R[2]),
                               L[0], R[0], "both", max(0.0, 1.0 - width_err))
        if L:
            return CorridorFix(CORRIDOR_WIDTH / 2.0 - L[0], L[1], L[0], None, "left", 0.6)
        return CorridorFix(R[0] - CORRIDOR_WIDTH / 2.0, R[1], None, R[0], "right", 0.6)