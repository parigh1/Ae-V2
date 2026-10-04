# =============================================================================
# vision/pixel_to_meters.py — Team Vajra AeroTHON 2026
#
# Turns "this QR is N pixels from the image centre" into "this QR is X metres
# ahead and Y metres to the right of the drone, on the ground".
#
# CHANGES vs previous version
#   [NEW] pitch_deg: the camera may be TILTED (90 = straight down,
#         0 = straight forward). The pixel is turned into a ray and that ray is
#         intersected with the flat ground. At pitch 90 the result is identical
#         to the old formula.
#   [NEW] CAM_IMAGE_ROTATION_DEG replaces CAM_X_SIGN / CAM_Y_SIGN
#         (handles a camera bolted on sideways or upside-down).
#   [CHANGED] offset_body() returns None when the pixel looks at or above the
#         horizon (that ray never hits the ground).
#
# Assumptions: flat ground, drone roughly level, camera mounted on the drone's
# centre line, height above ground = rangefinder reading.
# =============================================================================

import math
from typing import Optional, Tuple

from config.params import (
    CAM_FOV_H_DEG, CAM_FOV_V_DEG, CAM_IMG_W, CAM_IMG_H,
    CAM_IMAGE_ROTATION_DEG,
)

_MIN_DOWN_COMPONENT = 0.05   # ray must point down at least ~3 deg below horizon


class PixelToMeters:
    CAMERA_FOV_H_DEG = CAM_FOV_H_DEG
    CAMERA_FOV_V_DEG = CAM_FOV_V_DEG
    IMG_W = CAM_IMG_W
    IMG_H = CAM_IMG_H

    # ── helpers ───────────────────────────────────────────────────────────────
    @classmethod
    def focal_lengths(cls, img_w=None, img_h=None) -> Tuple[float, float]:
        """Focal lengths in pixels (fx, fy) from the field of view."""
        img_w = img_w or cls.IMG_W
        img_h = img_h or cls.IMG_H
        fx = (img_w / 2.0) / math.tan(math.radians(cls.CAMERA_FOV_H_DEG) / 2)
        fy = (img_h / 2.0) / math.tan(math.radians(cls.CAMERA_FOV_V_DEG) / 2)
        return fx, fy

    @staticmethod
    def _unrotate(dx, dy, rotation_deg):
        """Undo a camera bolted on rotated clockwise by rotation_deg, so that
        the result behaves as if 'top of image = drone nose'."""
        a = math.radians(rotation_deg)
        c, s = round(math.cos(a), 12), round(math.sin(a), 12)
        return dx * c - dy * s, dx * s + dy * c

    # ── nadir-only helpers (kept for old callers / tests) ─────────────────────
    @classmethod
    def metres_per_pixel(cls, altitude_m, img_w=None, img_h=None):
        fx, fy = cls.focal_lengths(img_w, img_h)
        return altitude_m / fx, altitude_m / fy

    @classmethod
    def offset_at_altitude(cls, dx_px, dy_px, altitude_m, img_w=None, img_h=None):
        """Straight-down camera only. Image axes: +x right, +y DOWN."""
        mx, my = cls.metres_per_pixel(altitude_m, img_w, img_h)
        return dx_px * mx, dy_px * my

    # ── the one flight code should use ────────────────────────────────────────
    @classmethod
    def offset_body(cls, dx_px, dy_px, altitude_m, img_w=None, img_h=None,
                    pitch_deg: float = 90.0,
                    rotation_deg: Optional[float] = None
                    ) -> Optional[Tuple[float, float]]:
        """
        Where is the thing at pixel offset (dx_px right, dy_px down) from the
        image centre, relative to the point on the ground directly under the
        drone?   Returns (forward_m, right_m)   (+ = ahead / to the right),
        or None if that pixel is looking at/above the horizon.
        """
        if rotation_deg is None:
            rotation_deg = CAM_IMAGE_ROTATION_DEG
        dx, dy = cls._unrotate(dx_px, dy_px, rotation_deg)

        fx, fy = cls.focal_lengths(img_w, img_h)
        u, v = dx / fx, dy / fy                 # tangent of the ray angle

        th = math.radians(pitch_deg)
        ray_forward = math.cos(th) - math.sin(th) * v
        ray_right = u
        ray_down = math.sin(th) + math.cos(th) * v
        if ray_down < _MIN_DOWN_COMPONENT:
            return None

        t = altitude_m / ray_down               # distance along ray to the ground
        return t * ray_forward, t * ray_right
