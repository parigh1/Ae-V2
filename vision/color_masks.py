# =============================================================================
# vision/color_masks.py — Team Vajra AeroTHON 2026
# Turns a picture into a black/white mask of "pixels that are green / red".
# HSV (hue-saturation-value) is used because hue barely changes with sunlight.
# All thresholds live in config/params.py; scripts/tune_hsv.py helps pick them.
# Frames are RGB (see hardware/camera.py).
# =============================================================================
import cv2
import numpy as np

from config.params import (
    BANNER_H_LOW, BANNER_H_HIGH, BANNER_S_LOW, BANNER_S_HIGH, BANNER_V_LOW, BANNER_V_HIGH,
    RED_H_LOW1, RED_H_HIGH1, RED_H_LOW2, RED_H_HIGH2,
    RED_S_LOW, RED_S_HIGH, RED_V_LOW, RED_V_HIGH,
)


def hsv_range_mask(rgb, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi) -> np.ndarray:
    """255 where the pixel's HSV lies inside the box, else 0. (OpenCV hue is 0-179.)"""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    return cv2.inRange(hsv, (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi))


def green_mask(rgb) -> np.ndarray:
    return hsv_range_mask(rgb, BANNER_H_LOW, BANNER_H_HIGH, BANNER_S_LOW, BANNER_S_HIGH,
                          BANNER_V_LOW, BANNER_V_HIGH)


def red_mask(rgb) -> np.ndarray:
    """Red sits at BOTH ends of the hue scale, so two ranges are joined."""
    low = hsv_range_mask(rgb, RED_H_LOW1, RED_H_HIGH1, RED_S_LOW, RED_S_HIGH, RED_V_LOW, RED_V_HIGH)
    high = hsv_range_mask(rgb, RED_H_LOW2, RED_H_HIGH2, RED_S_LOW, RED_S_HIGH, RED_V_LOW, RED_V_HIGH)
    return cv2.bitwise_or(low, high)


def odd_kernel(size_px_at_full_width: float, img_w: int, full_w: int) -> np.ndarray:
    """Square structuring element scaled to the image width, always odd and >= 3."""
    k = max(3, int(round(size_px_at_full_width * img_w / full_w)))
    k += 1 - k % 2
    return np.ones((k, k), np.uint8)
