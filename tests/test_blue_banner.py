# tests/test_blue_banner.py — exit-corridor blue banner detection
# The official AeroTHON 2026 field properties PDF (page 11) confirms:
#   entry corridor banner: GREEN #00B140, 11.5 ft x 1.5 ft (arch) / 1 ft (ground)
#   exit  corridor banner: BLUE  #0000FF, same dimensions
import numpy as np
import pytest
import cv2
from vision.banner_detector import BannerDetector, BlueBannerDetector
from vision.color_masks import blue_mask, green_mask

W, H = 1280, 960


def solid_frame(r, g, b):
    f = np.zeros((H, W, 3), np.uint8); f[:] = (r, g, b); return f


def banner_frame(r, g, b, width_px=900, height_px=80):
    f = np.full((H, W, 3), 130, np.uint8)
    x0, x1 = (W - width_px) // 2, (W + width_px) // 2
    y0, y1 = (H - height_px) // 2, (H + height_px) // 2
    f[y0:y1, x0:x1] = (r, g, b); return f


# ── colour mask sanity ────────────────────────────────────────────────────────
def test_green_mask_detects_00B140():
    assert green_mask(solid_frame(0, 177, 64)).sum() > 0

def test_blue_mask_detects_0000FF():
    assert blue_mask(solid_frame(0, 0, 255)).sum() > 0

def test_green_mask_does_not_detect_blue():
    assert green_mask(solid_frame(0, 0, 255)).sum() == 0

def test_blue_mask_does_not_detect_green():
    assert blue_mask(solid_frame(0, 177, 64)).sum() == 0

def test_blue_mask_does_not_detect_red():
    assert blue_mask(solid_frame(255, 0, 0)).sum() == 0


# ── detector routing ─────────────────────────────────────────────────────────
def test_BannerDetector_finds_green_banner():
    assert BannerDetector().detect(banner_frame(0, 177, 64)) is not None

def test_BannerDetector_does_not_find_blue_banner():
    assert BannerDetector().detect(banner_frame(0, 0, 255)) is None

def test_BlueBannerDetector_finds_blue_banner():
    assert BlueBannerDetector().detect(banner_frame(0, 0, 255)) is not None

def test_BlueBannerDetector_does_not_find_green_banner():
    assert BlueBannerDetector().detect(banner_frame(0, 177, 64)) is None

def test_blue_banner_offset_sign():
    """Banner left of centre → negative offset."""
    f = np.full((H, W, 3), 130, np.uint8)
    f[400:480, 50:400] = (0, 0, 255)      # banner in the left third
    fix = BlueBannerDetector().detect(f)
    assert fix is not None and fix.offset_px < 0