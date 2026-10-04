import math

import cv2
import numpy as np
import pytest

from navigation.banner_align import yaw_rate_for_offset
from navigation.red_zone_avoidance import adjust_velocity
from vision.banner_detector import BannerDetector
from vision.red_zone import RedZoneDetector, NearestRed
from config import params

W, H = 1280, 720
GREEN = (30, 160, 60)      # RGB
RED = (220, 30, 30)
ORANGE = (230, 120, 40)


def canvas():
    return np.full((H, W, 3), 100, np.uint8)


def rect(img, x0, y0, x1, y1, colour):
    img[y0:y1, x0:x1] = colour


def grass_speckle(img, seed=1, n=400):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        x, y = int(rng.integers(0, W - 12)), int(rng.integers(0, H - 12))
        w, h = int(rng.integers(3, 12)), int(rng.integers(3, 12))
        img[y:y + h, x:x + w] = (40, 130, 50)


# ── banner ───────────────────────────────────────────────────────────────────
def test_banner_found_with_text_gaps_and_offset_sign():
    img = canvas()
    rect(img, 800, 200, 1100, 450, GREEN)                 # centre x = 950 -> right of 640
    rect(img, 830, 280, 1070, 310, (240, 240, 240))       # white "text" stripe through it
    fix = BannerDetector().detect(img)
    assert fix is not None
    assert fix.cx == pytest.approx(950, abs=15)
    assert fix.offset_px > 250 and fix.area > 40000


def test_banner_left_gives_negative_offset():
    img = canvas()
    rect(img, 100, 200, 400, 450, GREEN)
    assert BannerDetector().detect(img).offset_px < -200


def test_nothing_green_means_no_banner():
    assert BannerDetector().detect(canvas()) is None


def test_ragged_grass_is_not_a_banner():
    img = canvas()
    grass_speckle(img)
    assert BannerDetector().detect(img) is None


def test_banner_still_found_among_grass():
    img = canvas()
    grass_speckle(img)
    rect(img, 300, 150, 620, 420, GREEN)
    fix = BannerDetector().detect(img)
    assert fix is not None and fix.cx == pytest.approx(460, abs=20)


def test_small_green_blob_ignored():
    img = canvas()
    rect(img, 600, 300, 640, 340, GREEN)                  # 1600 px² < min
    assert BannerDetector().detect(img) is None


def test_yaw_law_sign_and_limit():
    assert yaw_rate_for_offset(+100) > 0 > yaw_rate_for_offset(-100)
    assert yaw_rate_for_offset(10_000) == params.BANNER_YAW_MAX_DPS
    assert yaw_rate_for_offset(0) == 0


# ── red zone ─────────────────────────────────────────────────────────────────
def test_red_rectangle_detected():
    img = canvas()
    rect(img, 400, 200, 800, 500, RED)
    blobs = RedZoneDetector().detect(img)
    assert len(blobs) == 1 and blobs[0].area > 80_000


def test_red_checkerboard_qr_is_not_a_zone():
    img = canvas()
    for i in range(0, 300, 16):
        for j in range(0, 300, 16):
            if (i // 16 + j // 16) % 2 == 0:
                img[200 + i:200 + i + 16, 400 + j:400 + j + 16] = RED
    assert RedZoneDetector().detect(img) == []


def test_orange_is_not_red():
    img = canvas()
    rect(img, 400, 200, 800, 500, ORANGE)
    assert RedZoneDetector().detect(img) == []


def test_nearest_edge_straight_down_camera():
    img = canvas()
    rect(img, 540, 100, 740, 300, RED)                    # above image centre -> AHEAD
    n = RedZoneDetector().nearest(img, altitude_m=10.0, pitch_deg=90.0)
    assert n is not None and n.forward_m > 0 and abs(n.right_m) < 3
    # its nearest edge is the BOTTOM edge (y=300), 60 px above centre (y=360)
    fx, fy = (W / 2) / math.tan(math.radians(params.CAM_FOV_H_DEG / 2)), (H / 2) / math.tan(math.radians(params.CAM_FOV_V_DEG / 2))
    assert n.forward_m == pytest.approx(10.0 * 60 / fy, rel=0.15)


def test_no_red_means_none():
    assert RedZoneDetector().nearest(canvas(), 10.0) is None


# ── avoidance ────────────────────────────────────────────────────────────────
def near(fwd, right):
    return NearestRed(fwd, right, math.hypot(fwd, right))


def test_far_zone_changes_nothing():
    assert adjust_velocity(1.2, 0.0, near(20.0, 0.0)) == (1.2, 0.0)
    assert adjust_velocity(1.2, 0.0, None) == (1.2, 0.0)


def test_zone_dead_ahead_inside_stop_distance_blocks_forward_motion():
    vx, vy = adjust_velocity(1.2, 0.0, near(params.RED_STOP_M - 0.5, 0.0))
    assert vx <= 0.0                                       # not moving toward it any more


def test_zone_between_stop_and_influence_slows_us():
    vx, _ = adjust_velocity(1.2, 0.0, near(4.0, 0.0))
    assert 0 < vx < 1.2


def test_zone_to_the_side_does_not_stop_forward_flight():
    vx, vy = adjust_velocity(1.2, 0.0, near(0.0, 2.0))
    assert vx == pytest.approx(1.2, abs=1e-6) and vy < 0   # pushed gently away (left)


def test_push_is_capped():
    vx, vy = adjust_velocity(0.0, 0.0, near(0.2, 0.0))
    assert math.hypot(vx, vy) <= params.RED_REPULSION_MAX + 1e-9
    assert vx < 0


def test_rotated_red_zone_still_counts():
    img = canvas()
    pts = np.array([[640, 150], [900, 360], [640, 570], [380, 360]], np.int32)   # a diamond
    cv2.fillPoly(img, [pts], RED)
    assert len(RedZoneDetector().detect(img)) == 1
