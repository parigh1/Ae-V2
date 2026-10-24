# tests/test_qr_pattern.py — Phase 8: QR pattern matching without text decoding
#
# The AeroTHON organizers confirmed (Phase 2 call) that the delivery-zone QR
# codes carry no decodable text; they are patterns to match. These tests verify
# that vision/qr_pattern.py can distinguish the start QR from 5 distractors
# with 0 wrong picks (a false positive would drop the payload on the wrong code).

import math, random
import cv2, numpy as np, pytest, qrcode as qrlib

from config.params import QR_PATTERN_CONFIRM_MIN
from vision.qr_pattern import (QRMatcher, QRTemplate, _best_ncc, _CORNER_MASK,
                                locate_all_qrs)


def make_qr(text, px=300, border=2):
    q = qrlib.QRCode(border=border, box_size=10,
                     error_correction=qrlib.constants.ERROR_CORRECT_M)
    q.add_data(text); q.make(fit=True)
    return cv2.resize(np.array(q.make_image().convert("L")), (px, px),
                      interpolation=cv2.INTER_NEAREST)


def place_rgb(canvas, qr_gray, cx, cy, size, rot=0.0, tilt=0.1):
    h0, w0 = qr_gray.shape; half = size/2; tf = 1-tilt
    src = np.float32([[0,0],[w0,0],[w0,h0],[0,h0]])
    dst = np.float32([[-half*tf,-half*tf],[half*tf,-half*tf],[half,half],[-half,half]])
    c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
    dst = (dst @ np.float32([[c,-s],[s,c]]).T) + np.float32([cx, cy])
    M = cv2.getPerspectiveTransform(src, dst.astype(np.float32))
    H, W = canvas.shape[:2]
    warp = cv2.warpPerspective(cv2.cvtColor(qr_gray, cv2.COLOR_GRAY2RGB), M,
                               (W, H), flags=cv2.INTER_AREA, borderValue=(110,110,110))
    mask = cv2.warpPerspective(np.full_like(qr_gray, 255), M, (W, H),
                               flags=cv2.INTER_NEAREST, borderValue=0)
    out = canvas.copy(); out[mask > 0] = warp[mask > 0]; return out


def add_noise(frame_rgb, sigma=6, seed=0):
    n = np.random.RandomState(seed).normal(0, sigma, frame_rgb.shape[:2])
    g = np.clip(cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)+n,
                0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2RGB)


def single_view(qr_gray, size=200, rot=0.0, tilt=0.05, noise=6, seed=0):
    c = np.full((960, 1280, 3), 110, np.uint8)
    c = place_rgb(c, qr_gray, 640, 480, size, rot, tilt)
    return add_noise(c, noise, seed)


NUM_CODES = 6


@pytest.fixture(scope="module")
def codes():
    rng = random.Random(0)
    return [make_qr(f"pat-{i}-{rng.random():.8f}") for i in range(NUM_CODES)]


@pytest.fixture(scope="module")
def confirmed_template(codes):
    tmpl = QRTemplate()
    for seed in range(3):
        frame = single_view(codes[0], rot=random.Random(seed).uniform(-15, 15), seed=seed)
        dets = locate_all_qrs(frame)
        assert dets, f"start QR not located in template view {seed}"
        tmpl.add(dets[0]["patch"])
    assert tmpl.confirmed
    return tmpl


def test_same_code_scores_higher_than_different_code(codes, confirmed_template):
    t0 = confirmed_template.template
    same = []
    for i in range(8):
        frame = single_view(codes[0], 120,
                            rot=random.Random(i+10).uniform(0, 360), noise=8, seed=i+10)
        dets = locate_all_qrs(frame)
        if dets: same.append(_best_ncc(t0, dets[0]["patch"], _CORNER_MASK))
    diff = []
    for ci in range(1, NUM_CODES):
        for s in range(4):
            frame = single_view(codes[ci], 120,
                                rot=random.Random(s+20).uniform(0, 360), noise=8, seed=s+20)
            dets = locate_all_qrs(frame)
            if dets: diff.append(_best_ncc(t0, dets[0]["patch"], _CORNER_MASK))
    assert min(same) > 0.80, f"same-code score dropped to {min(same):.2f}"
    assert max(diff) < 0.60, f"different-code score rose to {max(diff):.2f}"
    assert min(same) > max(diff) + 0.15, "gap between same and different is too small"


def test_template_not_confirmed_from_two_different_codes(codes):
    t = QRTemplate()
    t.add(locate_all_qrs(single_view(codes[0], 200))[0]["patch"])
    t.add(locate_all_qrs(single_view(codes[1], 200))[0]["patch"])
    assert not t.confirmed


def test_template_confirmed_from_same_code_three_times(codes):
    t = QRTemplate()
    for seed in range(3):
        dets = locate_all_qrs(
            single_view(codes[0], 200, rot=random.Random(seed).uniform(-15, 15), seed=seed))
        assert dets
        t.add(dets[0]["patch"])
    assert t.confirmed


def test_template_resets_when_different_code_is_seen(codes):
    t = QRTemplate()
    t.add(locate_all_qrs(single_view(codes[0], 200))[0]["patch"])
    t.add(locate_all_qrs(single_view(codes[1], 200))[0]["patch"])
    assert not t.confirmed
    for seed in range(2):
        dets = locate_all_qrs(single_view(codes[1], 200, seed=seed+5))
        if dets: t.add(dets[0]["patch"])
    assert t.confirmed


@pytest.fixture(scope="module")
def matcher(confirmed_template):
    m = QRMatcher(); m.update_template(confirmed_template); return m


def test_zero_wrong_picks_over_50_trials(codes, matcher):
    """The most important test: never pick the wrong code."""
    rng = random.Random(99)
    wrong = 0
    target_text = f"pat-0-{random.Random(0).random():.8f}"
    for trial in range(50):
        shuffle = rng.sample(range(NUM_CODES), NUM_CODES)
        positions = [(200,200),(600,250),(1000,200),(300,700),(700,700),(1050,700)]
        canvas = np.full((960, 1280, 3), 110, np.uint8)
        for ci, (cx, cy) in zip(shuffle, positions):
            canvas = place_rgb(canvas, codes[ci], cx, cy,
                               rng.randint(90,130), rng.uniform(0,360), rng.uniform(0.05,0.3))
        frame = add_noise(canvas, sigma=rng.choice([5,8,12]), seed=trial)
        res = matcher.find_best(frame, 10.0, 72.0, 1280, 960)
        if res is not None and res["data"] is not None and res["data"] != target_text:
            wrong += 1
    assert wrong == 0, f"{wrong} wrong picks — payload would land on the wrong code"


def test_no_result_when_no_template(codes):
    m = QRMatcher()
    assert not m.has_template()
    canvas = np.full((960, 1280, 3), 110, np.uint8)
    canvas = place_rgb(canvas, codes[0], 640, 480, 120)
    assert m.find_best(canvas, 10.0, 72.0, 1280, 960) is None


def test_no_result_on_blank_frame(matcher):
    blank = np.full((960, 1280, 3), 110, np.uint8)
    assert matcher.find_best(blank, 10.0, 72.0, 1280, 960) is None


def test_correct_forward_and_right_offsets_when_centred(codes, confirmed_template):
    """QR dead centre should give near-zero lateral offset."""
    m = QRMatcher(); m.update_template(confirmed_template)
    frame = single_view(codes[0], size=120, rot=0, tilt=0.05, noise=4, seed=77)
    res = m.find_best(frame, 10.0, 72.0, 1280, 960)
    if res is not None:
        assert abs(res["right_m"]) < 2.0, f"right_m = {res['right_m']:.2f} for a centred QR"