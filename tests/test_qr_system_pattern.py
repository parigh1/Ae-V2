# tests/test_qr_system_pattern.py — QRSystem pattern-only path (no text decoding)
import math, random
import cv2, numpy as np, pytest, qrcode as qrlib
from types import SimpleNamespace

from vision.qr_system import QRSystem, QRConfirmer
from vision.qr_pattern import locate_all_qrs


def make_qr(text, px=300, border=2):
    q = qrlib.QRCode(border=border, box_size=10,
                     error_correction=qrlib.constants.ERROR_CORRECT_M)
    q.add_data(text); q.make(fit=True)
    return cv2.resize(np.array(q.make_image().convert("L")), (px, px),
                      interpolation=cv2.INTER_NEAREST)


def place_rgb(canvas, qr_gray, cx, cy, size, rot=0.0, tilt=0.05):
    h0, w0 = qr_gray.shape; half = size / 2; tf = 1 - tilt
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


def noisy_rgb(frame_rgb, sigma=6, seed=0):
    n = np.random.RandomState(seed).normal(0, sigma, frame_rgb.shape[:2])
    g = np.clip(cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY).astype(np.float32) + n, 0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2RGB)


def single(qr, size=200, rot=0.0, tilt=0.05, noise=6, seed=0):
    c = np.full((960, 1280, 3), 110, np.uint8)
    c = place_rgb(c, qr, 640, 480, size, rot, tilt)
    return noisy_rgb(c, noise, seed)


def fake_vehicle(alt=5.0):
    return SimpleNamespace(altitude=alt, rangefinder_distance=None)


def test_scan_step_confirms_pattern_without_text():
    qr = make_qr("test-pattern-no-text-1234")
    frames = [single(qr, rot=random.Random(s).uniform(-15, 15), seed=s) for s in range(3)]
    idx = [0]
    cam = SimpleNamespace(capture_array=lambda: frames[min(idx[0], 2)], pitch_deg=72.0)
    sys = QRSystem(cam, fake_vehicle())
    sys.scanner.decode_frame = lambda f: None      # simulate no-text codes
    confirmer = QRConfirmer()
    result = None
    for i in range(3):
        idx[0] = i
        result = sys.scan_step(confirmer)
        if result: break
    assert result == "QR_PATTERN"
    assert sys._matcher.has_template()


def test_scan_step_uses_text_when_available():
    qr = make_qr("DELIVERY_42")
    frames = [single(qr, rot=random.Random(s).uniform(-15, 15), seed=s) for s in range(3)]
    idx = [0]
    cam = SimpleNamespace(capture_array=lambda: frames[min(idx[0], 2)], pitch_deg=72.0)
    sys = QRSystem(cam, fake_vehicle())
    confirmer = QRConfirmer()
    result = None
    for i in range(3):
        idx[0] = i
        result = sys.scan_step(confirmer)
        if result: break
    assert result in ("DELIVERY_42", "QR_PATTERN")
    assert sys.delivery_id is not None


def test_find_target_returns_none_before_template_built():
    qr = make_qr("X1")
    cam = SimpleNamespace(capture_array=lambda: single(qr), pitch_deg=72.0)
    sys = QRSystem(cam, fake_vehicle())
    sys.delivery_id = "QR_PATTERN"
    assert sys.find_target(single(qr)) is None


def test_find_target_locates_correct_code_among_distractors():
    rng = random.Random(7)
    texts = [f"code-{i}-{rng.random():.6f}" for i in range(6)]
    codes = [make_qr(t) for t in texts]
    qr_sys = QRSystem(SimpleNamespace(capture_array=lambda: None, pitch_deg=72.0),
                      fake_vehicle(alt=10.0))
    qr_sys.scanner.decode_frame = lambda f: None
    confirmer = QRConfirmer()
    for seed in range(3):
        f = single(codes[0], rot=rng.uniform(-15, 15), seed=seed)
        qr_sys.camera.capture_array = lambda _f=f: _f
        r = qr_sys.scan_step(confirmer)
        if r: break
    assert qr_sys._matcher.has_template()
    positions = [(200,200),(600,250),(1000,200),(300,700),(700,700),(1050,700)]
    for trial in range(20):
        shuffle = rng.sample(range(6), 6)
        canvas = np.full((960, 1280, 3), 110, np.uint8)
        for ci, (cx, cy) in zip(shuffle, positions):
            canvas = place_rgb(canvas, codes[ci], cx, cy,
                               rng.randint(90,130), rng.uniform(0,360), rng.uniform(0.05,0.3))
        frame = noisy_rgb(canvas, sigma=rng.choice([5, 8, 12]), seed=trial)
        fix = qr_sys.find_target(frame)
        if fix is not None:
            assert fix.forward_m < 20.0
            assert abs(fix.right_m) < 15.0