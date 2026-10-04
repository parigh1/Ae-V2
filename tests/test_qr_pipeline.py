import numpy as np
import qrcode

from config.params import CAM_IMG_W, CAM_IMG_H
from vision.pixel_to_meters import PixelToMeters
from vision.qr_scanner import RobustQRScanner, QRConfirmer
from vision.qr_system import QRSystem
from tests.sim_helpers import SimClock, SimVehicle


def qr_img(text, px):
    img = qrcode.make(text, border=2).convert("L").resize((px, px))
    return np.array(img)


def make_frame(placements, size_px=220):
    """white 1280x720 RGB frame with QR codes pasted at (cx, cy) centres."""
    frame = np.full((CAM_IMG_H, CAM_IMG_W, 3), 255, np.uint8)
    for text, (cx, cy) in placements.items():
        q = qr_img(text, size_px)
        x0, y0 = int(cx - size_px / 2), int(cy - size_px / 2)
        frame[y0:y0 + size_px, x0:x0 + size_px] = q[..., None]
    return frame


class FakeCamera:
    def __init__(self, frame):
        self.frame = frame

    def capture_array(self):
        return self.frame


def test_decode_all_returns_all_codes_with_centres():
    pos = {"AAA": (250, 200), "TARGET_A3": (900, 450), "ZZZ": (400, 600)}
    dets = {d.data: d for d in RobustQRScanner().decode_all(make_frame(pos))}
    assert set(dets) == set(pos)
    for name, (cx, cy) in pos.items():
        assert abs(dets[name].cx - cx) < 8 and abs(dets[name].cy - cy) < 8


def test_want_picks_the_right_code_among_many():
    pos = {"AAA": (250, 200), "TARGET_A3": (900, 450)}
    dets = RobustQRScanner().decode_all(make_frame(pos), want="TARGET_A3")
    assert any(d.data == "TARGET_A3" for d in dets)


def test_body_frame_signs():
    # target right of and ABOVE centre => to the right and AHEAD
    fwd, right = PixelToMeters.offset_body(+100, -100, 5.0)
    assert right > 0 and fwd > 0
    # target left of and BELOW centre => left and BEHIND
    fwd, right = PixelToMeters.offset_body(-100, +100, 5.0)
    assert right < 0 and fwd < 0


def test_scale_matches_readme_numbers():
    # README: 100 px at 5 m ≈ 0.70 m
    dx_m, _ = PixelToMeters.offset_at_altitude(100, 0, 5.0)
    assert abs(dx_m - 0.70) < 0.02


def test_find_target_end_to_end():
    clock = SimClock()
    veh = SimVehicle(clock)
    veh.altitude = 10.0
    cam = FakeCamera(make_frame({"DECOY": (300, 300), "GO_HERE": (960, 180)}))
    qs = QRSystem(cam, veh, sleep=lambda s: None, clock=clock.now)
    qs.delivery_id = "GO_HERE"
    fix = qs.find_target()
    assert fix is not None
    # centre is (640,360); target is right (+320) and above (-180) => ahead+right
    assert fix.right_m > 0 and fix.forward_m > 0


def test_find_target_none_when_absent():
    clock = SimClock()
    veh = SimVehicle(clock); veh.altitude = 10.0
    qs = QRSystem(FakeCamera(make_frame({"OTHER": (300, 300)})), veh)
    qs.delivery_id = "GO_HERE"
    assert qs.find_target() is None


def test_scan_start_requires_confirmation():
    clock = SimClock()
    veh = SimVehicle(clock); veh.altitude = 5.0
    cam = FakeCamera(make_frame({"DELIVERY_42": (640, 360)}))
    qs = QRSystem(cam, veh, sleep=lambda s: clock.__setattr__("t", clock.t + 0.1),
                  clock=clock.now)
    assert qs.scan_start_qr(timeout=5.0) == "DELIVERY_42"


def test_confirmer():
    c = QRConfirmer(2)
    assert c.update("A") is None
    assert c.update("B") is None      # changed -> restart
    assert c.update("B") == "B"
    assert c.update(None) is None
