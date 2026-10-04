import math
import numpy as np
import qrcode
import pytest

from config.params import CAM_IMG_W, CAM_IMG_H
from hardware.camera import FakeCamera, build_camera_rig
from vision.pixel_to_meters import PixelToMeters
from vision.qr_system import QRSystem
from tests.sim_helpers import SimClock, SimVehicle

P = PixelToMeters


def test_pitch_90_matches_the_old_straight_down_formula():
    fwd, right = P.offset_body(100, -50, 5.0, pitch_deg=90)
    dx_m, dy_m = P.offset_at_altitude(100, -50, 5.0)
    assert right == pytest.approx(dx_m)
    assert fwd == pytest.approx(-dy_m)          # image down = behind


def test_centre_pixel_at_45_degrees_is_one_height_ahead():
    fwd, right = P.offset_body(0, 0, 10.0, pitch_deg=45)
    assert fwd == pytest.approx(10.0, rel=1e-6)
    assert right == pytest.approx(0.0, abs=1e-9)


def test_pixel_looking_straight_down_is_zero_offset():
    # At pitch 60 deg, 'straight down' is 30 deg below the image centre.
    _, fy = P.focal_lengths()
    dy = fy * math.tan(math.radians(30))
    fwd, right = P.offset_body(0, dy, 8.0, pitch_deg=60)
    assert fwd == pytest.approx(0.0, abs=1e-6)


def test_above_horizon_returns_none():
    _, fy = P.focal_lengths()
    dy = -fy * math.tan(math.radians(40))       # 40 deg above centre
    assert P.offset_body(0, dy, 5.0, pitch_deg=30) is None


def test_farther_up_the_picture_means_farther_ahead():
    near = P.offset_body(0, 100, 5.0, pitch_deg=60)[0]
    far = P.offset_body(0, -100, 5.0, pitch_deg=60)[0]
    assert far > near


@pytest.mark.parametrize("rot", [0, 90, 180, 270])
def test_image_rotation(rot):
    """A thing straight AHEAD of the drone must come out as forward>0, right~0
    no matter how the camera is rotated on the airframe."""
    # where does 'ahead' appear in the image for a camera rolled clockwise by rot?
    ahead_in_image = {0: (0, -100), 90: (-100, 0), 180: (0, 100), 270: (100, 0)}[rot]
    fwd, right = P.offset_body(*ahead_in_image, 5.0, pitch_deg=90, rotation_deg=rot)
    assert fwd > 0.3 and abs(right) < 1e-6
    # and something to the RIGHT of the drone
    right_in_image = {0: (100, 0), 90: (0, -100), 180: (-100, 0), 270: (0, 100)}[rot]
    fwd, right = P.offset_body(*right_in_image, 5.0, pitch_deg=90, rotation_deg=rot)
    assert right > 0.3 and abs(fwd) < 1e-6


class _TiltedCam(FakeCamera):
    pitch_deg = 45.0


def test_qr_system_uses_the_camera_tilt():
    frame = np.full((CAM_IMG_H, CAM_IMG_W, 3), 255, np.uint8)
    q = np.array(qrcode.make("T1", border=2).convert("L").resize((220, 220)))
    frame[250:470, 530:750] = q[..., None]               # centred on (640, 360)
    veh = SimVehicle(SimClock()); veh.altitude = 10.0
    qs = QRSystem(_TiltedCam(frame), veh)
    qs.delivery_id = "T1"
    fix = qs.find_target()
    assert fix is not None
    assert fix.forward_m == pytest.approx(10.0, rel=0.05)   # 45 deg => 1 x height ahead
    assert abs(fix.right_m) < 0.3


def test_rig_view_feeds_qr_system_tilt():
    rig = build_camera_rig(mode="dual", cameras={"down": FakeCamera(np.zeros((720, 1280, 3), np.uint8)),
                                                  "forward": FakeCamera(np.zeros((720, 1280, 3), np.uint8))})
    veh = SimVehicle(SimClock())
    assert QRSystem(rig.view("down"), veh).pitch_deg == 90.0
    assert QRSystem(rig.view("forward"), veh).pitch_deg == 10.0
