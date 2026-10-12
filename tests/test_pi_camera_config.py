# tests/test_pi_camera_config.py - the real Pi camera must ask for the full-width sensor mode
import sys
import types

import pytest

from config import params
from hardware import camera as cam_mod


class FakePicamera2:
    last = None

    def __init__(self):
        FakePicamera2.last = self
        self.config, self.started = None, False

    def create_video_configuration(self, **kwargs):
        return kwargs

    def configure(self, cfg):
        self.config = cfg

    def start(self):
        self.started = True

    def capture_array(self):
        return "frame"

    def stop(self):
        self.started = False


@pytest.fixture
def fake_picamera2(monkeypatch):
    module = types.ModuleType("picamera2")
    module.Picamera2 = FakePicamera2
    monkeypatch.setitem(sys.modules, "picamera2", module)


def test_the_v2_camera_is_forced_into_its_full_width_sensor_mode():
    assert params.CAM_SENSOR_MODE == (1640, 1232)


def test_pi_camera_asks_for_the_forced_mode_and_the_right_picture(fake_picamera2):
    cam = cam_mod.PiCamera(width=1280, height=960)
    cfg = FakePicamera2.last.config
    assert cfg["main"] == {"size": (1280, 960), "format": "BGR888"}
    assert cfg["raw"] == {"size": tuple(params.CAM_SENSOR_MODE)}
    assert FakePicamera2.last.started and cam.capture_array() == "frame"
    cam.close()
    assert not FakePicamera2.last.started


def test_automatic_mode_when_no_mode_is_configured(fake_picamera2, monkeypatch):
    monkeypatch.setattr(cam_mod, "CAM_SENSOR_MODE", None)
    cam_mod.PiCamera()
    assert "raw" not in FakePicamera2.last.config