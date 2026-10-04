import numpy as np
import pytest

from config import params
from hardware.camera import (build_camera_rig, FakeCamera, CameraServo)
from tests.sim_helpers import SimClock, SimVehicle


def frame(value):
    return np.full((720, 1280, 3), value, np.uint8)


def test_single_fixed_both_views_use_the_same_camera():
    rig = build_camera_rig(mode="single_fixed", cameras={"main": FakeCamera(frame(7))})
    assert rig.view("down").capture_array()[0, 0, 0] == 7
    assert rig.view("forward").capture_array()[0, 0, 0] == 7
    assert rig.view("down").pitch_deg == params.CAM_MOUNTS["single_fixed"]["down"]
    assert rig.view("forward").pitch_deg == params.CAM_MOUNTS["single_fixed"]["forward"]


def test_dual_routes_each_view_to_its_own_camera():
    rig = build_camera_rig(mode="dual", cameras={"down": FakeCamera(frame(1)),
                                                  "forward": FakeCamera(frame(2))})
    assert rig.view("down").capture_array()[0, 0, 0] == 1
    assert rig.view("forward").capture_array()[0, 0, 0] == 2
    assert rig.view("down").pitch_deg == 90.0


def test_servo_mode_tilts_before_capturing_and_only_when_view_changes():
    veh = SimVehicle(SimClock())
    rig = build_camera_rig(vehicle=veh, mode="servo", cameras={"main": FakeCamera(frame(3))})
    rig._servo._sleep = lambda s: None
    rig.view("down").capture_array()
    rig.view("down").capture_array()          # same view -> no new servo command
    rig.view("forward").capture_array()
    servo_cmds = [c for c in veh._commands if "SERVO" in c]
    assert len(servo_cmds) == 2
    assert f"{CameraServo.pwm_for_pitch(90.0)}" in servo_cmds[0]
    assert f"{CameraServo.pwm_for_pitch(0.0)}" in servo_cmds[1]


def test_servo_pwm_mapping_endpoints_and_clamp():
    assert CameraServo.pwm_for_pitch(90) == params.CAM_SERVO_PWM_DOWN
    assert CameraServo.pwm_for_pitch(0) == params.CAM_SERVO_PWM_FORWARD
    assert CameraServo.pwm_for_pitch(500) == params.CAM_SERVO_PWM_DOWN


def test_servo_mode_requires_vehicle():
    with pytest.raises(ValueError):
        build_camera_rig(mode="servo", cameras={"main": FakeCamera(frame(0))})


def test_bad_mode_and_bad_view_rejected():
    with pytest.raises(ValueError):
        build_camera_rig(mode="banana", cameras={"main": FakeCamera(frame(0))})
    rig = build_camera_rig(mode="single_fixed", cameras={"main": FakeCamera(frame(0))})
    with pytest.raises(ValueError):
        rig.view("sideways")
