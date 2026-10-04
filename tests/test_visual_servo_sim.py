import pytest

from navigation.visual_servo import VisualServo
from tests.sim_helpers import SimClock, SimVehicle


@pytest.mark.parametrize("heading", [0, 70, 180, 290])
@pytest.mark.parametrize("target", [(3.0, 2.0), (-2.5, 1.0), (1.0, -3.0)])
def test_converges_at_any_heading(heading, target):
    """Body-frame control must reach the target regardless of which way the
    drone is facing (the old LOCAL_NED version only worked facing north)."""
    clock = SimClock()
    veh = SimVehicle(clock, heading_deg=heading)
    servo = VisualServo(veh, clock=clock.now, sleep=veh.advance)
    ok = servo.run(lambda: veh.target_in_body(*target), timeout=30.0)
    assert ok, f"did not converge: still {veh.target_in_body(*target)}"
    fwd, right = veh.target_in_body(*target)
    assert abs(fwd) < 0.3 and abs(right) < 0.3


def test_gives_up_when_target_lost():
    clock = SimClock()
    veh = SimVehicle(clock)
    servo = VisualServo(veh, clock=clock.now, sleep=veh.advance)
    assert servo.run(lambda: None, timeout=30.0) is False
    assert veh.last_body_cmd == (0.0, 0.0, 0.0)      # left hovering


def test_speed_limit_preserves_direction():
    veh = SimVehicle(SimClock())
    servo = VisualServo(veh)
    vx, vy = servo.compute(10.0, 5.0, now=1.0)
    assert (vx ** 2 + vy ** 2) ** 0.5 == pytest.approx(0.30, abs=1e-6)
    assert vx / vy == pytest.approx(2.0, rel=1e-6)
