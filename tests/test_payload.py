import pytest

from config import params
from payload.release import PayloadReleaser
from tests.sim_helpers import SimClock, SimVehicle


def servo_cmds(veh):
    return [c.split("SERVO ")[1] for c in veh._commands if "SERVO" in c]


def run_until_done(rel, step=0.1, limit=60.0):
    t = 0.0
    while t < limit:
        if rel.step(t):
            return t
        t += step
    raise AssertionError("never finished")


def test_gripper_drop_opens_once_then_finishes_after_wait():
    veh = SimVehicle(SimClock())
    rel = PayloadReleaser(veh, mode="gripper_drop")
    assert rel.step(0.0) is False
    assert servo_cmds(veh) == [f"ch{params.GRIPPER_PWM_CHANNEL} = {params.PWM_GRIPPER_OPEN}µs"]
    assert rel.step(params.PAYLOAD_RELEASE_WAIT_S - 0.2) is False
    assert rel.step(params.PAYLOAD_RELEASE_WAIT_S + 0.1) is True
    assert len(servo_cmds(veh)) == 1          # no repeated commands


def test_winch_sequence_order_and_total_time():
    veh = SimVehicle(SimClock())
    rel = PayloadReleaser(veh, mode="winch")
    total = run_until_done(rel)
    assert [c.split(" = ")[0] for c in servo_cmds(veh)] == [
        f"ch{params.WINCH_PWM_CHANNEL}", f"ch{params.WINCH_PWM_CHANNEL}",
        f"ch{params.GRIPPER_PWM_CHANNEL}", f"ch{params.WINCH_PWM_CHANNEL}",
        f"ch{params.WINCH_PWM_CHANNEL}"]
    assert [int(c.split(" = ")[1].rstrip("µs")) for c in servo_cmds(veh)] == [
        params.PWM_WINCH_LOWER, params.PWM_WINCH_STOP, params.PWM_GRIPPER_OPEN,
        params.PWM_WINCH_RAISE, params.PWM_WINCH_STOP]
    assert total < params.STATE_TIMEOUTS["DEPLOY_PAYLOAD"]    # fits the state's time budget


def test_hold_closes_gripper():
    veh = SimVehicle(SimClock())
    PayloadReleaser(veh, mode="gripper_drop").hold()
    assert servo_cmds(veh) == [f"ch{params.GRIPPER_PWM_CHANNEL} = {params.PWM_GRIPPER_CLOSED}µs"]


def test_bad_mode_rejected():
    with pytest.raises(ValueError):
        PayloadReleaser(SimVehicle(SimClock()), mode="catapult")
