import pytest

from mission.handlers import default_handlers
from mission.state_machine import StateMachine
from mission.states import State
from config import params
from tests.test_state_machine import build, states_visited, sim_handlers

def flight_until_banner_done(bearing, start_heading=0.0):
    ctx, veh = build(banner_bearing=bearing)
    veh._heading = start_heading
    seen = {}
    handlers = sim_handlers()
    real = handlers[State.FIND_BANNER_FWD]

    def spy(c):
        out = real(c)
        if out is not None:
            seen["heading"] = veh.heading
            seen["at"] = c.state_age()
        return out
    handlers[State.FIND_BANNER_FWD] = spy
    res = StateMachine(ctx, handlers).run()
    return res, seen, ctx


@pytest.mark.parametrize("bearing", [0.0, 25.0, -30.0, 120.0, 200.0, 300.0])
def test_drone_turns_to_face_the_banner(bearing):
    res, seen, _ = flight_until_banner_done(bearing)
    assert res.success, res.reason
    err = (seen["heading"] - bearing + 180) % 360 - 180
    assert abs(err) < 6.0                                   # within a few degrees
    assert seen["at"] < params.STATE_TIMEOUTS["FIND_BANNER_FWD"]


def test_banner_never_visible_ends_in_emergency_rtl():
    ctx, veh = build(banner_bearing=None)
    res = StateMachine(ctx, sim_handlers()).run()
    assert not res.success
    names = states_visited(res)
    assert names[names.index("FIND_BANNER_FWD") + 1] == "EMERGENCY"
    assert any("RTL" in c for c in veh._commands)


def test_full_turn_search_fits_in_the_state_time():
    # worst case the banner is just behind us: one almost-full clockwise turn
    assert 360.0 / params.BANNER_SEARCH_YAW_DPS + 10 < params.STATE_TIMEOUTS["FIND_BANNER_RTN"]
