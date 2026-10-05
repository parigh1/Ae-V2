from types import SimpleNamespace

import pytest

from config import params
from mission.context import MissionContext
from mission.handlers import default_handlers, h_center_over_qr, _hold_altitude, stub
from mission.state_machine import StateMachine
from mission.states import State, NEXT, ORDER, TIMEOUT_FALLBACK, allowed_next
from navigation.altitude import AltitudeController
from navigation.visual_servo import VisualServo
from payload.release import PayloadReleaser
from safety.watchdog import Watchdog
from hardware.camera import build_camera_rig
from tests.sim_helpers import SimClock, FlightSim, BannerWorldCamera
from tests.test_qr_pipeline import make_frame, FakeCamera
from vision.qr_system import QRSystem

TARGET_WORLD = (2.0, 1.0)      # where the 'right' QR lies, metres N / E of the start


def build(frame=None, battery=100.0, see_target=True, banner_bearing=0.0):
    clock = SimClock()
    veh = FlightSim(clock)
    veh._battery = battery
    if frame is None:
        frame = make_frame({"DELIVERY_42": (640, 360)})
    qr = QRSystem(FakeCamera(frame), veh, sleep=veh.advance, clock=clock.now)
    if see_target:   # pretend camera: exact geometry from the sim
        def fake_find(frame=None):
            f, r = veh.target_in_body(*TARGET_WORLD)
            return SimpleNamespace(forward_m=f, right_m=r)
        qr.find_target = fake_find
    rig = build_camera_rig(mode="single_fixed",
                           cameras={"main": BannerWorldCamera(veh, banner_bearing)})
    ctx = MissionContext(
        veh, rig=rig, qr=qr,
        altitude=AltitudeController(veh, clock=clock.now),
        servo=VisualServo(veh, clock=clock.now, sleep=veh.advance),
        safety=Watchdog(veh), payload=PayloadReleaser(veh),
        clock=clock.now, sleep=veh.advance)
    return ctx, veh

def sim_handlers():
      """Real handlers, except the corridor (it has its own camera world and tests)."""
      h = default_handlers()
      h[State.CORRIDOR_FORWARD] = stub(State.CORRIDOR_FORWARD)
      h[State.CORRIDOR_RETURN] = stub(State.CORRIDOR_RETURN)
      return h


def fly(ctx, handlers=None):
    return StateMachine(ctx, handlers or sim_handlers()).run()

def states_visited(result):
    return [name for name, _ in result.history]


# ── the tables ───────────────────────────────────────────────────────────────
def test_22_states_and_complete_tables():
    assert len(State) == 22
    for s in State:
        if s in (State.DONE, State.EMERGENCY):
            continue
        assert s.name in params.STATE_TIMEOUTS, f"{s.name} has no time budget"
        assert s in TIMEOUT_FALLBACK, f"{s.name} has no timeout fallback"
        assert NEXT[s] in allowed_next(s)
        assert State.EMERGENCY in allowed_next(s)


def test_worst_case_time_fits_in_the_mission_clock():
    worst = sum(params.STATE_TIMEOUTS.values())
    assert worst < params.MISSION_TIME_LIMIT_S - params.MISSION_EMERGENCY_MARGIN_S


# ── full flights in the simulator ────────────────────────────────────────────
def test_happy_path_flies_every_state_in_order():
    ctx, veh = build()
    res = fly(ctx)
    assert res.success and res.reason == "completed"
    assert states_visited(res) == [s.name for s in ORDER]
    assert res.payload_released
    assert any("ch10" in c and str(params.PWM_GRIPPER_OPEN) in c for c in veh._commands)
    assert not veh.is_armed and veh.altitude == 0.0
    assert res.elapsed_s < params.MISSION_TIME_LIMIT_S - params.MISSION_EMERGENCY_MARGIN_S
    assert ctx.delivery_id == "DELIVERY_42"


def test_no_start_qr_still_finishes_but_skips_the_drop():
    ctx, veh = build(frame=make_frame({}))
    ctx.qr.scanner.decode_frame = lambda frame: None     # "never sees a code" (keeps the test fast)
    res = fly(ctx)
    assert res.success and not res.payload_released
    names = states_visited(res)
    assert names[names.index("SEARCH_DELIVERY") + 1] == "FIND_BANNER_RTN"
    assert "DEPLOY_PAYLOAD" not in names and names[-1] == "DONE"


def test_state_timeout_uses_its_fallback():
    ctx, veh = build()
    handlers = sim_handlers()
    handlers[State.SEARCH_DELIVERY] = lambda c: _hold_altitude(c)      # never finds anything
    res = fly(ctx, handlers)
    names = states_visited(res)
    assert names[names.index("SEARCH_DELIVERY") + 1] == "FIND_BANNER_RTN"
    assert res.success and not res.payload_released


# ── things going wrong ───────────────────────────────────────────────────────
def test_crash_in_a_handler_becomes_emergency_rtl():
    ctx, veh = build()
    handlers = sim_handlers()
    def boom(c): raise RuntimeError("camera exploded")
    handlers[State.CORRIDOR_FORWARD] = boom
    res = fly(ctx, handlers)
    assert not res.success and "camera exploded" in res.reason
    assert states_visited(res)[-2:] == ["EMERGENCY", "DONE"]
    assert any("RTL" in c for c in veh._commands)


def test_illegal_jump_becomes_emergency():
    ctx, veh = build()
    handlers = sim_handlers()
    handlers[State.CORRIDOR_FORWARD] = lambda c: State.LAND
    res = fly(ctx, handlers)
    assert not res.success and "illegal transition" in res.reason


def test_low_battery_mid_flight_triggers_rtl():
    ctx, veh = build()
    handlers = sim_handlers()
    def drain(c):
        veh._battery = 20.0
    handlers[State.CORRIDOR_FORWARD] = drain
    res = fly(ctx, handlers)
    assert not res.success and "low_battery" in res.reason
    assert states_visited(res)[-2:] == ["EMERGENCY", "DONE"]


def test_pilot_takeover_stops_all_commands():
    ctx, veh = build()
    handlers = sim_handlers()
    seen = {}
    def pilot_flips_switch(c):
        veh._mode = "STABILIZE"
        seen["n"] = len(veh._commands)
    handlers[State.CORRIDOR_FORWARD] = pilot_flips_switch
    res = fly(ctx, handlers)
    assert not res.success and "pilot_override" in res.reason
    assert len(veh._commands) == seen["n"]            # the Pi sent NOTHING afterwards


def test_running_out_of_mission_time_triggers_rtl():
    ctx, veh = build()
    handlers = sim_handlers()
    def time_warp(c):
        c.mission_t0 -= 850                             # pretend 14 minutes passed
    handlers[State.CORRIDOR_FORWARD] = time_warp
    res = fly(ctx, handlers)
    assert not res.success and "mission_time" in res.reason


def test_preflight_failure_never_arms():
    ctx, veh = build(battery=50.0)
    res = fly(ctx)
    assert not res.success and "battery" in res.reason
    assert states_visited(res) == ["INIT", "PREFLIGHT", "EMERGENCY", "DONE"]
    assert not any("ARMED" in c for c in veh._commands)


# ── target handling in isolation ─────────────────────────────────────────────
def test_losing_the_target_goes_back_to_searching_then_gives_up():
    ctx, veh = build(see_target=False)
    ctx.qr.find_target = lambda frame=None: None
    ctx.state = State.CENTER_OVER_QR
    for expected_retry, expected_state in [(1, State.SEARCH_DELIVERY), (2, State.SEARCH_DELIVERY),
                                           (3, State.FIND_BANNER_RTN)]:
        ctx.sd = {}
        out = None
        for _ in range(params.SERVO_LOST_FRAMES_MAX + 2):
            out = h_center_over_qr(ctx)
            if out is not None:
                break
        assert out == expected_state
        assert ctx.data["center_retries"] == expected_retry


def test_timeout_that_falls_back_to_emergency_counts_as_failure():
    ctx, veh = build()
    handlers = sim_handlers()
    handlers[State.CORRIDOR_FORWARD] = lambda c: _hold_altitude(c)   # corridor never finishes
    res = fly(ctx, handlers)
    assert not res.success and "timeout in CORRIDOR_FORWARD" in res.reason
