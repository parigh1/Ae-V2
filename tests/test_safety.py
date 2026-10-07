# tests/test_safety.py - Phase 7b: watchdog health checks, stricter pre-flight, persistent emergency
from types import SimpleNamespace

import pytest

from config.params import (HEARTBEAT_LOST_S, BATTERY_UNKNOWN_S, GPS_LOST_S, PY_FENCE_RADIUS_M,
                           PY_FENCE_MAX_ALT_M, PY_FENCE_DEBOUNCE_S, EMERGENCY_RTL_TRIES, EMERGENCY_RTL_RETRY_S)
from hardware.vehicle import MockVehicle, haversine_m
from mission import handlers
from mission.states import State
from safety.watchdog import Watchdog

M_PER_DEG_LAT = 111_320.0


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


def armed_mock(**over):
    v = MockVehicle(verbose=False)
    v.arm()
    for k, val in over.items():
        setattr(v, k, val)
    return v


def watchdog(v):
    clock = Clock()
    return Watchdog(v, clock=clock), clock


def run_for(wd, clock, seconds, step=0.1):
    """Call check() every `step` seconds; return the first event (or None)."""
    for _ in range(int(round(seconds / step)) + 1):
        ev = wd.check()
        if ev is not None:
            return ev
        clock.advance(step)
    return None


# ── watchdog: things that must NOT trigger ───────────────────────────────────
def test_healthy_flight_has_no_events():
    wd, clock = watchdog(armed_mock())
    assert run_for(wd, clock, 30) is None


def test_nothing_is_checked_on_the_ground():
    v = MockVehicle(verbose=False)                      # not armed
    v._heartbeat_age, v._gps_fix, v._battery_known = 99.0, 0, False
    wd, clock = watchdog(v)
    assert run_for(wd, clock, 10) is None


def test_pilot_takeover_still_wins():
    v = armed_mock()
    v._mode = "STABILIZE"
    wd, _ = watchdog(v)
    ev = wd.check()
    assert ev.action == "stop" and "pilot_override" in ev.reason


# ── heartbeat ────────────────────────────────────────────────────────────────
def test_lost_heartbeat_is_an_emergency():
    v = armed_mock()
    wd, _ = watchdog(v)
    v._heartbeat_age = HEARTBEAT_LOST_S - 0.1
    assert wd.check() is None
    v._heartbeat_age = HEARTBEAT_LOST_S + 0.1
    ev = wd.check()
    assert ev.action == "emergency" and "link_lost" in ev.reason


# ── battery ──────────────────────────────────────────────────────────────────
def test_low_percent_and_low_voltage_still_trigger():
    v = armed_mock(_battery=20.0)
    assert watchdog(v)[0].check().reason.startswith("low_battery")
    v = armed_mock(_voltage=14.0)
    assert watchdog(v)[0].check().reason.startswith("low_battery")


def test_missing_battery_data_is_not_treated_as_full():
    v = armed_mock(_battery_known=False)                 # the real Vehicle would report 100 % here
    wd, clock = watchdog(v)
    assert wd.check() is None                            # a short gap is tolerated
    ev = run_for(wd, clock, BATTERY_UNKNOWN_S + 1)
    assert ev.action == "emergency" and "battery_unknown" in ev.reason


def test_battery_data_coming_back_resets_the_timer():
    v = armed_mock()
    wd, clock = watchdog(v)
    v._battery_known = False
    assert run_for(wd, clock, BATTERY_UNKNOWN_S - 1) is None
    v._battery_known = True
    assert run_for(wd, clock, 1) is None
    v._battery_known = False
    assert run_for(wd, clock, BATTERY_UNKNOWN_S - 1) is None       # timer restarted, not continued


# ── GPS ──────────────────────────────────────────────────────────────────────
def test_gps_fix_lost_for_a_while_is_an_emergency_but_a_blip_is_not():
    v = armed_mock()
    wd, clock = watchdog(v)
    v._gps_fix = 1
    assert run_for(wd, clock, GPS_LOST_S - 1) is None
    v._gps_fix = 3
    assert run_for(wd, clock, 1) is None
    v._gps_fix = 1
    ev = run_for(wd, clock, GPS_LOST_S + 1)
    assert ev.action == "emergency" and "gps_lost" in ev.reason


# ── software geofence ────────────────────────────────────────────────────────
def move_north(v, metres):
    v._lat = metres / M_PER_DEG_LAT


def test_far_from_the_takeoff_point_is_an_emergency_after_the_debounce():
    v = armed_mock()
    wd, clock = watchdog(v)
    assert wd.check() is None                            # home is captured here
    move_north(v, PY_FENCE_RADIUS_M + 10)
    assert run_for(wd, clock, PY_FENCE_DEBOUNCE_S - 0.5) is None
    ev = run_for(wd, clock, 2)
    assert ev.action == "emergency" and "geofence" in ev.reason


def test_a_single_gps_glitch_does_not_abort():
    v = armed_mock()
    wd, clock = watchdog(v)
    wd.check()
    move_north(v, 500)                                   # one wild reading ...
    assert wd.check() is None
    clock.advance(0.1)
    move_north(v, 5)                                     # ... then normal again
    assert run_for(wd, clock, 10) is None


def test_inside_the_fence_is_fine():
    v = armed_mock()
    wd, clock = watchdog(v)
    wd.check()
    move_north(v, PY_FENCE_RADIUS_M - 10)
    assert run_for(wd, clock, 10) is None


def test_fence_is_centred_on_the_takeoff_point_not_on_zero_zero():
    v = armed_mock()
    v._lat, v._lon = 21.0, 79.0                          # take-off far from (0, 0)
    wd, clock = watchdog(v)
    assert run_for(wd, clock, 10) is None


def test_too_high_is_an_emergency_after_the_debounce():
    v = armed_mock(_altitude=PY_FENCE_MAX_ALT_M + 3)
    wd, clock = watchdog(v)
    ev = run_for(wd, clock, PY_FENCE_DEBOUNCE_S + 1)
    assert ev.action == "emergency" and "max_altitude" in ev.reason


class NoPositionVehicle(MockVehicle):
    """The real Vehicle returns lat/lon = None until the GPS has a fix."""
    @property
    def gps_location(self):
        return SimpleNamespace(lat=None, lon=None, alt=None)


def test_no_position_yet_skips_the_fence_instead_of_crashing():
    v = NoPositionVehicle(verbose=False)
    v.arm()
    wd, clock = watchdog(v)
    assert run_for(wd, clock, 5) is None


def test_a_crashing_check_becomes_an_emergency_not_an_exception():
    v = armed_mock()
    wd, _ = watchdog(v)
    v.distance_to_m = lambda lat, lon: (_ for _ in ()).throw(RuntimeError("boom"))
    wd.check()                                            # captures home; the distance call then raises
    ev = wd.check()
    assert ev.action == "emergency" and "watchdog_error" in ev.reason


# ── MockVehicle has the new readings ─────────────────────────────────────────
def test_mock_vehicle_defaults_are_healthy():
    v = MockVehicle(verbose=False)
    assert (v.gps_fix_type, v.gps_satellites, v.heartbeat_age_s, v.ekf_ok, v.battery_known) == (3, 12, 0.0, True, True)


# ── pre-flight ───────────────────────────────────────────────────────────────
def pre_ctx(vehicle=None, recorder=None, payload=None):
    logs = []
    return SimpleNamespace(vehicle=vehicle or MockVehicle(verbose=False), data={}, payload=payload,
                           recorder=recorder, log=logs.append, logs=logs)


def test_preflight_all_good_arms():
    ctx = pre_ctx()
    assert handlers.h_preflight(ctx) is State.ARM and "emergency_reason" not in ctx.data


@pytest.mark.parametrize("field, value, words", [
    ("_battery", 50.0, "battery 50%"),
    ("_battery_known", False, "not reported"),
    ("_heartbeat_age", 9.0, "heartbeat"),
    ("_gps_fix", 2, "GPS not ready"),
    ("_gps_sats", 5, "GPS not ready"),
    ("_ekf_ok", False, "EKF"),
])
def test_preflight_refuses_each_problem(field, value, words):
    v = MockVehicle(verbose=False)
    setattr(v, field, value)
    ctx = pre_ctx(v)
    assert handlers.h_preflight(ctx) is State.EMERGENCY
    assert words in ctx.data["emergency_reason"]


def test_preflight_lists_every_problem_at_once():
    v = MockVehicle(verbose=False)
    v._battery, v._gps_sats, v._ekf_ok = 50.0, 4, False
    ctx = pre_ctx(v)
    handlers.h_preflight(ctx)
    reason = ctx.data["emergency_reason"]
    assert "battery" in reason and "GPS" in reason and "EKF" in reason


def test_preflight_refuses_when_the_flight_log_is_dead():
    ctx = pre_ctx(recorder=SimpleNamespace(ok=False))
    assert handlers.h_preflight(ctx) is State.EMERGENCY
    assert "flight log" in ctx.data["emergency_reason"]
    assert handlers.h_preflight(pre_ctx(recorder=SimpleNamespace(ok=True))) is State.ARM


def test_preflight_wrong_failsafe_params_still_refused():
    v = MockVehicle(verbose=False)
    v.verify_params = lambda required: False
    ctx = pre_ctx(v)
    assert handlers.h_preflight(ctx) is State.EMERGENCY
    assert "parameters" in ctx.data["emergency_reason"]


class NoRangefinderVehicle(MockVehicle):
    @property
    def rangefinder_distance(self):
        return None


def test_preflight_no_rangefinder_is_only_a_warning():
    ctx = pre_ctx(NoRangefinderVehicle(verbose=False))
    assert handlers.h_preflight(ctx) is State.ARM
    assert any("rangefinder" in line for line in ctx.logs)


# ── emergency handler: never gives up while the drone is still flying ────────
class EmVehicle:
    def __init__(self, fail_rtl=0, fail_land=0):
        self.is_armed, self.mode_name = True, "GUIDED"
        self.fail_rtl, self.fail_land, self.calls = fail_rtl, fail_land, []

    def rtl(self):
        self.calls.append("rtl")
        if self.fail_rtl > 0:
            self.fail_rtl -= 1
            raise RuntimeError("mode change timed out")
        self.mode_name = "RTL"

    def land(self):
        self.calls.append("land")
        if self.fail_land > 0:
            self.fail_land -= 1
            raise RuntimeError("mode change timed out")
        self.mode_name = "LAND"


def em_ctx(vehicle):
    clock = Clock()
    return SimpleNamespace(vehicle=vehicle, sd={}, clock=clock, log=lambda m: None)


def test_emergency_on_the_ground_finishes_immediately():
    v = EmVehicle()
    v.is_armed = False
    assert handlers.h_emergency(em_ctx(v)) is State.DONE and v.calls == []


def test_emergency_switches_to_rtl_once_and_waits_for_touchdown():
    v = EmVehicle()
    ctx = em_ctx(v)
    assert handlers.h_emergency(ctx) is None and v.calls == ["rtl"]
    for _ in range(5):
        ctx.clock.advance(1.0)
        assert handlers.h_emergency(ctx) is None
    assert v.calls == ["rtl"]                            # no repeated RTL commands once it took
    v.is_armed = False
    assert handlers.h_emergency(ctx) is State.DONE


def test_failed_rtl_is_retried_not_abandoned():
    v = EmVehicle(fail_rtl=1)
    ctx = em_ctx(v)
    assert handlers.h_emergency(ctx) is None             # attempt 1 fails: the old code ended the mission here
    assert handlers.h_emergency(ctx) is None and v.calls == ["rtl"]    # too soon to retry
    ctx.clock.advance(EMERGENCY_RTL_RETRY_S)
    handlers.h_emergency(ctx)
    assert v.calls == ["rtl", "rtl"] and v.mode_name == "RTL"


def test_rtl_that_never_works_falls_back_to_land():
    v = EmVehicle(fail_rtl=99)
    ctx = em_ctx(v)
    for _ in range(EMERGENCY_RTL_TRIES + 2):
        handlers.h_emergency(ctx)
        ctx.clock.advance(EMERGENCY_RTL_RETRY_S)
    assert v.calls[:EMERGENCY_RTL_TRIES] == ["rtl"] * EMERGENCY_RTL_TRIES
    assert "land" in v.calls and v.mode_name == "LAND"