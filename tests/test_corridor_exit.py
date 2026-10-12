# tests/test_corridor_exit.py - the camera only sees a few metres ahead, so near the end of the corridor the
# walls "run out" of view before the corridor does. The drone must keep creeping out on distance alone.
from types import SimpleNamespace

import pytest

from config import params
from mission import handlers
from mission.states import State, NEXT

DT = 0.1
WALLS_VISIBLE_UNTIL_M = 8.9        # what a narrow-view camera really does in the simulator: lost ~1 m early


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


class FakeController:
    def compute(self, fix, obstacle, now):
        if fix is None or obstacle is not None:                             # blind or blocked -> full stop (the real behaviour)
            return SimpleNamespace(vx=0.0, vy=0.0, yaw_rate_dps=0.0, status="lost")
        return SimpleNamespace(vx=params.CORRIDOR_FWD_SPEED, vy=0.0, yaw_rate_dps=0.0, status="ok")


class FakeTracker:                                                          # dead reckoning only
    disagreement = False

    def __init__(self):
        self.total = 0.0

    def update(self, vx, dt, gps_distance):
        self.total += max(vx, 0.0) * dt
        return self.total


class FakeVehicle:
    gps_location = SimpleNamespace(lat=0.0, lon=0.0)
    rangefinder_distance = None
    altitude = 3.0

    def __init__(self):
        self.n, self.hovered = 0.0, False

    def distance_to_m(self, lat, lon):
        return self.n

    def send_velocity_yawrate(self, vx, vy, vz, yaw):
        self.n += vx * DT

    def hover(self):
        self.hovered = True


def make(monkeypatch, visible_until=WALLS_VISIBLE_UNTIL_M, obstacle_from_m=None):
    veh = FakeVehicle()
    clock = Clock()
    ctx = SimpleNamespace(
        vehicle=veh, sd={}, clock=clock, log=lambda m: None,
        altitude=SimpleNamespace(compute=lambda: 0.0, set_target=lambda m: None),
        rig=SimpleNamespace(view=lambda name: SimpleNamespace(pitch_deg=65.0, capture_array=lambda: None)),
        corridor=SimpleNamespace(detect=lambda frame, h, p: object() if veh.n < visible_until else None),
        obstacle_sensors=[])
    monkeypatch.setattr(handlers, "CorridorController", FakeController)
    monkeypatch.setattr(handlers, "DistanceTracker", FakeTracker)
    monkeypatch.setattr(handlers, "nearest_obstacle",
                        lambda *a: object() if obstacle_from_m is not None and veh.n >= obstacle_from_m else None)
    return ctx, veh, clock, handlers.corridor_state(State.CORRIDOR_FORWARD)


def run(ctx, veh, clock, handler, seconds):
    out = None
    for _ in range(int(seconds / DT)):
        out = handler(ctx)
        clock.t += DT
        if out is not None:
            break
    return out


def test_drone_keeps_going_when_the_walls_run_out_early_and_leaves_at_the_end(monkeypatch):
    ctx, veh, clock, handler = make(monkeypatch)
    out = run(ctx, veh, clock, handler, 120)
    assert out == NEXT[State.CORRIDOR_FORWARD]
    assert veh.n >= params.CORRIDOR_LENGTH_M                               # it did NOT stop 1 m short
    assert veh.n <= params.CORRIDOR_LENGTH_M + params.CORRIDOR_EXIT_MARGIN_M + 0.5


def test_it_does_not_creep_blind_at_the_start_of_the_corridor(monkeypatch):
    ctx, veh, clock, handler = make(monkeypatch, visible_until=0.0)         # never sees a wall
    assert run(ctx, veh, clock, handler, 60) is None                        # stays put until the state clock ends it
    assert veh.n == 0.0


def test_it_does_not_creep_blind_into_an_obstacle(monkeypatch):
    # inside the last stretch, the walls run out of view at the same moment something appears in the lane
    appears_at = params.CORRIDOR_LENGTH_M - params.CORRIDOR_COAST_M + 0.5
    ctx, veh, clock, handler = make(monkeypatch, visible_until=appears_at, obstacle_from_m=appears_at)
    run(ctx, veh, clock, handler, 120)
    assert veh.n < appears_at + 0.5                                         # it stopped, it did not creep on


def test_creeping_speed_is_slower_than_normal_flight():
    assert 0.0 < params.CORRIDOR_COAST_SPEED < params.CORRIDOR_FWD_SPEED