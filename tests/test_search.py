# tests/test_search.py - Phase 6: lawnmower path, local frame, planner closed loop
import math
from types import SimpleNamespace

import pytest

from config.params import (SEARCH_ZONE_W, SEARCH_ZONE_H, SEARCH_STRIP_W, SEARCH_MARGIN_M,
                          SEARCH_ENTRY_FROM_LEFT_M, SEARCH_SPEED, STATE_TIMEOUTS)
from navigation.search import (LocalFrame, SearchPlanner, default_path, lawnmower_path,
                               path_length_m, wrap180)

LEFT = SEARCH_ENTRY_FROM_LEFT_M
RIGHT = SEARCH_ZONE_W - SEARCH_ENTRY_FROM_LEFT_M
DT = 0.1


# ── tiny point-mass world (entry frame: x ahead, y right, heading 0 = along +x) ──
class World:
    def __init__(self, red_rects=(), view_m=12.0, camera_forward_only=True):
        self.x = self.y = self.hd = 0.0
        self.t = 0.0
        self.red_rects = list(red_rects)          # (x0, x1, y0, y1)
        self.view_m, self.fwd_only = view_m, camera_forward_only
        self.min_gap = 1e9                        # closest approach to any red rectangle (m, 0 = inside)

    def gap(self, rect):
        x0, x1, y0, y1 = rect
        return math.hypot(max(x0 - self.x, 0, self.x - x1), max(y0 - self.y, 0, self.y - y1))

    def nearest_red(self):
        best = None
        th = math.radians(self.hd)
        for (x0, x1, y0, y1) in self.red_rects:
            px, py = min(max(self.x, x0), x1), min(max(self.y, y0), y1)      # nearest point on rectangle
            dx, dy = px - self.x, py - self.y
            d = math.hypot(dx, dy)
            fwd, right = dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)
            if d > self.view_m or (self.fwd_only and fwd < -1.0):
                continue
            if best is None or d < best.distance_m:
                best = SimpleNamespace(forward_m=fwd, right_m=right, distance_m=max(d, 1e-3))
        return best

    def pose(self):
        return (self.x, self.y, wrap180(self.hd))

    def apply(self, cmd):
        th = math.radians(self.hd)
        self.x += (cmd.vx * math.cos(th) - cmd.vy * math.sin(th)) * DT
        self.y += (cmd.vx * math.sin(th) + cmd.vy * math.cos(th)) * DT
        self.hd += cmd.yaw_rate_dps * DT
        self.t += DT
        for r in self.red_rects:
            self.min_gap = min(self.min_gap, self.gap(r))


def fly(world, planner, limit_s=225.0):
    """Run the planner in simulated time. Returns True if the whole path finished."""
    xs, ys = [], []
    while world.t < limit_s:
        cmd = planner.step(world.pose(), world.nearest_red(), world.t)
        if cmd is None:
            return True
        world.apply(cmd)
        xs.append(world.x); ys.append(world.y)
    return False


# ── path geometry ────────────────────────────────────────────────────────────
def test_strip_count_and_even_spacing():
    path = default_path()
    ys = sorted({w.y for w in path})
    assert len(ys) == math.ceil((SEARCH_ZONE_W - 2 * SEARCH_MARGIN_M) / SEARCH_STRIP_W)
    gaps = [b - a for a, b in zip(ys, ys[1:])]
    assert all(g <= SEARCH_STRIP_W + 1e-9 for g in gaps)          # no gap wider than a strip
    assert max(gaps) - min(gaps) < 1e-9                           # evenly spaced
    assert len(path) == 2 * len(ys)


def test_path_stays_inside_the_margins():
    for w in default_path():
        assert SEARCH_MARGIN_M - 1e-9 <= w.x <= SEARCH_ZONE_H - SEARCH_MARGIN_M + 1e-9
        assert -LEFT + SEARCH_MARGIN_M - 1e-9 <= w.y <= RIGHT - SEARCH_MARGIN_M + 1e-9


def test_headings_alternate_each_strip_and_face_travel_direction():
    path = default_path()
    for i in range(0, len(path), 2):
        a, b = path[i], path[i + 1]
        assert a.heading_deg == b.heading_deg
        assert (b.x - a.x > 0) == (a.heading_deg == 0.0)
        if i >= 2:
            assert path[i].heading_deg != path[i - 2].heading_deg


def test_starts_at_the_end_strip_nearest_the_drone():
    near_left = lawnmower_path(40, 10, 20, 8, 2, start_y=-10)
    near_right = lawnmower_path(40, 10, 20, 8, 2, start_y=+18)
    assert near_left[0].y < near_left[-1].y
    assert near_right[0].y > near_right[-1].y


def test_path_fits_the_state_budget():
    t = path_length_m(default_path()) / SEARCH_SPEED
    assert t < STATE_TIMEOUTS["SEARCH_DELIVERY"] - 40            # leaves room for the slow turns


# ── local frame ──────────────────────────────────────────────────────────────
M = 111_320.0


def test_local_frame_facing_north():
    f = LocalFrame(20.0, 78.0, 0.0)
    x, y, h = f.pose(20.0 + 10 / M, 78.0, 0.0)                    # 10 m north
    assert (round(x, 3), round(y, 3), round(h, 3)) == (10.0, 0.0, 0.0)
    x, y, _ = f.pose(20.0, 78.0 + 10 / (M * math.cos(math.radians(20.0))), 0.0)   # 10 m east
    assert (round(x, 3), round(y, 3)) == (0.0, 10.0)


def test_local_frame_facing_east():
    f = LocalFrame(20.0, 78.0, 90.0)
    x, y, h = f.pose(20.0, 78.0 + 10 / (M * math.cos(math.radians(20.0))), 90.0)  # 10 m east = ahead
    assert (round(x, 3), round(y, 3), round(h, 3)) == (10.0, 0.0, 0.0)
    x, y, _ = f.pose(20.0 + 10 / M, 78.0, 90.0)                                    # 10 m north = to the LEFT
    assert (round(x, 3), round(y, 3)) == (0.0, -10.0)


def test_local_frame_heading_wraps():
    f = LocalFrame(0.0, 0.0, 350.0)
    assert round(f.pose(0.0, 0.0, 10.0)[2], 6) == 20.0
    assert round(f.pose(0.0, 0.0, 340.0)[2], 6) == -10.0


# ── planner, closed loop ─────────────────────────────────────────────────────
def test_clear_zone_whole_path_finished_in_time_and_inside_zone():
    w = World()
    pl = SearchPlanner()
    xs, ys = [], []
    done = False
    while w.t < 225.0:
        cmd = pl.step(w.pose(), None, w.t)
        if cmd is None:
            done = True
            break
        w.apply(cmd)
        xs.append(w.x); ys.append(w.y)
    assert done, f"path not finished after {w.t:.0f} s (waypoint {pl.i}/{len(pl.path)})"
    assert min(xs) > -0.5 and max(xs) < SEARCH_ZONE_H
    assert min(ys) > -LEFT and max(ys) < RIGHT


@pytest.mark.parametrize("rect", [
    (14.0, 24.0, -6.0, -1.0),     # across the first pass
    (14.0, 24.0, -2.0, 4.0),      # across the middle two passes
    (20.0, 30.0, 6.0, 12.0),      # near the far pass
    (12.0, 20.0, -9.0, -1.0),     # head-on, hugging the left edge
])
def test_red_zone_never_entered_and_search_still_finishes(rect):
    w = World(red_rects=[rect])
    pl = SearchPlanner()
    finished = fly(w, pl)
    assert w.min_gap > 0.3, f"drone came within {w.min_gap:.2f} m of / inside the red zone"
    assert finished, f"stuck at waypoint {pl.i}/{len(pl.path)} after {w.t:.0f} s"
    assert pl.skipped == []                      # a zone in the way of a pass must not make us give up waypoints

def test_waypoint_inside_a_red_zone_is_skipped_and_search_continues():
    w = World(red_rects=[(30.0, 40.0, -11.0, -8.0)])             # covers the first pass's far end (38, -9.75)
    pl = SearchPlanner()
    finished = fly(w, pl)
    assert w.min_gap > 0.3
    assert finished, f"stuck at waypoint {pl.i}/{len(pl.path)} after {w.t:.0f} s"
    assert 1 in pl.skipped                                       # waypoint index 1 is (38, -9.75)


def test_no_skipping_when_no_red_zone_is_in_view():
    pl = SearchPlanner()
    cmd = pl.step((0.0, 0.0, 0.0), None, 0.0)
    assert pl.step((0.0, 0.0, 0.0), None, 500.0) is not None     # a long time later, still no red -> no skip
    assert pl.skipped == [] and pl.i == 0

def test_red_zone_blocking_the_whole_width_is_never_entered():
    """Nothing can finish this one; the only requirement is to stay out (the state clock then sends us home)."""
    w = World(red_rects=[(10.0, 30.0, -14.0, 14.0)])
    fly(w, SearchPlanner(), limit_s=120.0)
    assert w.min_gap > 0.3


def test_red_zone_head_on_uses_the_slide():
    w = World(red_rects=[(12.0, 20.0, -9.0, -1.0)])
    pl = SearchPlanner()
    statuses = set()
    while w.t < 60.0:
        cmd = pl.step(w.pose(), w.nearest_red(), w.t)
        statuses.add(cmd.status)
        w.apply(cmd)
    assert "red_slide" in statuses
    assert w.min_gap > 0.3


@pytest.mark.parametrize("goal_y, expect_right", [(8.0, True), (-8.0, False)])
def test_slide_goes_toward_the_side_the_goal_is_on(goal_y, expect_right):
    from navigation.search import Waypoint
    pl = SearchPlanner(path=[Waypoint(30.0, goal_y, 0.0)])
    red = SimpleNamespace(forward_m=3.0, right_m=0.0, distance_m=3.0)       # zone dead ahead, 3 m away
    cmd = pl.step((10.0, 0.0, 0.0), red, 0.0)
    assert cmd.status == "red_slide"
    assert (cmd.vy > 0) == expect_right


def test_stops_returning_commands_after_last_waypoint():
    w = World()
    pl = SearchPlanner()
    assert fly(w, pl)
    assert pl.step(w.pose(), None, w.t) is None


# ── the SEARCH_DELIVERY handler (fake vehicle, fake camera, real planner + frame) ──
from mission import handlers
from mission.states import State, allowed_next

LAT0, LON0, HD0 = 20.0, 78.0, 90.0          # entry heading east, so the frame rotation is exercised


class FakeVehicle:
    def __init__(self, world):
        self.world, self.n, self.e, self.hd = world, 0.0, 0.0, HD0
        self.altitude, self.rangefinder_distance = 10.0, None
        self.frame = LocalFrame(LAT0, LON0, HD0)
        self.hovered = False

    @property
    def heading(self):
        return self.hd % 360.0

    @property
    def gps_location(self):
        return SimpleNamespace(lat=LAT0 + self.n / M, lon=LON0 + self.e / (M * math.cos(math.radians(LAT0))))

    def send_velocity_yawrate(self, vx, vy, vz, yaw_dps):
        th = math.radians(self.hd)
        self.n += (vx * math.cos(th) - vy * math.sin(th)) * DT
        self.e += (vx * math.sin(th) + vy * math.cos(th)) * DT
        self.hd += yaw_dps * DT
        loc = self.gps_location
        self.world.x, self.world.y, self.world.hd = self.frame.pose(loc.lat, loc.lon, self.heading)
        self.world.t += DT
        for r in self.world.red_rects:
            self.world.min_gap = min(self.world.min_gap, self.world.gap(r))

    def hover(self):
        self.hovered = True


def make_ctx(world, delivery_id="B7", target_visible=lambda: False, data=None):
    veh = FakeVehicle(world)
    cam = SimpleNamespace(pitch_deg=65.0, capture_array=lambda: None)
    ctx = SimpleNamespace(
        vehicle=veh, sd={}, data=data if data is not None else {}, delivery_id=delivery_id,
        altitude=SimpleNamespace(compute=lambda: 0.0, set_target=lambda m: None),
        rig=SimpleNamespace(view=lambda name: cam),
        qr=SimpleNamespace(find_target=lambda: object() if target_visible() else None),
        clock=lambda: world.t, log=lambda msg: None)
    return ctx


@pytest.fixture
def red_from_world(monkeypatch):
    holder = {}

    class FakeRed:
        def nearest(self, frame, height, pitch):
            return holder["world"].nearest_red()

    monkeypatch.setattr(handlers, "RedZoneDetector", FakeRed)
    return holder


def run_handler(ctx, limit_s=225.0):
    while ctx.vehicle.world.t < limit_s:
        nxt = handlers.h_search_delivery(ctx)
        if nxt is not None:
            return nxt
    return None


def test_handler_no_delivery_id_goes_home():
    ctx = make_ctx(World(), delivery_id=None)
    assert handlers.h_search_delivery(ctx) is State.FIND_BANNER_RTN


def test_handler_target_visible_goes_to_centering(red_from_world):
    w = World(); red_from_world["world"] = w
    ctx = make_ctx(w, target_visible=lambda: True)
    handlers.h_search_delivery(ctx)                       # first tick sets up
    assert handlers.h_search_delivery(ctx) is State.CENTER_OVER_QR


def test_handler_finds_target_when_it_comes_into_view(red_from_world):
    w = World(); red_from_world["world"] = w
    ctx = make_ctx(w, target_visible=lambda: w.x > 25.0 and w.y > 0.0)    # seen on the second half
    assert run_handler(ctx) is State.CENTER_OVER_QR
    assert w.t < STATE_TIMEOUTS["SEARCH_DELIVERY"]


def test_handler_no_target_finishes_path_before_the_timeout_and_stays_out_of_red(red_from_world):
    w = World(red_rects=[(14.0, 24.0, -6.0, -1.0)]); red_from_world["world"] = w
    ctx = make_ctx(w)
    assert run_handler(ctx) is State.FIND_BANNER_RTN
    assert w.t < STATE_TIMEOUTS["SEARCH_DELIVERY"]
    assert w.min_gap > 0.3
    assert ctx.vehicle.hovered


def test_handler_resume_keeps_zone_and_progress(red_from_world):
    w = World(); red_from_world["world"] = w
    ctx = make_ctx(w, target_visible=lambda: w.t > 40.0)
    assert run_handler(ctx) is State.CENTER_OVER_QR
    frame, planner = ctx.data["search"]
    progress = planner.i
    assert progress > 0
    ctx.sd.clear()                                         # the state machine wipes sd on entry
    ctx.qr.find_target = lambda: None
    handlers.h_search_delivery(ctx)
    assert ctx.data["search"][0] is frame and ctx.data["search"][1] is planner
    assert planner.i == progress


def test_state_machine_allows_the_transitions_the_handler_uses():
    allowed = allowed_next(State.SEARCH_DELIVERY)
    assert State.CENTER_OVER_QR in allowed and State.FIND_BANNER_RTN in allowed