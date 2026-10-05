import itertools
import math

import pytest

from config import params
from hardware.camera import build_camera_rig
from mission.context import MissionContext
from mission.handlers import corridor_state
from mission.states import State
from navigation.altitude import AltitudeController
from navigation.corridor import (CorridorController, DistanceTracker, Obstacle, VisionObstacleSensor,
                                 LidarObstacleSensor, default_sensors, nearest_obstacle)
from tests.corridor_world import CorridorWorldCamera
from tests.sim_helpers import SimClock, FlightSim
from vision.corridor_detector import CorridorDetector, CorridorFix

PITCH = params.CAM_MOUNTS["single_fixed"]["forward"]


def drone_at(n=-1.0, e=0.0, heading=0.0, altitude=3.0):
    veh = FlightSim(SimClock())
    veh.altitude = altitude
    veh.n, veh.e, veh._heading = n, e, heading % 360
    return veh


def true_values(e, heading):
    """what the detector SHOULD say for a drone at lateral position e (corridor centre at 0)."""
    return -e * math.cos(math.radians(heading)), -heading


# ── wall detection ───────────────────────────────────────────────────────────
@pytest.mark.parametrize("e,heading", list(itertools.product([-1.2, -0.6, 0.0, 0.6, 1.2], [-20, 0, 20])))
def test_detector_matches_truth(e, heading):
    veh = drone_at(e=e, heading=heading)
    fix = CorridorDetector().detect(CorridorWorldCamera(veh, PITCH).capture_array(), 3.0, PITCH)
    centre, head = true_values(e, heading)
    assert fix is not None
    assert fix.center_offset_m == pytest.approx(centre, abs=0.2)
    assert fix.heading_error_deg == pytest.approx(head, abs=3.0)


@pytest.mark.parametrize("hidden,expected", [("left", "right"), ("right", "left")])
def test_one_wall_is_enough(hidden, expected):
    veh = drone_at(e=0.4)
    fix = CorridorDetector().detect(CorridorWorldCamera(veh, PITCH, hide=[hidden]).capture_array(), 3.0, PITCH)
    assert fix is not None and fix.walls == expected
    assert fix.center_offset_m == pytest.approx(-0.4, abs=0.25)


def test_no_walls_means_none():
    veh = drone_at()
    cam = CorridorWorldCamera(veh, PITCH, hide=["left", "right"])
    assert CorridorDetector().detect(cam.capture_array(), 3.0, PITCH) is None


def test_works_at_other_camera_tilts_too():
    for pitch in (60.0, 75.0, 90.0):
        veh = drone_at(e=0.5)
        fix = CorridorDetector().detect(CorridorWorldCamera(veh, pitch).capture_array(), 3.0, pitch)
        assert fix is not None and fix.center_offset_m == pytest.approx(-0.5, abs=0.25), pitch


# ── controller ───────────────────────────────────────────────────────────────
def fix_(offset=0.0, heading=0.0, walls="both", left=1.75, right=1.75):
    return CorridorFix(offset, heading, left, right, walls, 1.0)


def test_lost_means_full_stop():
    cmd = CorridorController().compute(None, None, 0.0)
    assert (cmd.vx, cmd.vy, cmd.yaw_rate_dps, cmd.status) == (0.0, 0.0, 0.0, "lost")


def test_steers_toward_the_centre_and_turns_toward_the_axis():
    cmd = CorridorController().compute(fix_(offset=+0.5, heading=+10), None, 0.0)
    assert cmd.vy > 0 and cmd.yaw_rate_dps > 0 and cmd.vx > 0
    cmd = CorridorController().compute(fix_(offset=-0.5, heading=-10), None, 0.0)
    assert cmd.vy < 0 and cmd.yaw_rate_dps < 0


def test_limits_are_respected():
    cmd = CorridorController().compute(fix_(offset=5.0, heading=90), None, 0.0)
    assert cmd.vy <= params.CORRIDOR_LAT_VMAX + 1e-9
    assert cmd.yaw_rate_dps <= params.CORRIDOR_YAW_MAX_DPS + 1e-9


def test_slows_down_when_far_off_centre_or_with_one_wall():
    full = CorridorController().compute(fix_(), None, 0.0).vx
    assert CorridorController().compute(fix_(offset=1.0), None, 0.0).vx < full
    assert CorridorController().compute(fix_(walls="left"), None, 0.0).vx < full


def test_obstacle_ahead_slows_then_stops():
    far = Obstacle(params.CORRIDOR_OBSTACLE_SLOW_M + 1, 0.0, 0.0, "vision")
    mid = Obstacle((params.CORRIDOR_OBSTACLE_SLOW_M + params.CORRIDOR_OBSTACLE_STOP_M) / 2, 0.0, 0.0, "vision")
    close = Obstacle(params.CORRIDOR_OBSTACLE_STOP_M - 0.2, 0.0, 0.0, "vision")
    v_far = CorridorController().compute(fix_(), far, 0.0).vx
    c_mid = CorridorController().compute(fix_(), mid, 0.0)
    c_close = CorridorController().compute(fix_(), close, 0.0)
    assert v_far == pytest.approx(params.CORRIDOR_FWD_SPEED)
    assert 0 < c_mid.vx < v_far and c_mid.status == "obstacle_slow"
    assert c_close.vx == 0.0 and c_close.status == "obstacle_stop"


def test_obstacle_on_the_right_makes_us_shift_left_and_vice_versa():
    right_ob = Obstacle(2.5, 0.0, +0.5, "vision")        # straddling, middle slightly right
    left_ob = Obstacle(2.5, 0.0, -0.5, "vision")
    assert CorridorController().compute(fix_(), right_ob, 0.0).vy < 0
    assert CorridorController().compute(fix_(), left_ob, 0.0).vy > 0


def test_obstacle_beside_our_line_is_ignored():
    beside = Obstacle(1.0, params.CORRIDOR_OBSTACLE_CLEAR_M + 0.3, 1.5, "vision")
    cmd = CorridorController().compute(fix_(), beside, 0.0)
    assert cmd.vx == pytest.approx(params.CORRIDOR_FWD_SPEED) and cmd.status == "ok"


def test_lidar_style_obstacle_stops_without_choosing_a_side():
    cmd = CorridorController().compute(fix_(), Obstacle(1.0, 0.0, None, "lidar"), 0.0)
    assert cmd.vx == 0.0


def test_sidestep_never_pushes_into_a_wall():
    tight = fix_(offset=0.0, left=1.0, right=1.0)
    cmd = CorridorController().compute(tight, Obstacle(2.5, 0.0, 0.0, "lidar"), 0.0)
    assert abs(cmd.vy) <= params.CORRIDOR_LAT_KP * max(0.0, 1.0 - params.CORRIDOR_WALL_MARGIN_M) + 1e-9 + params.CORRIDOR_I_CLAMP


def test_sidestep_is_held_until_the_obstacle_is_passed():
    c = CorridorController()
    t = 0.0
    c.compute(fix_(), Obstacle(2.5, 0.0, +0.5, "vision"), t)     # decides to go left and latches
    shifted = []
    for _ in range(200):                                         # obstacle now out of sight
        t += 0.1
        shifted.append(c.compute(fix_(offset=0.0), None, t).vy)
    assert shifted[0] < 0                                        # still holding the shifted line
    assert shifted[-1] == pytest.approx(0.0, abs=0.02)           # ...and released once past


# ── distance ─────────────────────────────────────────────────────────────────
def test_distance_prefers_gps_when_it_agrees():
    t = DistanceTracker()
    assert t.update(0.4, 1.0, 0.5) == 0.5 and not t.disagreement


def test_distance_without_gps_is_dead_reckoning():
    t = DistanceTracker()
    for _ in range(10):
        t.update(0.4, 1.0, None)
    assert t.distance_m == pytest.approx(4.0)


def test_distance_takes_the_smaller_when_they_disagree():
    t = DistanceTracker()
    t.update(0.4, 1.0, 20.0)
    assert t.distance_m == pytest.approx(0.4) and t.disagreement
    t = DistanceTracker()
    for _ in range(20):
        t.update(0.4, 1.0, 0.0)           # GPS stuck at zero
    assert t.distance_m == 0.0 and t.disagreement


# ── obstacles from the camera / LiDAR ────────────────────────────────────────
def test_vision_sees_a_dark_box_in_the_lane():
    veh = drone_at(e=0.0)
    cam = CorridorWorldCamera(veh, PITCH, obstacles=[(2.0, -0.2, 0.4, 0.4)])
    ob = VisionObstacleSensor().measure(cam.capture_array(), 3.0, PITCH)
    assert ob is not None and ob.source == "vision"
    assert ob.forward_m == pytest.approx(3.0, abs=0.7)           # drone at n=-1, box front at n=2
    assert ob.right_m == 0.0                                      # straddles the centre line


def test_vision_reports_nothing_in_an_empty_corridor():
    veh = drone_at()
    assert VisionObstacleSensor().measure(CorridorWorldCamera(veh, PITCH).capture_array(), 3.0, PITCH) is None


class FakeLidar:
    def __init__(self, d):
        self.d = d

    def distance_m(self):
        return self.d


def test_lidar_plugs_in_through_the_same_door():
    veh = drone_at()
    frame = CorridorWorldCamera(veh, PITCH, obstacles=[(2.0, -0.2, 0.4, 0.4)]).capture_array()
    sensors = default_sensors(lidar=FakeLidar(2.0))
    assert len(sensors) == 2
    ob = nearest_obstacle(sensors, frame, 3.0, PITCH)
    assert ob.source == "lidar" and ob.forward_m == 2.0          # nearest wins
    assert len(default_sensors()) == 1                            # camera only by default
    assert LidarObstacleSensor(FakeLidar(None)).measure(frame, 3.0, PITCH) is None


# ── closed loop: fly the corridor in the simulator ───────────────────────────
def fly_corridor(e0, heading0, obstacles=(), hide=(), max_s=90.0):
    clock = SimClock()
    veh = FlightSim(clock, lag=0.3)
    veh.altitude = 3.0
    veh.n, veh.e, veh._heading = -1.0, e0, heading0 % 360
    cam = CorridorWorldCamera(veh, PITCH, obstacles=obstacles, hide=hide)
    rig = build_camera_rig(mode="single_fixed", cameras={"main": cam})
    ctx = MissionContext(veh, rig=rig, altitude=AltitudeController(veh, clock=clock.now),
                         clock=clock.now, sleep=veh.advance)
    ctx.mission_t0 = clock.now()
    handler = corridor_state(State.CORRIDOR_FORWARD)
    ctx.state = State.CORRIDOR_FORWARD
    track = []
    while clock.now() < max_s:
        out = handler(ctx)
        track.append((veh.n, veh.e, veh.heading))
        if out is not None:
            return out, track, ctx, veh
        veh.advance(0.1)
    return None, track, ctx, veh


@pytest.mark.parametrize("e0,heading0", [(0.0, 0), (1.0, 10), (-1.0, -10), (1.0, -15), (-0.8, 15)])
def test_drone_flies_the_whole_corridor_without_touching_a_wall(e0, heading0):
    out, track, ctx, veh = fly_corridor(e0, heading0)
    assert out == State.CLIMB_TO_DELIVERY
    assert veh.n > params.CORRIDOR_LENGTH_M - 0.5                       # actually got through
    inside = [(n, e) for n, e, _ in track if 0.0 <= n <= params.CORRIDOR_LENGTH_M]
    clearance = params.CORRIDOR_WIDTH / 2 - max(abs(e) for _, e in inside)
    assert clearance > 0.35, f"came within {clearance:.2f} m of a wall"
    n_mid, e_mid, h_mid = next(t for t in track if t[0] >= 8.0)
    assert abs(e_mid) < 0.35 and abs((h_mid + 180) % 360 - 180) < 6.0   # settled in the middle, pointing straight
    assert ctx.clock() < params.STATE_TIMEOUTS["CORRIDOR_FORWARD"]


def test_flies_with_only_one_wall_visible():
    out, track, ctx, veh = fly_corridor(0.5, 0, hide=["right"])
    assert out == State.CLIMB_TO_DELIVERY
    assert max(abs(e) for n, e, _ in track if 0 <= n <= 10) < 1.4


def test_side_steps_around_an_obstacle_without_hitting_it():
    box = (5.0, 0.15, 0.5, 0.6)                    # n, e, size_n, size_e: right of centre
    out, track, ctx, veh = fly_corridor(0.0, 0, obstacles=[box])
    assert out == State.CLIMB_TO_DELIVERY
    n0, e0_, sn, se = box
    for n, e, _ in track:
        if n0 - 0.3 <= n <= n0 + sn + 0.3:         # while level with the box
            assert not (e0_ - 0.3 <= e <= e0_ + se + 0.3), f"hit the box at n={n:.1f}, e={e:.2f}"
    assert any("obstacle" in line for line in ctx.events)


def test_walls_vanishing_at_the_end_ends_the_state():
    out, track, ctx, veh = fly_corridor(0.0, 0)
    assert any("corridor finished" in line for line in ctx.events)


def test_blind_drone_stops_and_waits_for_the_state_timeout():
    clock = SimClock()
    veh = FlightSim(clock)
    veh.altitude = 3.0
    cam = CorridorWorldCamera(veh, PITCH, hide=["left", "right"])
    rig = build_camera_rig(mode="single_fixed", cameras={"main": cam})
    ctx = MissionContext(veh, rig=rig, altitude=AltitudeController(veh, clock=clock.now),
                         clock=clock.now, sleep=veh.advance)
    ctx.mission_t0 = clock.now()
    handler = corridor_state(State.CORRIDOR_FORWARD)
    for _ in range(100):
        assert handler(ctx) is None                # never "finishes" at distance 0
        veh.advance(0.1)
    assert veh.n == pytest.approx(0.0, abs=0.1) or abs(veh.v_fwd) < 0.05