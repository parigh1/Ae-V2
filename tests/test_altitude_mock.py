from navigation.altitude import AltitudeController
from tests.sim_helpers import SimClock, SimVehicle


def run_climb(target, start=0.0, secs=40.0):
    clock = SimClock()
    veh = SimVehicle(clock)
    veh.altitude = start
    ctrl = AltitudeController(veh, clock=clock.now)
    ctrl.set_target(target)
    dt = 0.1
    for _ in range(int(secs / dt)):
        vz = ctrl.compute()
        veh.send_body_velocity(0, 0, vz)
        veh.altitude += -vz * dt           # NED: negative vz = climb
        veh.advance(dt)
    return veh, ctrl


def test_climbs_to_5m():
    veh, ctrl = run_climb(5.0)
    assert abs(veh.altitude - 5.0) < 0.15


def test_descends_10_to_5():
    veh, ctrl = run_climb(5.0, start=10.0)
    assert abs(veh.altitude - 5.0) < 0.15


def test_compute_does_not_send():
    clock = SimClock()
    veh = SimVehicle(clock)
    ctrl = AltitudeController(veh, clock=clock.now)
    ctrl.set_target(5.0)
    before = len(veh._commands)
    ctrl.compute(); clock.t += 0.1; ctrl.compute()
    assert len(veh._commands) == before


def test_blend_no_jump_at_crossover():
    clock = SimClock()
    veh = SimVehicle(clock)
    ctrl = AltitudeController(veh, clock=clock.now)
    veh.altitude = 7.0     # rangefinder_distance == altitude (<8) in the mock
    a = ctrl.get_altitude()
    veh.altitude = 7.99
    b = ctrl.get_altitude()
    assert abs(b - a) < 1.5
