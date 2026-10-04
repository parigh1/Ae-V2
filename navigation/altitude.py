# =============================================================================
# navigation/altitude.py — Team Vajra AeroTHON 2026
#
# Altitude hold using a PID controller with sensor fusion:
#   - TF-Luna rangefinder: trusted below ALT_RANGEFINDER_MAX - blend width
#   - Barometer (via DroneKit): used above ALT_RANGEFINDER_MAX + blend width
#   - Blended crossover region to prevent altitude spikes at the transition
#
# NED convention: vz NEGATIVE = climb, POSITIVE = descend.
# Internally this module works in "positive = up" and negates at the end.
#
# CHANGES vs previous version
#   [NEW] compute()  — runs the PID and RETURNS vz, but sends nothing.
#         State handlers combine it with lateral commands and send ONE
#         message:  vehicle.send_body_velocity(vx, vy, alt.compute()).
#   [FIX] update() no longer exists as the only entry point; it is now
#         compute() + send, and sends vx=vy=0 (altitude-only phases).
#         Calling update() while a corridor/servo controller is running would
#         overwrite its lateral command with zeros — don't.
#   [FIX] blend zone now comes from params (ALT_RANGEFINDER_MAX ± width).
#   [FIX] removed sys.path hack; run from the project root (PyCharm does this).
# =============================================================================

import time

from config.params import (
    ALT_KP, ALT_KI, ALT_KD,
    ALT_ICLAMP, ALT_VZ_MAX,
    ALT_TOLERANCE, ALT_RANGEFINDER_MAX, ALT_BLEND_HALF_WIDTH,
    CONTROL_HZ,
)


class AltitudeController:
    """PID altitude hold. Call compute()/update() at 10–20 Hz."""

    def __init__(self, vehicle, clock=time.monotonic):
        self.vehicle = vehicle
        self._clock = clock
        self.kp = ALT_KP
        self.ki = ALT_KI
        self.kd = ALT_KD
        self.i_clamp = ALT_ICLAMP
        self.vz_max = ALT_VZ_MAX
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._target_alt = None

    # ── Target ────────────────────────────────────────────────────────────────
    @property
    def target(self):
        return self._target_alt

    def set_target(self, altitude_m: float):
        """Set a new altitude target. Resets integral and derivative history."""
        self._target_alt = altitude_m
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        print(f"[Altitude] Target set → {altitude_m}m")

    # ── Sensor fusion ─────────────────────────────────────────────────────────
    def get_altitude(self) -> float:
        """
        Fused altitude (m).
        Below (MAX - w): rangefinder.  Above (MAX + w): barometer.
        In between: linear blend. No rangefinder → barometer only.
        """
        baro = self.vehicle.altitude
        rf = self.vehicle.rangefinder_distance
        if rf is None:
            return baro

        lo = ALT_RANGEFINDER_MAX - ALT_BLEND_HALF_WIDTH
        hi = ALT_RANGEFINDER_MAX + ALT_BLEND_HALF_WIDTH
        if baro <= lo:
            return rf
        if baro >= hi:
            return baro
        alpha = (baro - lo) / (hi - lo)          # 0 at lo → 1 at hi
        return (1.0 - alpha) * rf + alpha * baro

    # ── PID ───────────────────────────────────────────────────────────────────
    def compute(self) -> float:
        """
        Run one PID step and RETURN the NED vz command (negative = climb).
        Sends nothing. Returns 0.0 on the first call after set_target()
        (no dt yet) or if no target is set.
        """
        if self._target_alt is None:
            return 0.0

        now = self._clock()
        if self._prev_time is None:
            self._prev_time = now
            self._prev_error = self._target_alt - self.get_altitude()
            return 0.0

        dt = now - self._prev_time
        if dt < 1e-3:
            return 0.0
        self._prev_time = now

        error = self._target_alt - self.get_altitude()   # + = need to climb

        p_term = self.kp * error

        # Integral with anti-windup clamp (clamp on output, back-calculate state)
        self._integral += error * dt
        i_raw = self.ki * self._integral
        i_term = max(-self.i_clamp, min(self.i_clamp, i_raw))
        if i_raw != i_term and self.ki != 0:
            self._integral = i_term / self.ki

        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        climb_rate = p_term + i_term + d_term
        climb_rate = max(-self.vz_max, min(self.vz_max, climb_rate))
        return -climb_rate

    def update(self) -> float:
        """compute() + send an altitude-only command (vx = vy = 0)."""
        vz = self.compute()
        self.vehicle.send_body_velocity(0.0, 0.0, vz)
        return vz

    # ── Status helpers ────────────────────────────────────────────────────────
    def reached_target(self, tolerance: float = ALT_TOLERANCE) -> bool:
        if self._target_alt is None:
            return False
        return abs(self.get_altitude() - self._target_alt) < tolerance

    def hold_until_reached(self, altitude_m: float, timeout: float = 20.0,
                           loop_hz: float = CONTROL_HZ,
                           sleep=time.sleep) -> bool:
        """Blocking helper: set target, loop update() until reached or timeout."""
        self.set_target(altitude_m)
        deadline = self._clock() + timeout
        dt = 1.0 / loop_hz
        while self._clock() < deadline:
            self.update()
            current = self.get_altitude()
            print(f"[Altitude] {current:.2f}m → {altitude_m}m "
                  f"(err: {altitude_m - current:+.2f}m)")
            if self.reached_target():
                print(f"[Altitude] ✓ Reached {altitude_m}m")
                return True
            sleep(dt)
        print(f"[Altitude] ✗ Timeout reaching {altitude_m}m "
              f"(stuck at {self.get_altitude():.2f}m)")
        return False
