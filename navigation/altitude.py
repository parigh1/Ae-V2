# =============================================================================
#  navigation/altitude.py  —  Team Vajra AeroTHON 2026
#
#  Altitude hold using a PID controller with sensor fusion:
#    - TF-Luna rangefinder: trusted below ALT_RANGEFINDER_MAX (8m)
#    - Barometer (via DroneKit): used above 8m
#    - Blended crossover region to prevent altitude spikes at transition
#
#  NED convention: vz NEGATIVE = climb, POSITIVE = descend.
#  This module always uses physical "positive = up" internally and
#  negates when calling send_ned_velocity.
# =============================================================================

import time
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config.params import (
    ALT_KP, ALT_KI, ALT_KD,
    ALT_ICLAMP, ALT_VZ_MAX,
    ALT_TOLERANCE, ALT_RANGEFINDER_MAX
)


class AltitudeController:
    """
    PID altitude hold. Call update() at your control loop rate (10–20 Hz).
    """

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.kp = ALT_KP
        self.ki = ALT_KI
        self.kd = ALT_KD
        self.i_clamp = ALT_ICLAMP
        self.vz_max  = ALT_VZ_MAX

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None
        self._target_alt = None

    def set_target(self, altitude_m: float):
        """Set a new altitude target. Resets integral."""
        self._target_alt = altitude_m
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None
        print(f"[Altitude] Target set → {altitude_m}m")

    def get_altitude(self) -> float:
        """
        Fused altitude reading.
        Below 8m: prefer rangefinder if available.
        Above 8m: use barometer.
        Crossover 7–9m: blend both to prevent spike.
        """
        baro = self.vehicle.altitude
        rf   = self.vehicle.rangefinder_distance

        if rf is None:
            return baro  # no rangefinder — baro only

        if baro <= 7.0:
            return rf          # fully trust rangefinder
        elif baro >= 9.0:
            return baro        # fully trust barometer
        else:
            # linear blend in 7–9m transition zone
            alpha = (baro - 7.0) / 2.0   # 0.0 at 7m → 1.0 at 9m
            return (1.0 - alpha) * rf + alpha * baro

    def update(self) -> float:
        """
        Compute PID output and send velocity command to vehicle.
        Returns the vz command sent (negative = climb).
        Call this at ~10–20 Hz.
        """
        if self._target_alt is None:
            return 0.0

        now  = time.time()
        if self._prev_time is None:
            self._prev_time = now
            return 0.0

        dt    = now - self._prev_time
        if dt < 0.001:
            return 0.0
        self._prev_time = now

        current = self.get_altitude()
        error   = self._target_alt - current   # positive = need to climb

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        i_raw  = self.ki * self._integral
        i_term = max(-self.i_clamp, min(self.i_clamp, i_raw))
        if i_raw != i_term:                    # clamp hit — reset integral
            self._integral = i_term / self.ki

        # Derivative
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Total output (physical: positive = climb)
        climb_rate = p_term + i_term + d_term
        climb_rate = max(-self.vz_max, min(self.vz_max, climb_rate))

        # NED: vz NEGATIVE = climb
        vz_ned = -climb_rate
        self.vehicle.send_ned_velocity(0, 0, vz_ned)

        return vz_ned

    def reached_target(self, tolerance: float = ALT_TOLERANCE) -> bool:
        """Returns True when within tolerance of target altitude."""
        if self._target_alt is None:
            return False
        return abs(self.get_altitude() - self._target_alt) < tolerance

    def hold_until_reached(self, altitude_m: float,
                           timeout: float = 20.0,
                           loop_hz: float = 10.0) -> bool:
        """
        Convenience: set target, run PID loop until reached or timeout.
        Returns True on success, False on timeout.
        """
        self.set_target(altitude_m)
        deadline = time.time() + timeout
        dt = 1.0 / loop_hz

        while time.time() < deadline:
            self.update()
            current = self.get_altitude()
            print(f"[Altitude] {current:.2f}m → {altitude_m}m  "
                  f"(err: {altitude_m - current:+.2f}m)")
            if self.reached_target():
                print(f"[Altitude] ✓ Reached {altitude_m}m")
                return True
            time.sleep(dt)

        print(f"[Altitude] ✗ Timeout reaching {altitude_m}m "
              f"(stuck at {self.get_altitude():.2f}m)")
        return False
