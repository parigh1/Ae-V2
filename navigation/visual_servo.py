# =============================================================================
# navigation/visual_servo.py — Team Vajra AeroTHON 2026
#
# Image-based visual servoing: drive the drone until the target is centred
# under it (within SERVO_TOLERANCE for SERVO_CONFIRM_FRAMES in a row).
#
# Input is the target position in the BODY frame, (forward_m, right_m) — what
# QRSystem.find_target() / PixelToMeters.offset_body() produce. Positive means
# "target is ahead / to the right", so a PD law on these values flies TOWARD it.
#
# Commands use vehicle.send_body_velocity(): velocity relative to heading, so
# this works at any yaw (the old LOCAL_NED version only worked facing north).
# Altitude can be held in the same message via an optional vz provider.
# =============================================================================

import math
import time
from typing import Callable, Optional, Tuple

from config.params import (
    SERVO_KP, SERVO_KD, SERVO_TOLERANCE, SERVO_MAX_SPEED,
    SERVO_TIMEOUT, SERVO_CONFIRM_FRAMES, SERVO_LOST_FRAMES_MAX, CONTROL_HZ,
)

Offset = Tuple[float, float]          # (forward_m, right_m)


class VisualServo:
    def __init__(self, vehicle, clock=time.monotonic, sleep=time.sleep):
        self.vehicle = vehicle
        self._clock = clock
        self._sleep = sleep
        self.kp = SERVO_KP
        self.kd = SERVO_KD
        self.max_speed = SERVO_MAX_SPEED
        self.tolerance = SERVO_TOLERANCE
        self.reset()

    def reset(self):
        self._prev: Optional[Offset] = None
        self._prev_t: Optional[float] = None

    # ── Pure control law (easy to unit-test) ──────────────────────────────────
    def compute(self, forward_m: float, right_m: float, now: float) -> Offset:
        """PD on the body-frame offset. Returns (vx_forward, vy_right) in m/s,
        speed-limited by vector magnitude so direction is preserved."""
        if self._prev is None or self._prev_t is None or now <= self._prev_t:
            d_f = d_r = 0.0                      # no derivative on first sample
        else:
            dt = now - self._prev_t
            d_f = (forward_m - self._prev[0]) / dt
            d_r = (right_m - self._prev[1]) / dt
        self._prev, self._prev_t = (forward_m, right_m), now

        vx = self.kp * forward_m + self.kd * d_f
        vy = self.kp * right_m + self.kd * d_r

        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx, vy = vx * scale, vy * scale
        return vx, vy

    def is_centered(self, forward_m: float, right_m: float) -> bool:
        return abs(forward_m) < self.tolerance and abs(right_m) < self.tolerance

    # ── Closed loop ───────────────────────────────────────────────────────────
    def run(self,
            get_target: Callable[[], Optional[Offset]],
            vz_provider: Optional[Callable[[], float]] = None,
            timeout: float = SERVO_TIMEOUT,
            hz: float = CONTROL_HZ) -> bool:
        """
        get_target(): returns (forward_m, right_m) or None if the target is not
                      visible this frame.
        vz_provider(): optional, returns NED vz (e.g. AltitudeController.compute)
                       so altitude is held while centering.
        Returns True once centred for SERVO_CONFIRM_FRAMES consecutive frames;
        False on timeout or after SERVO_LOST_FRAMES_MAX frames without target.
        The vehicle is left hovering in both cases.
        """
        self.reset()
        period = 1.0 / hz
        deadline = self._clock() + timeout
        centered_frames = 0
        lost_frames = 0

        while self._clock() < deadline:
            vz = vz_provider() if vz_provider else 0.0
            target = get_target()

            if target is None:
                lost_frames += 1
                centered_frames = 0
                self.reset()                      # stale derivative is useless
                self.vehicle.send_body_velocity(0.0, 0.0, vz)   # hold, keep altitude
                if lost_frames > SERVO_LOST_FRAMES_MAX:
                    print("[Servo] ✗ target lost")
                    self.vehicle.hover()
                    return False
            else:
                lost_frames = 0
                fwd, right = target
                if self.is_centered(fwd, right):
                    centered_frames += 1
                    self.vehicle.send_body_velocity(0.0, 0.0, vz)
                    if centered_frames >= SERVO_CONFIRM_FRAMES:
                        print(f"[Servo] ✓ centred (fwd {fwd:+.2f} m, right {right:+.2f} m)")
                        self.vehicle.hover()
                        return True
                else:
                    centered_frames = 0
                    vx, vy = self.compute(fwd, right, self._clock())
                    self.vehicle.send_body_velocity(vx, vy, vz)

            self._sleep(period)

        print("[Servo] ✗ timeout")
        self.vehicle.hover()
        return False
