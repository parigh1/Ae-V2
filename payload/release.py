# =============================================================================
# payload/release.py — Team Vajra AeroTHON 2026
#
# Drops the payload. Two styles, chosen by params.PAYLOAD_MODE:
#   "gripper_drop" : open the gripper where the drone hovers (report, section 4.1)
#   "winch"        : lower on a line, release on the ground, wind the line back
#                    (rulebook Figure 3 wording)
#
# It never blocks: call step(now) once per control tick until it returns True.
# =============================================================================
from typing import Callable, List, Optional, Tuple

from config.params import (
    PAYLOAD_MODE, PAYLOAD_RELEASE_WAIT_S,
    GRIPPER_PWM_CHANNEL, PWM_GRIPPER_OPEN, PWM_GRIPPER_CLOSED,
    WINCH_PWM_CHANNEL, PWM_WINCH_LOWER, PWM_WINCH_RAISE, PWM_WINCH_STOP,
    WINCH_LOWER_TIME, WINCH_RETRACT_TIME,
)


class PayloadReleaser:
    def __init__(self, vehicle, mode: Optional[str] = None):
        self.vehicle = vehicle
        self.mode = mode or PAYLOAD_MODE
        if self.mode not in ("gripper_drop", "winch"):
            raise ValueError(f"PAYLOAD_MODE must be 'gripper_drop' or 'winch', got '{self.mode}'")
        self._plan = self._build_plan()
        self._i = -1
        self._t_phase: Optional[float] = None

    # Each phase = (what to do, how long to wait afterwards, in seconds)
    def _build_plan(self) -> List[Tuple[Callable[[], None], float]]:
        v = self.vehicle
        open_grip = lambda: v.set_servo(GRIPPER_PWM_CHANNEL, PWM_GRIPPER_OPEN)
        if self.mode == "gripper_drop":
            return [(open_grip, PAYLOAD_RELEASE_WAIT_S)]
        return [
            (lambda: v.set_servo(WINCH_PWM_CHANNEL, PWM_WINCH_LOWER), WINCH_LOWER_TIME),
            (lambda: v.set_servo(WINCH_PWM_CHANNEL, PWM_WINCH_STOP), 0.2),
            (open_grip, PAYLOAD_RELEASE_WAIT_S),
            (lambda: v.set_servo(WINCH_PWM_CHANNEL, PWM_WINCH_RAISE), WINCH_RETRACT_TIME),
            (lambda: v.set_servo(WINCH_PWM_CHANNEL, PWM_WINCH_STOP), 0.0),
        ]

    def hold(self):
        """Close the gripper on the payload (call during preflight)."""
        self.vehicle.set_servo(GRIPPER_PWM_CHANNEL, PWM_GRIPPER_CLOSED)

    def step(self, now: float) -> bool:
        """Advance the release sequence. Returns True when it has finished."""
        if self._i >= len(self._plan):
            return True
        if self._i == -1 or now - self._t_phase >= self._plan[self._i][1]:
            self._i += 1
            if self._i >= len(self._plan):
                return True
            self._plan[self._i][0]()
            self._t_phase = now
            if self._i == len(self._plan) - 1 and self._plan[self._i][1] == 0.0:
                self._i += 1
                return True
        return False

    @property
    def finished(self) -> bool:
        return self._i >= len(self._plan)
