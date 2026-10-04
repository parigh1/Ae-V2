# =============================================================================
# safety/watchdog.py — Team Vajra AeroTHON 2026
#
# check() is called by the state machine every tick. It answers:
#   None                          everything fine
#   SafetyEvent("emergency", why) abort the mission and RTL
#   SafetyEvent("stop", why)      the pilot took over -> STOP commanding the drone
#
# The pilot's RC mode switch must ALWAYS win. If the flight mode is anything
# other than GUIDED / RTL / LAND while armed, we assume a human flipped it and
# the Pi stops sending commands (otherwise it would fight the pilot).
# =============================================================================

from dataclasses import dataclass
from typing import Optional

from config.params import BATTERY_RTL_PCT, VOLTAGE_RTL_V, BATTERY_WARN_PCT

ALLOWED_MODES = ("GUIDED", "RTL", "LAND")


@dataclass
class SafetyEvent:
    action: str      # "emergency" | "stop"
    reason: str


class Watchdog:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._warned = False

    def check(self) -> Optional[SafetyEvent]:
        v = self.vehicle
        if not v.is_armed:
            return None                       # on the ground / not started: nothing to watch

        if v.mode_name not in ALLOWED_MODES:
            return SafetyEvent("stop", f"pilot_override (mode {v.mode_name})")

        pct, volts = v.battery_level, v.battery_voltage
        if pct < BATTERY_RTL_PCT or volts < VOLTAGE_RTL_V:
            return SafetyEvent("emergency", f"low_battery ({pct:.0f}%, {volts:.1f} V)")
        if pct < BATTERY_WARN_PCT and not self._warned:
            print(f"[Safety] WARNING battery {pct:.0f}%")
            self._warned = True
        return None
