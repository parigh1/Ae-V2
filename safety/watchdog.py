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
#
# Phase 7b: besides the battery it now also watches
#   - the Pixhawk heartbeat (is the Pi still talking to the flight controller?)
#   - the battery readings themselves (missing data must never look like "100 %")
#   - the GPS fix
#   - a software geofence (max distance from the take-off point, max height)
# Every "bad for N seconds" check is debounced so one glitch cannot abort a flight.
# =============================================================================

import time
from dataclasses import dataclass
from typing import Optional

from config.params import (
    BATTERY_RTL_PCT, VOLTAGE_RTL_V, BATTERY_WARN_PCT,
    HEARTBEAT_LOST_S, BATTERY_UNKNOWN_S, GPS_LOST_S, PREFLIGHT_MIN_GPS_FIX,
    PY_FENCE_RADIUS_M, PY_FENCE_MAX_ALT_M, PY_FENCE_DEBOUNCE_S,
)

ALLOWED_MODES = ("GUIDED", "RTL", "LAND")


@dataclass
class SafetyEvent:
    action: str      # "emergency" | "stop"
    reason: str


class Watchdog:
    def __init__(self, vehicle, clock=time.monotonic):
        self.vehicle = vehicle
        self.clock = clock
        self._warned = False
        self._home = None                 # (lat, lon) where we were first seen armed = the take-off point
        self._since = {}                  # problem name -> time it started (for the debounce)

    def _bad_for(self, name, is_bad, seconds, now) -> bool:
        """True once `is_bad` has been true continuously for `seconds`."""
        if not is_bad:
            self._since.pop(name, None)
            return False
        start = self._since.setdefault(name, now)
        return now - start >= seconds

    def check(self) -> Optional[SafetyEvent]:
        v = self.vehicle
        if not v.is_armed:
            return None                       # on the ground / not started: nothing to watch

        if v.mode_name not in ALLOWED_MODES:
            return SafetyEvent("stop", f"pilot_override (mode {v.mode_name})")

        try:
            return self._check_flight_health(v)
        except Exception as exc:              # noqa: BLE001 - a broken check must not crash the flight, nor pass silently
            return SafetyEvent("emergency", f"watchdog_error ({exc!r})")

    def _check_flight_health(self, v) -> Optional[SafetyEvent]:
        now = self.clock()

        # 1. link to the flight controller
        age = v.heartbeat_age_s
        if age > HEARTBEAT_LOST_S:
            return SafetyEvent("emergency", f"link_lost (no heartbeat for {min(age, 999):.1f} s)")

        # 2. battery (missing readings are NOT "100 %")
        if self._bad_for("battery_unknown", not v.battery_known, BATTERY_UNKNOWN_S, now):
            return SafetyEvent("emergency", "battery_unknown (no battery data from the flight controller)")
        if v.battery_known:
            pct, volts = v.battery_level, v.battery_voltage
            if pct < BATTERY_RTL_PCT or volts < VOLTAGE_RTL_V:
                return SafetyEvent("emergency", f"low_battery ({pct:.0f}%, {volts:.1f} V)")
            if pct < BATTERY_WARN_PCT and not self._warned:
                print(f"[Safety] WARNING battery {pct:.0f}%")
                self._warned = True

        # 3. GPS
        if self._bad_for("gps_lost", v.gps_fix_type < PREFLIGHT_MIN_GPS_FIX, GPS_LOST_S, now):
            return SafetyEvent("emergency", f"gps_lost (fix type {v.gps_fix_type})")

        # 4. software geofence
        loc = v.gps_location
        have_position = loc.lat is not None and loc.lon is not None
        if have_position and self._home is None:
            self._home = (loc.lat, loc.lon)
        if have_position and self._home is not None:
            dist = v.distance_to_m(*self._home)
            if self._bad_for("fence_radius", dist > PY_FENCE_RADIUS_M, PY_FENCE_DEBOUNCE_S, now):
                return SafetyEvent("emergency", f"geofence ({dist:.0f} m from take-off, limit {PY_FENCE_RADIUS_M:.0f} m)")
        if self._bad_for("fence_alt", v.altitude > PY_FENCE_MAX_ALT_M, PY_FENCE_DEBOUNCE_S, now):
            return SafetyEvent("emergency", f"max_altitude ({v.altitude:.1f} m, limit {PY_FENCE_MAX_ALT_M:.0f} m)")
        return None