# =============================================================================
# hardware/vehicle.py — Team Vajra AeroTHON 2026
#
# Thin wrapper around DroneKit. The rest of the codebase NEVER imports
# dronekit directly — they call methods on this class. This means:
#   • In simulation: connects to SITL
#   • On real hardware: connects to Pixhawk UART
#   • In unit tests: swap this class for MockVehicle (same interface)
#
# CHANGES vs previous version
#   [FIX] connect() was called twice (leftover "change this / to this" block)
#   [NEW] send_body_velocity()  — velocity RELATIVE TO DRONE HEADING
#         (forward / right / down). All vision + corridor control must use this.
#   [NEW] send_velocity_yawrate() — velocity + yaw rate in ONE message, so
#         banner alignment can yaw while holding position / altitude.
#   [FIX] send_ned_velocity() docstring: it is NORTH/EAST, not body frame.
#   [FIX] rangefinder_distance no longer crashes if distance is None.
#   [NEW] MockVehicle gets the same new methods + quiet mode.
# =============================================================================

import time
import math

from pymavlink import mavutil

# DroneKit compatibility shim for Python 3.10+
# dronekit uses collections.MutableMapping which was removed in 3.10
import collections
import collections.abc

for _name in ("Callable", "MutableMapping", "Mapping", "MutableSequence",
              "Sequence", "Iterable", "Iterator", "MutableSet"):
    if not hasattr(collections, _name):
        setattr(collections, _name, getattr(collections.abc, _name))

from dronekit import connect, VehicleMode, LocationGlobalRelative  # noqa: E402

# ── SET_POSITION_TARGET_LOCAL_NED type masks ──────────────────────────────────
# bit set = field IGNORED.  bits: 0-2 pos, 3-5 vel, 6-8 accel, 9 force,
#                                 10 yaw, 11 yaw_rate
_MASK_VEL_ONLY = 0b110111000111      # use vx,vy,vz            (3527)
_MASK_VEL_YAWRATE = 0b010111000111   # use vx,vy,vz + yaw_rate (1479)
_MASK_YAWRATE_ONLY = 0b010111111111  # use yaw_rate only       (1535)


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Ground distance in metres between two GPS points."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class Vehicle:
    """
    Thin wrapper around a DroneKit vehicle object.
    Translates our clean property names to DroneKit's actual API.
    All properties are read-only except through the explicit command methods.
    """

    def __init__(self, dk_vehicle):
        self._dk = dk_vehicle

    # ── identity / state ─────────────────────────────────────────────────────
    @property
    def is_armed(self) -> bool:
        return bool(self._dk.armed)

    @property
    def mode_name(self) -> str:
        return self._dk.mode.name

    # ── position ─────────────────────────────────────────────────────────────
    @property
    def altitude(self) -> float:
        """Barometric altitude above the takeoff point, metres."""
        return self._dk.location.global_relative_frame.alt or 0.0

    @property
    def heading(self) -> float:
        """Heading in degrees (0 = North, clockwise)."""
        return self._dk.heading or 0.0

    @property
    def gps_location(self):
        """Object with .lat, .lon, .alt (WGS84 altitude)."""
        return self._dk.location.global_frame

    # ── GPS health ───────────────────────────────────────────────────────────
    @property
    def gps_fix_type(self) -> int:
        """0=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK. Need ≥ 3 for flight."""
        gps = self._dk.gps_0
        return gps.fix_type if gps is not None else 0

    @property
    def gps_satellites(self) -> int:
        gps = self._dk.gps_0
        return gps.satellites_visible if gps is not None else 0

    # ── battery ──────────────────────────────────────────────────────────────
    @property
    def battery_level(self) -> float:
        """State of charge, 0–100. Returns 0 if not reported."""
        b = self._dk.battery
        return b.level if (b is not None and b.level is not None) else 0.0

    @property
    def battery_voltage(self) -> float:
        """Pack voltage in volts. Returns 0 if not reported."""
        b = self._dk.battery
        return b.voltage if (b is not None and b.voltage is not None) else 0.0

    @property
    def battery_known(self) -> bool:
        """False if the flight controller has never reported battery data."""
        b = self._dk.battery
        return b is not None and b.level is not None and b.voltage is not None

    # ── flight controller health ─────────────────────────────────────────────
    @property
    def heartbeat_age_s(self) -> float:
        """Seconds since the last heartbeat from the flight controller."""
        import time
        lhb = self._dk.last_heartbeat
        # DroneKit stores last_heartbeat as time.time() (wall clock), not monotonic
        return (time.time() - lhb) if lhb is not None else 999.0

    @property
    def ekf_ok(self) -> bool:
        return bool(self._dk.ekf_ok)

    # ── rangefinder ──────────────────────────────────────────────────────────
    @property
    def rangefinder_distance(self):
        """Distance in metres from the downward rangefinder, or None."""
        try:
            rf = self._dk.rangefinder
            return rf.distance if rf is not None else None
        except Exception:
            return None

    # ── commands ─────────────────────────────────────────────────────────────
    def arm(self):
        import time
        self._dk.mode = __import__('dronekit').VehicleMode("GUIDED")
        time.sleep(0.5)
        self._dk.armed = True

    def disarm(self):
        self._dk.armed = False

    def start_takeoff(self, altitude_m: float):
        self._dk.simple_takeoff(altitude_m)

    def land(self):
        import dronekit
        self._dk.mode = dronekit.VehicleMode("LAND")

    def rtl(self):
        import dronekit
        self._dk.mode = dronekit.VehicleMode("RTL")

    def goto(self, lat: float, lon: float, alt_m: float):
        import dronekit
        target = dronekit.LocationGlobalRelative(lat, lon, alt_m)
        self._dk.simple_goto(target)

    def hover(self):
        self.send_body_velocity(0.0, 0.0, 0.0)

    def send_body_velocity(self, vx: float, vy: float, vz_down: float):
        """
        Send a velocity command in the body frame.
        vx = forward m/s, vy = right m/s, vz_down = down m/s (positive = descend).
        Uses MAV_FRAME_BODY_OFFSET_NED.
        """
        from pymavlink import mavutil
        msg = self._dk.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,   # use velocity only, ignore position and acceleration
            0, 0, 0,              # position (ignored)
            vx, vy, vz_down,     # velocity
            0, 0, 0,              # acceleration (ignored)
            0, 0)                 # yaw, yaw rate (ignored)
        self._dk.send_mavlink(msg)
        self._dk.flush()

    def send_velocity_yawrate(self, vx: float, vy: float, vz_down: float,
                               yaw_rate_dps: float):
        """Body-frame velocity with yaw rate. yaw_rate_dps > 0 = clockwise."""
        import math
        from pymavlink import mavutil
        msg = self._dk.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000011111000111,   # use velocity + yaw rate
            0, 0, 0,
            vx, vy, vz_down,
            0, 0, 0,
            0, math.radians(yaw_rate_dps))
        self._dk.send_mavlink(msg)
        self._dk.flush()

    def set_servo(self, channel: int, pwm: int):
        msg = self._dk.message_factory.command_long_encode(
            0, 0,
            183,          # MAV_CMD_DO_SET_SERVO
            0,
            channel, pwm, 0, 0, 0, 0, 0)
        self._dk.send_mavlink(msg)

    def distance_to_m(self, lat: float, lon: float) -> float:
        """Horizontal distance in metres from the current GPS position to (lat, lon)."""
        import math
        loc = self._dk.location.global_frame
        if loc is None or loc.lat is None:
            return 0.0
        R = 6371000.0
        dlat = math.radians(lat - loc.lat)
        dlon = math.radians(lon - loc.lon)
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(loc.lat)) * math.cos(math.radians(lat)) *
             math.sin(dlon / 2) ** 2)
        return R * 2 * math.asin(math.sqrt(a))

    def verify_params(self, required: dict) -> bool:
        """Check ArduPilot parameters. Returns True if all match."""
        ok = True
        for name, expected in required.items():
            actual = self._dk.parameters.get(name)
            if actual != expected:
                ok = False
        return ok

    def close(self):
        self._dk.close()
# =============================================================================
# MockVehicle — identical interface, zero hardware needed.
# =============================================================================
class MockVehicle:
    """
    Drop-in replacement for Vehicle that logs commands instead of flying.
    verbose=False silences per-command printing (use in loops/tests); every
    command is still appended to self._commands.
    """

    def __init__(self, connection_string: str = "mock", verbose: bool = True,
                 **kwargs):
        self.verbose = verbose
        if verbose:
            print("[MockVehicle] Initialized (no hardware)")
        self._altitude = 0.0
        self._armed = False
        self._mode = "STABILIZE"
        self._battery = 100.0
        self._voltage = 16.8
        self._heading = 0.0
        self._lat = 0.0
        self._lon = 0.0
        self._commands = []
        self.last_body_cmd = (0.0, 0.0, 0.0)   # vx, vy, vz of last body cmd
        self.last_yaw_rate = 0.0
        self._start_time = time.time()
        # Phase 7b: health readings the safety layer looks at (healthy by default)
        self._gps_fix = 3
        self._gps_sats = 12
        self._heartbeat_age = 0.0
        self._ekf_ok = True
        self._battery_known = True


    def _log(self, cmd: str):
        t = time.time() - self._start_time
        entry = f"[{t:6.2f}s] {cmd}"
        self._commands.append(entry)
        if self.verbose:
            print(entry)

    # Arming / modes
    def set_mode(self, mode: str):
        self._mode = mode
        self._log(f"MODE → {mode}")

    def arm(self, timeout: float = 15.0):
        self._mode = "GUIDED"
        self._armed = True
        self._log("ARMED")

    def disarm(self):
        self._armed = False
        self._log("DISARMED")

    # Takeoff / land
    def takeoff(self, target_alt: float, timeout: float = 30.0):
        self._log(f"TAKEOFF → {target_alt}m")
        self._altitude = target_alt

    def start_takeoff(self, target_alt: float):
        self._log(f"TAKEOFF → {target_alt}m (non-blocking)")
        self._altitude = target_alt

    def goto(self, lat: float, lon: float, alt_rel: float):
        self._log(f"GOTO {lat:.6f},{lon:.6f} @ {alt_rel:.1f}m")
        self._lat, self._lon = lat, lon

    def distance_to_m(self, lat: float, lon: float) -> float:
        return haversine_m(self._lat, self._lon, lat, lon)

    def land(self):
        self._mode = "LAND"
        self._log("LAND")
        self._altitude = 0.0
        self._armed = False          # ArduPilot disarms itself after landing

    def rtl(self):
        self._mode = "RTL"
        self._log("RTL")
        self._altitude = 0.0         # mock: instantly home and landed
        self._armed = False

    # Velocity control
    def send_ned_velocity(self, vx, vy, vz, duration: float = 0.0):
        self._log(f"VEL NED  vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f}")
        if duration > 0:
            time.sleep(duration)

    def send_body_velocity(self, vx, vy, vz, duration: float = 0.0):
        self.last_body_cmd = (vx, vy, vz)
        self.last_yaw_rate = 0.0
        self._log(f"VEL BODY fwd={vx:+.2f} right={vy:+.2f} down={vz:+.2f}")
        if duration > 0:
            time.sleep(duration)

    def send_velocity_yawrate(self, vx, vy, vz, yaw_rate_dps):
        self.last_body_cmd = (vx, vy, vz)
        self.last_yaw_rate = yaw_rate_dps
        self._log(f"VEL BODY fwd={vx:+.2f} right={vy:+.2f} down={vz:+.2f} "
                  f"yawrate={yaw_rate_dps:+.1f}°/s")

    def hover(self):
        self.last_body_cmd = (0.0, 0.0, 0.0)
        self._log("HOVER (zero velocity)")

    def condition_yaw(self, heading_deg: float, relative: bool = False):
        self._log(f"YAW {'REL' if relative else 'ABS'} → {heading_deg}°")

    def send_yaw_rate(self, yaw_rate_dps: float):
        self.last_yaw_rate = yaw_rate_dps
        self._log(f"YAW_RATE → {yaw_rate_dps:+.1f} °/s")

    # Servo
    def set_servo(self, channel: int, pwm_us: int):
        self._log(f"SERVO ch{channel} = {pwm_us}µs")

    # Sensor reads
    @property
    def altitude(self) -> float:
        return self._altitude

    @altitude.setter
    def altitude(self, v: float):
        self._altitude = v

    @property
    def rangefinder_distance(self):
        return self._altitude if self._altitude < 8.0 else None

    @property
    def battery_level(self) -> float:
        return self._battery

    @property
    def battery_voltage(self) -> float:
        return self._voltage

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def mode_name(self) -> str:
        return self._mode

    @property
    def gps_location(self):
        class Loc:
            pass
        loc = Loc()
        loc.lat, loc.lon, loc.alt = self._lat, self._lon, self._altitude
        return loc

    @property
    def groundspeed(self) -> float:
        return 0.0

    # Phase 7b: health readings (same names as the real Vehicle)
    @property
    def gps_fix_type(self) -> int:
        return self._gps_fix

    @property
    def gps_satellites(self) -> int:
        return self._gps_sats

    @property
    def heartbeat_age_s(self) -> float:
        return self._heartbeat_age

    @property
    def ekf_ok(self) -> bool:
        return self._ekf_ok

    @property
    def battery_known(self) -> bool:
        return self._battery_known

    # Params
    def get_param(self, name: str):
        defaults = {"FS_THR_ENABLE": 1, "FS_GCS_ENABLE": 1,
                    "FS_BATT_ENABLE": 2, "FENCE_ENABLE": 1, "FENCE_ACTION": 1}
        return defaults.get(name, 0)

    def set_param(self, name: str, value):
        self._log(f"PARAM {name} = {value}")

    def verify_params(self, required: dict) -> bool:
        if self.verbose:
            print("[MockVehicle] Param check — all ✓ (mock always passes)")
        return True

    def close(self):
        self._log("CONNECTION CLOSED")
        if self.verbose:
            print(f"[MockVehicle] Total commands logged: {len(self._commands)}")
