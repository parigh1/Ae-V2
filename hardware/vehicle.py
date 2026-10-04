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

from dronekit import connect, VehicleMode  # noqa: E402

# ── SET_POSITION_TARGET_LOCAL_NED type masks ──────────────────────────────────
# bit set = field IGNORED.  bits: 0-2 pos, 3-5 vel, 6-8 accel, 9 force,
#                                 10 yaw, 11 yaw_rate
_MASK_VEL_ONLY = 0b110111000111      # use vx,vy,vz            (3527)
_MASK_VEL_YAWRATE = 0b010111000111   # use vx,vy,vz + yaw_rate (1479)
_MASK_YAWRATE_ONLY = 0b010111111111  # use yaw_rate only       (1535)


class Vehicle:
    """
    Wraps a DroneKit vehicle connection.
    All velocity commands, mode changes, and sensor reads go through here.
    """

    def __init__(self, connection_string: str, baud: int = 115200,
                 wait_ready: bool = True):
        print(f"[Vehicle] Connecting to: {connection_string}")
        self._dk = connect(
            connection_string,
            baud=baud,
            wait_ready=wait_ready,
            heartbeat_timeout=60,
            rate=4,
        )

    # ── Arming / modes ────────────────────────────────────────────────────────
    def set_mode(self, mode: str):
        self._dk.mode = VehicleMode(mode)
        deadline = time.time() + 10
        while self._dk.mode.name != mode:
            if time.time() > deadline:
                raise RuntimeError(f"[Vehicle] Mode change to {mode} timed out")
            time.sleep(0.1)
        print(f"[Vehicle] Mode: {mode}")

    def arm(self, timeout: float = 15.0):
        """Arms the vehicle. Requires GUIDED mode first."""
        self.set_mode("GUIDED")
        self._dk.armed = True
        deadline = time.time() + timeout
        while not self._dk.armed:
            if time.time() > deadline:
                raise RuntimeError("[Vehicle] Arming timed out")
            print("[Vehicle] Waiting for arm...")
            time.sleep(0.5)
        print("[Vehicle] Armed ✓")

    def disarm(self):
        self._dk.armed = False
        time.sleep(1)
        print("[Vehicle] Disarmed")

    # ── Takeoff / land ────────────────────────────────────────────────────────
    def takeoff(self, target_alt: float, timeout: float = 30.0):
        """Simple takeoff to target_alt (metres AGL). Blocks until ~92% reached."""
        print(f"[Vehicle] Taking off to {target_alt}m")
        self._dk.simple_takeoff(target_alt)
        deadline = time.time() + timeout
        while True:
            alt = self.altitude
            print(f"[Vehicle] Altitude: {alt:.2f}m / {target_alt}m")
            if alt >= target_alt * 0.92:
                print(f"[Vehicle] Takeoff complete at {alt:.2f}m")
                return
            if time.time() > deadline:
                raise RuntimeError(f"[Vehicle] Takeoff timed out at {alt:.2f}m")
            time.sleep(0.5)

    def land(self):
        self.set_mode("LAND")
        print("[Vehicle] Landing initiated")

    def rtl(self):
        self.set_mode("RTL")
        print("[Vehicle] RTL initiated")

    # ── Velocity control ──────────────────────────────────────────────────────
    def _send_vel(self, frame, mask, vx, vy, vz, yaw_rate_rad=0.0):
        msg = self._dk.message_factory.set_position_target_local_ned_encode(
            0,          # time_boot_ms (unused)
            0, 0,       # target system, component
            frame,
            mask,
            0, 0, 0,    # position (ignored)
            vx, vy, vz,
            0, 0, 0,    # acceleration (ignored)
            0, yaw_rate_rad,
        )
        self._dk.send_mavlink(msg)

    def send_ned_velocity(self, vx: float, vy: float, vz: float,
                          duration: float = 0.0):
        """
        Velocity in the LOCAL NED frame (NOT relative to heading):
          vx +North, vy +East, vz +DOWN (negative = climb).
        Use send_body_velocity() for anything driven by the camera/LiDAR.
        duration > 0: re-sends every 0.1 s for that long (blocking).
        """
        def once():
            self._send_vel(mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                           _MASK_VEL_ONLY, vx, vy, vz)
        if duration <= 0:
            once()
            return
        deadline = time.time() + duration
        while time.time() < deadline:
            once()
            time.sleep(0.1)

    def send_body_velocity(self, vx: float, vy: float, vz: float,
                           duration: float = 0.0):
        """
        Velocity RELATIVE TO THE DRONE'S HEADING (MAV_FRAME_BODY_OFFSET_NED):
          vx +forward (nose), vy +right, vz +DOWN (negative = climb).
        This is the frame vision / corridor controllers must use.
        ArduCopter expires velocity setpoints after ~1 s, so call at ≥ 5 Hz.
        """
        def once():
            self._send_vel(mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                           _MASK_VEL_ONLY, vx, vy, vz)
        if duration <= 0:
            once()
            return
        deadline = time.time() + duration
        while time.time() < deadline:
            once()
            time.sleep(0.1)

    def send_velocity_yawrate(self, vx: float, vy: float, vz: float,
                              yaw_rate_dps: float):
        """Body-frame velocity AND yaw rate (deg/s, + = clockwise) in one message."""
        self._send_vel(mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                       _MASK_VEL_YAWRATE, vx, vy, vz,
                       math.radians(yaw_rate_dps))

    def hover(self):
        """Stop all motion — zero body-frame velocity."""
        self.send_body_velocity(0, 0, 0)

    # ── Yaw control ───────────────────────────────────────────────────────────
    def condition_yaw(self, heading_deg: float, relative: bool = False):
        """
        Yaw to heading_deg. relative=True rotates BY heading_deg (negative =
        counter-clockwise); relative=False goes to an absolute compass heading.
        """
        is_relative = 1 if relative else 0
        direction = 1
        angle = abs(heading_deg) if relative else heading_deg
        if relative and heading_deg < 0:
            direction = -1
        msg = self._dk.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle, 20,      # target angle, yaw rate deg/s
            direction, is_relative,
            0, 0, 0,
        )
        self._dk.send_mavlink(msg)

    def send_yaw_rate(self, yaw_rate_dps: float):
        """Yaw-rate-only command (deg/s, + = clockwise)."""
        self._send_vel(mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                       _MASK_YAWRATE_ONLY, 0, 0, 0,
                       math.radians(yaw_rate_dps))

    # ── Servo / PWM output (winch + gripper) ──────────────────────────────────
    def set_servo(self, channel: int, pwm_us: int):
        """Set a servo channel to a PWM value (µs). AUX channels start at 9."""
        msg = self._dk.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel, pwm_us,
            0, 0, 0, 0, 0,
        )
        self._dk.send_mavlink(msg)

    # ── Sensor reads ──────────────────────────────────────────────────────────
    @property
    def altitude(self) -> float:
        """Altitude relative to home from barometer/EKF (metres)."""
        return self._dk.location.global_relative_frame.alt or 0.0

    @property
    def rangefinder_distance(self):
        """TF-Luna downward distance (m), or None if unavailable/invalid."""
        rf = self._dk.rangefinder
        dist = getattr(rf, "distance", None) if rf else None
        if dist is not None and dist > 0:
            return dist
        return None

    @property
    def battery_level(self) -> float:
        b = self._dk.battery
        return b.level if b and b.level is not None else 100.0

    @property
    def battery_voltage(self) -> float:
        b = self._dk.battery
        return b.voltage if b and b.voltage is not None else 16.8

    @property
    def heading(self) -> float:
        return self._dk.heading or 0.0

    @property
    def is_armed(self) -> bool:
        return self._dk.armed

    @property
    def mode_name(self) -> str:
        return self._dk.mode.name

    @property
    def gps_location(self):
        return self._dk.location.global_relative_frame

    @property
    def groundspeed(self) -> float:
        return self._dk.groundspeed or 0.0

    # ── MAVLink parameter read/write ──────────────────────────────────────────
    def get_param(self, name: str):
        return self._dk.parameters[name]

    def set_param(self, name: str, value):
        self._dk.parameters[name] = value
        time.sleep(0.1)

    def verify_params(self, required: dict) -> bool:
        """True if all required ArduPilot params match; prints mismatches."""
        all_ok = True
        for name, expected in required.items():
            try:
                actual = self.get_param(name)
                if abs(actual - expected) > 0.01:
                    print(f"[Safety] ✗ MISMATCH {name}: expected {expected}, got {actual}")
                    all_ok = False
                else:
                    print(f"[Safety] ✓ {name} = {actual}")
            except Exception as e:
                print(f"[Safety] ✗ Could not read {name}: {e}")
                all_ok = False
        return all_ok

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def close(self):
        self._dk.close()
        print("[Vehicle] Connection closed")


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
        self._commands = []
        self.last_body_cmd = (0.0, 0.0, 0.0)   # vx, vy, vz of last body cmd
        self.last_yaw_rate = 0.0
        self._start_time = time.time()

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
        self._armed = True
        self._log("ARMED")

    def disarm(self):
        self._armed = False
        self._log("DISARMED")

    # Takeoff / land
    def takeoff(self, target_alt: float, timeout: float = 30.0):
        self._log(f"TAKEOFF → {target_alt}m")
        self._altitude = target_alt

    def land(self):
        self._log("LAND")
        self._altitude = 0.0

    def rtl(self):
        self._log("RTL")

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
            lat, lon, alt = 0.0, 0.0, 0.0
        return Loc()

    @property
    def groundspeed(self) -> float:
        return 0.0

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
