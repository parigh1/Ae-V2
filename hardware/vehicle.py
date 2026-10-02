# =============================================================================
#  hardware/vehicle.py  —  Team Vajra AeroTHON 2026
#
#  Thin wrapper around DroneKit. The rest of the codebase NEVER imports
#  dronekit directly — they call methods on this class. This means:
#    • In simulation: connects to SITL at tcp:127.0.0.1:5760
#    • On real hardware: connects to Pixhawk UART
#    • In unit tests: swap this class for MockVehicle (same interface)
# =============================================================================

import time
import math
import threading
from pymavlink import mavutil

# DroneKit compatibility shim for Python 3.12+
# dronekit uses collections.MutableMapping which was removed in 3.10
import collections
import collections.abc
for name in ("Callable", "MutableMapping", "Mapping", "MutableSequence",
             "Sequence", "Iterable", "Iterator", "MutableSet"):
    if not hasattr(collections, name):
        setattr(collections, name, getattr(collections.abc, name))

import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative


class Vehicle:
    """
    Wraps a DroneKit vehicle connection.
    All NED velocity commands, mode changes, and sensor reads go through here.
    """

    def __init__(self, connection_string: str, baud: int = 115200,
                 wait_ready: bool = True):
        print(f"[Vehicle] Connecting to: {connection_string}")
        # change this:
        self._dk = connect(
            connection_string,
            baud=baud,
            wait_ready=wait_ready,
            heartbeat_timeout=30
        )

        # to this:
        self._dk = connect(
            connection_string,
            baud=baud,
            wait_ready=wait_ready,
            heartbeat_timeout=60,
            rate=4
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
        """
        Simple takeoff to target_alt (metres AGL).
        Blocks until within ALT_TOLERANCE of target.
        """
        print(f"[Vehicle] Taking off to {target_alt}m")
        self._dk.simple_takeoff(target_alt)
        deadline = time.time() + timeout
        while True:
            alt = self.altitude
            print(f"[Vehicle] Altitude: {alt:.2f}m / {target_alt}m")
            if alt >= target_alt * 0.92:   # 92% threshold = reached
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

    # ── Velocity control (NED frame) ──────────────────────────────────────────

    def send_ned_velocity(self, vx: float, vy: float, vz: float,
                          duration: float = 0.0):
        """
        Send a body-frame NED velocity command.
          vx: positive = North (forward when heading 0)
          vy: positive = East  (right when heading 0)
          vz: positive = DOWN  (use negative to climb!)

        If duration > 0: sends command and blocks for that many seconds,
        refreshing every 0.1s (DroneKit velocity commands expire after ~1s).
        If duration == 0: sends once (caller handles the refresh loop).
        """
        msg = self._dk.message_factory.set_position_target_local_ned_encode(
            0,          # time_boot_ms (not used)
            0, 0,       # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask: only velocity components used
            0, 0, 0,    # x, y, z positions (ignored)
            vx, vy, vz, # velocities
            0, 0, 0,    # accelerations (ignored)
            0, 0        # yaw, yaw_rate (ignored)
        )
        if duration <= 0:
            self._dk.send_mavlink(msg)
            return

        deadline = time.time() + duration
        while time.time() < deadline:
            self._dk.send_mavlink(msg)
            time.sleep(0.1)

    def hover(self):
        """Stop all motion — send zero velocity."""
        self.send_ned_velocity(0, 0, 0)

    # ── Yaw control ───────────────────────────────────────────────────────────

    def condition_yaw(self, heading_deg: float, relative: bool = False):
        """
        Yaw to heading_deg.
        relative=True: rotate by heading_deg from current heading.
        relative=False: rotate to absolute compass heading.
        """
        is_relative = 1 if relative else 0
        direction = 1  # 1=CW, -1=CCW; positive heading = CW
        msg = self._dk.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading_deg, 20,   # target angle, yaw rate deg/s
            direction, is_relative,
            0, 0, 0
        )
        self._dk.send_mavlink(msg)

    def send_yaw_rate(self, yaw_rate_dps: float):
        """
        Continuous yaw rate command (degrees/second).
        Positive = clockwise when viewed from above.
        """
        msg = self._dk.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000010111111111,  # type_mask: only yaw rate used
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, math.radians(yaw_rate_dps)
        )
        self._dk.send_mavlink(msg)

    # ── Servo / PWM output (winch + gripper) ──────────────────────────────────

    def set_servo(self, channel: int, pwm_us: int):
        """
        Set a servo channel to a specific PWM value (microseconds).
        channel: 1-16 (AUX channels start at 9 on Pixhawk)
        pwm_us: typically 1000–2000µs
        """
        msg = self._dk.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            channel, pwm_us,
            0, 0, 0, 0, 0
        )
        self._dk.send_mavlink(msg)

    # ── Sensor reads ──────────────────────────────────────────────────────────

    @property
    def altitude(self) -> float:
        """Altitude AGL from barometer (metres). Always available in SITL."""
        return self._dk.location.global_relative_frame.alt or 0.0

    @property
    def rangefinder_distance(self) -> float:
        """
        TF-Luna downward rangefinder distance (metres).
        Returns None if no rangefinder connected (SITL default).
        Use altitude property as fallback when this is None.
        """
        rf = self._dk.rangefinder
        if rf and rf.distance > 0:
            return rf.distance
        return None

    @property
    def battery_level(self) -> float:
        """Battery remaining as percentage (0–100)."""
        b = self._dk.battery
        return b.level if b and b.level is not None else 100.0

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in volts."""
        b = self._dk.battery
        return b.voltage if b and b.voltage is not None else 16.8

    @property
    def heading(self) -> float:
        """Current compass heading in degrees (0–359)."""
        return self._dk.heading or 0.0

    @property
    def is_armed(self) -> bool:
        return self._dk.armed

    @property
    def gps_location(self):
        """Returns DroneKit LocationGlobalRelative."""
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
        """
        Checks all required ArduPilot parameters.
        Returns True if all match, prints mismatches and returns False otherwise.
        """
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
#  MockVehicle — identical interface, zero hardware needed.
#  Use this for unit tests and pure logic testing.
# =============================================================================

class MockVehicle:
    """
    Drop-in replacement for Vehicle that logs commands instead of flying.
    All properties return safe default values.
    Supports scripted altitude profiles for testing altitude PID.
    """

    def __init__(self, connection_string: str = "mock", **kwargs):
        print(f"[MockVehicle] Initialized (no hardware)")
        self._altitude   = 0.0
        self._armed      = False
        self._mode       = "STABILIZE"
        self._battery    = 100.0
        self._voltage    = 16.8
        self._heading    = 0.0
        self._commands   = []   # log of all commands sent
        # Scripted altitude: list of (time_offset, altitude) tuples
        self._alt_script = []
        self._start_time = time.time()

    # ── Command logging ───────────────────────────────────────────────────────

    def _log(self, cmd: str):
        t = time.time() - self._start_time
        entry = f"[{t:6.2f}s] {cmd}"
        self._commands.append(entry)
        print(entry)

    # ── Arming / modes ────────────────────────────────────────────────────────

    def set_mode(self, mode: str):
        self._mode = mode
        self._log(f"MODE → {mode}")

    def arm(self, timeout: float = 15.0):
        self._armed = True
        self._log("ARMED")

    def disarm(self):
        self._armed = False
        self._log("DISARMED")

    # ── Takeoff / land ────────────────────────────────────────────────────────

    def takeoff(self, target_alt: float, timeout: float = 30.0):
        self._log(f"TAKEOFF → {target_alt}m")
        self._altitude = target_alt  # instant in mock

    def land(self):
        self._log("LAND")
        self._altitude = 0.0

    def rtl(self):
        self._log("RTL")

    # ── Velocity control ──────────────────────────────────────────────────────

    def send_ned_velocity(self, vx: float, vy: float, vz: float,
                          duration: float = 0.0):
        self._log(f"VEL NED  vx={vx:+.2f}  vy={vy:+.2f}  vz={vz:+.2f}")
        if duration > 0:
            time.sleep(duration)

    def hover(self):
        self._log("HOVER (zero velocity)")

    def condition_yaw(self, heading_deg: float, relative: bool = False):
        mode = "REL" if relative else "ABS"
        self._log(f"YAW {mode} → {heading_deg}°")

    def send_yaw_rate(self, yaw_rate_dps: float):
        self._log(f"YAW_RATE → {yaw_rate_dps:+.1f} °/s")

    # ── Servo ─────────────────────────────────────────────────────────────────

    def set_servo(self, channel: int, pwm_us: int):
        self._log(f"SERVO ch{channel} = {pwm_us}µs")

    # ── Sensor reads ──────────────────────────────────────────────────────────

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
    def gps_location(self):
        class Loc:
            lat, lon, alt = 0.0, 0.0, 0.0
        return Loc()

    @property
    def groundspeed(self) -> float:
        return 0.0

    # ── Params ────────────────────────────────────────────────────────────────

    def get_param(self, name: str):
        defaults = {
            "FS_THR_ENABLE": 1, "FS_GCS_ENABLE": 1,
            "FS_BATT_ENABLE": 2, "FENCE_ENABLE": 1, "FENCE_ACTION": 1
        }
        return defaults.get(name, 0)

    def set_param(self, name: str, value):
        self._log(f"PARAM {name} = {value}")

    def verify_params(self, required: dict) -> bool:
        print("[MockVehicle] Param check — all ✓ (mock always passes)")
        return True

    def close(self):
        self._log("CONNECTION CLOSED")
        print(f"[MockVehicle] Total commands logged: {len(self._commands)}")
