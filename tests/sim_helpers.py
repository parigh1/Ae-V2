# Tiny kinematic simulator used by unit tests (no hardware, no SITL).
import math

from hardware.vehicle import MockVehicle


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t


class SimVehicle(MockVehicle):
    """MockVehicle that integrates body-frame velocity commands in a NED world.
    Time is simulated: call advance(dt) (use it as the `sleep` function)."""

    def __init__(self, clock: SimClock, heading_deg=0.0, lag=0.3):
        super().__init__(verbose=False)
        self.clock = clock
        self._heading = heading_deg
        self.n = 0.0           # north, m
        self.e = 0.0           # east, m
        self.v_fwd = 0.0       # actual (lagged) body velocity
        self.v_right = 0.0
        self.lag = lag         # first-order command lag, seconds

    def advance(self, dt):
        a = dt / (self.lag + dt)
        cmd_f, cmd_r, _ = self.last_body_cmd
        self.v_fwd += a * (cmd_f - self.v_fwd)
        self.v_right += a * (cmd_r - self.v_right)
        psi = math.radians(self._heading)
        self.n += (self.v_fwd * math.cos(psi) - self.v_right * math.sin(psi)) * dt
        self.e += (self.v_fwd * math.sin(psi) + self.v_right * math.cos(psi)) * dt
        self.clock.t += dt

    def target_in_body(self, tn, te):
        """(forward, right) of a world target, exactly."""
        dn, de = tn - self.n, te - self.e
        psi = math.radians(self._heading)
        fwd = dn * math.cos(psi) + de * math.sin(psi)
        right = -dn * math.sin(psi) + de * math.cos(psi)
        return fwd, right


class FlightSim(SimVehicle):
    """SimVehicle that also climbs/descends: follows vz commands and non-blocking takeoff."""

    def __init__(self, clock, heading_deg=0.0, lag=0.3):
        super().__init__(clock, heading_deg, lag)
        self._takeoff_target = None

    def start_takeoff(self, target_alt):
        self._log(f"TAKEOFF → {target_alt}m (non-blocking)")
        self._takeoff_target = target_alt

    def advance(self, dt):
        super().advance(dt)
        if self._takeoff_target is not None:
            self._altitude = min(self._takeoff_target, self._altitude + 1.5 * dt)
            if self._altitude >= self._takeoff_target:
                self._takeoff_target = None
        else:
            vz = self.last_body_cmd[2]                 # NED: negative = climb
            self._altitude = max(0.0, self._altitude - vz * dt)
        self._heading = (self._heading + self.last_yaw_rate * dt) % 360.0   # + = clockwise


class BannerWorldCamera:
    """Fake FORWARD camera: draws the green banner where it would appear given the
    drone's heading. bearing_deg = compass direction of the banner from the drone."""

    def __init__(self, veh, bearing_deg=None, width=1280, height=720, hfov_deg=84.0):
        import math
        self.veh, self.bearing, self.w, self.h = veh, bearing_deg, width, height
        self.fx = (width / 2) / math.tan(math.radians(hfov_deg / 2))
        self.half_fov = hfov_deg / 2

    def capture_array(self):
        import math
        import numpy as np
        frame = np.full((self.h, self.w, 3), 90, np.uint8)             # grey background
        if self.bearing is None:
            return frame
        delta = (self.bearing - self.veh.heading + 180) % 360 - 180     # + = banner on the right
        if abs(delta) < self.half_fov - 8:
            cx = int(self.w / 2 + self.fx * math.tan(math.radians(delta)))
            x0, x1 = max(0, cx - 150), min(self.w, cx + 150)
            frame[250:450, x0:x1] = (30, 160, 60)                       # the banner
            frame[300:330, x0 + 20:x1 - 20] = (240, 240, 240)           # "text" stripe
        return frame
