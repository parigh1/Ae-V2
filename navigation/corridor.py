# navigation/corridor.py — steering through the 3.5 m corridor (camera only)
# 1. obstacle sensors  2. CorridorController  3. DistanceTracker  (pure logic, no hardware)
from dataclasses import dataclass
from typing import List, Optional

import cv2

from config.params import (
    CAM_IMG_W, CAM_IMG_H, CORRIDOR_WIDTH, CORRIDOR_FWD_SPEED,
    CORRIDOR_LAT_KP, CORRIDOR_LAT_KI, CORRIDOR_LAT_KD, CORRIDOR_LAT_VMAX, CORRIDOR_I_CLAMP,
    CORRIDOR_YAW_KP, CORRIDOR_YAW_MAX_DPS, CORRIDOR_SLOW_LATERAL_M, CORRIDOR_WALL_MARGIN_M,
    CORRIDOR_OBSTACLE_SLOW_M, CORRIDOR_OBSTACLE_STOP_M, CORRIDOR_OBSTACLE_CLEAR_M,
    CORRIDOR_SIDESTEP_M, CORRIDOR_OBSTACLE_PASS_M,
    OBSTACLE_V_MAX, OBSTACLE_MIN_AREA, OBSTACLE_MAX_RANGE_M,
)
from vision.color_masks import odd_kernel
from vision.corridor_detector import CorridorFix
from vision.pixel_to_meters import PixelToMeters


# ── 1. obstacle sensors ───────────────────────────────────────────────────────
@dataclass
class Obstacle:
    forward_m: float                    # distance to its nearest point
    right_m: float                      # sideways position of the edge NEAREST our centre line (0 = straight ahead)
    center_right_m: Optional[float]     # sideways position of its middle (None = sensor cannot tell)
    source: str


class VisionObstacleSensor:
    """Dark objects standing on the ground (the rulebook's black bars)."""

    def measure(self, frame, altitude_m: float, pitch_deg: float) -> Optional[Obstacle]:
        h, w = frame.shape[:2]
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_RGB2HSV), (0, 0, 0), (179, 255, OBSTACLE_V_MAX))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, odd_kernel(7, w, CAM_IMG_W))
        min_area = OBSTACLE_MIN_AREA * (w * h) / (CAM_IMG_W * CAM_IMG_H)
        best = None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            foot = y + bh                                              # where it meets the ground
            pts = [PixelToMeters.offset_body(px - w / 2.0, foot - h / 2.0, altitude_m, w, h, pitch_deg)
                   for px in (x, x + bw / 2.0, x + bw)]
            if any(p is None for p in pts):
                continue
            fwd = min(p[0] for p in pts)
            left_edge, mid, right_edge = pts[0][1], pts[1][1], pts[2][1]
            if left_edge <= 0.0 <= right_edge:
                near_side = 0.0                                        # straddles our centre line
            else:
                near_side = left_edge if abs(left_edge) < abs(right_edge) else right_edge
            if 0.0 < fwd <= OBSTACLE_MAX_RANGE_M and (best is None or fwd < best.forward_m):
                best = Obstacle(fwd, near_side, mid, "vision")
        return best


class LidarObstacleSensor:
    """For later: wrap any object that has distance_m() (e.g. a TF-Luna driver)."""

    def __init__(self, lidar):
        self.lidar = lidar

    def measure(self, frame, altitude_m, pitch_deg) -> Optional[Obstacle]:
        d = self.lidar.distance_m()
        return None if d is None else Obstacle(d, 0.0, None, "lidar")  # one beam: straight ahead only


def default_sensors(lidar=None) -> List[object]:
    """Camera always. To add a LiDAR later: default_sensors(lidar=my_driver)."""
    sensors: List[object] = [VisionObstacleSensor()]
    if lidar is not None:
        sensors.append(LidarObstacleSensor(lidar))
    return sensors


def nearest_obstacle(sensors, frame, altitude_m: float, pitch_deg: float) -> Optional[Obstacle]:
    found = [o for o in (s.measure(frame, altitude_m, pitch_deg) for s in sensors) if o is not None]
    return min(found, key=lambda o: o.forward_m) if found else None


# ── 2. controller ─────────────────────────────────────────────────────────────
def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class CorridorCommand:
    vx: float              # forward m/s
    vy: float              # sideways m/s (+ = right)
    yaw_rate_dps: float    # turn rate (+ = clockwise)
    status: str            # ok | single_wall | lost | obstacle_slow | obstacle_stop | obstacle_passing


class CorridorController:
    def __init__(self):
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_e: Optional[float] = None
        self._prev_t: Optional[float] = None
        # Once we decide to go around something we STAY on the shifted line until we have
        # flown past it (otherwise we drift clear, forget it, swing back and bump into it).
        self._latch: Optional[dict] = None

    def _sidestep_bias(self, fix: CorridorFix, ob: Obstacle) -> float:
        """Metres to shift the aim point (+ = right) to get around the obstacle."""
        side = ob.center_right_m
        if side is None or abs(side) < 0.05:           # unknown / dead centre: use the roomier side
            right_room = fix.right_m if fix.right_m is not None else CORRIDOR_WIDTH / 2 + fix.center_offset_m
            left_room = fix.left_m if fix.left_m is not None else CORRIDOR_WIDTH / 2 - fix.center_offset_m
            direction = 1.0 if right_room >= left_room else -1.0
        else:
            direction = -1.0 if side > 0 else 1.0      # away from the obstacle
        room = fix.right_m if direction > 0 else fix.left_m
        limit = CORRIDOR_SIDESTEP_M if room is None else max(0.0, room - CORRIDOR_WALL_MARGIN_M)
        return direction * min(CORRIDOR_SIDESTEP_M, limit)

    def compute(self, fix: Optional[CorridorFix], ob: Optional[Obstacle], now: float) -> CorridorCommand:
        if fix is None:                                 # blind: stop dead
            self.reset()
            return CorridorCommand(0.0, 0.0, 0.0, "lost")

        status = "ok" if fix.walls == "both" else "single_wall"
        e = fix.center_offset_m
        relevant = (ob is not None and abs(ob.right_m) < CORRIDOR_OBSTACLE_CLEAR_M
                    and ob.forward_m < CORRIDOR_OBSTACLE_SLOW_M)
        if relevant and self._latch is None:
            self._latch = {"bias": self._sidestep_bias(fix, ob), "travel": 0.0,
                           "need": ob.forward_m + CORRIDOR_OBSTACLE_PASS_M}
        if self._latch is not None:
            e += self._latch["bias"]

        dt = None if self._prev_t is None else max(1e-3, now - self._prev_t)
        if dt is not None:
            self._integral = _clamp(self._integral + CORRIDOR_LAT_KI * e * dt, -CORRIDOR_I_CLAMP, CORRIDOR_I_CLAMP)
        d_term = 0.0 if dt is None or self._prev_e is None else CORRIDOR_LAT_KD * (e - self._prev_e) / dt
        self._prev_e, self._prev_t = e, now
        vy = _clamp(CORRIDOR_LAT_KP * e + self._integral + d_term, -CORRIDOR_LAT_VMAX, CORRIDOR_LAT_VMAX)
        yaw = _clamp(CORRIDOR_YAW_KP * fix.heading_error_deg, -CORRIDOR_YAW_MAX_DPS, CORRIDOR_YAW_MAX_DPS)

        vx = CORRIDOR_FWD_SPEED
        if abs(fix.center_offset_m) > CORRIDOR_SLOW_LATERAL_M:
            vx *= 0.5
        if fix.walls != "both":
            vx *= 0.7
        if relevant:
            if ob.forward_m <= CORRIDOR_OBSTACLE_STOP_M:
                vx, status = 0.0, "obstacle_stop"
            else:
                vx *= (ob.forward_m - CORRIDOR_OBSTACLE_STOP_M) / (CORRIDOR_OBSTACLE_SLOW_M - CORRIDOR_OBSTACLE_STOP_M)
                status = "obstacle_slow"
        if self._latch is not None:
            if dt is not None:
                self._latch["travel"] += vx * dt
            if self._latch["travel"] >= self._latch["need"]:
                self._latch = None                      # past it: back to the middle
                self._integral = 0.0                    # forget the windup from the detour
            elif status == "ok":
                status = "obstacle_passing"
        return CorridorCommand(vx, vy, yaw, status)


# ── 3. distance ───────────────────────────────────────────────────────────────
class DistanceTracker:
    """GPS distance from the entry point when it agrees with dead reckoning (commanded
    speed x time) to within `trust_m`; otherwise the SMALLER of the two (leaving late is
    safer than leaving early)."""

    def __init__(self, trust_m: float = 4.0):
        self.dead_m = 0.0
        self.distance_m = 0.0
        self.trust_m = trust_m
        self.disagreement = False

    def update(self, vx: float, dt: float, gps_distance_m: Optional[float]) -> float:
        self.dead_m += max(0.0, vx) * dt
        if gps_distance_m is None:
            self.distance_m = self.dead_m
        elif abs(gps_distance_m - self.dead_m) <= self.trust_m:
            self.distance_m, self.disagreement = gps_distance_m, False
        else:
            self.distance_m, self.disagreement = min(gps_distance_m, self.dead_m), True
        return self.distance_m