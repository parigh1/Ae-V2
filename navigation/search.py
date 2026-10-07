import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from config.params import (SEARCH_ZONE_W, SEARCH_ZONE_H, SEARCH_STRIP_W, SEARCH_SPEED, SEARCH_ENTRY_FROM_LEFT_M,
    SEARCH_MARGIN_M, SEARCH_WP_TOL_M, SEARCH_HEADING_TOL_DEG, SEARCH_SPEED_KP, SEARCH_MIN_SPEED,
    SEARCH_YAW_KP, SEARCH_YAW_MAX_DPS, SEARCH_SLIDE_SPEED, SEARCH_SLIDE_TRIGGER_M, SEARCH_SLIDE_HOLD_S,
    RED_INFLUENCE_M)
from navigation.red_zone_avoidance import adjust_velocity
from vision.red_zone import NearestRed

M_PER_DEG = 111_320.0
def wrap180(a): return (a + 180.0) % 360.0 - 180.0

class LocalFrame:
    def __init__(self, lat0, lon0, heading0_deg): self.lat0, self.lon0, self.heading0 = lat0, lon0, heading0_deg
    def pose(self, lat, lon, heading_deg):
        n = (lat - self.lat0) * M_PER_DEG
        e = (lon - self.lon0) * M_PER_DEG * math.cos(math.radians(self.lat0))
        psi = math.radians(self.heading0)
        return (n*math.cos(psi) + e*math.sin(psi), -n*math.sin(psi) + e*math.cos(psi), wrap180(heading_deg - self.heading0))

@dataclass
class Waypoint: x: float; y: float; heading_deg: float

def lawnmower_path(length_m, left_m, right_m, strip_m, margin_m, start_y=0.0) -> List[Waypoint]:
    span = left_m + right_m - 2*margin_m
    n = max(1, math.ceil(span/strip_m)); step = span/n
    ys = [-left_m + margin_m + (i+0.5)*step for i in range(n)]
    if abs(ys[-1]-start_y) < abs(ys[0]-start_y): ys.reverse()
    x0, x1 = margin_m, length_m - margin_m
    path, out = [], True
    for y in ys:
        a, b, hd = (x0, x1, 0.0) if out else (x1, x0, 180.0)
        path += [Waypoint(a, y, hd), Waypoint(b, y, hd)]; out = not out
    return path

def default_path():
    return lawnmower_path(SEARCH_ZONE_H, SEARCH_ENTRY_FROM_LEFT_M, SEARCH_ZONE_W - SEARCH_ENTRY_FROM_LEFT_M,
                          SEARCH_STRIP_W, SEARCH_MARGIN_M)

def path_length_m(path, start=(0.0, 0.0)):
    pts = [start] + [(w.x, w.y) for w in path]
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:]))

@dataclass
class SearchCommand: vx: float; vy: float; yaw_rate_dps: float; status: str   # status: ok | red_slide | red_slowing

def _clamp(x, lo, hi): return max(lo, min(hi, x))

class SearchPlanner:
    def __init__(self, path=None, left_m=SEARCH_ENTRY_FROM_LEFT_M, right_m=SEARCH_ZONE_W - SEARCH_ENTRY_FROM_LEFT_M):
        self.path = path if path is not None else default_path()
        self.i, self.left_m, self.right_m, self._slide = 0, left_m, right_m, None
    @property
    def done(self): return self.i >= len(self.path)

    def _avoid(self, vf, vr, red: Optional[NearestRed], y, hd, now):
        if red is None or red.distance_m >= RED_INFLUENCE_M:
            self._slide = None; return vf, vr, "ok"
        nvf, nvr = adjust_velocity(vf, vr, red)
        d = max(red.distance_m, 1e-3); ux, uy = red.forward_m/d, red.right_m/d
        toward = vf*ux + vr*uy; status = "red_slowing"
        th = math.radians(hd)
        if self._slide is not None and now > self._slide[2] and toward <= 0.1:
            self._slide = None                      # held long enough and no longer pushing into the zone
        if d < SEARCH_SLIDE_TRIGGER_M and (toward > 0.1 or self._slide is not None):
            if self._slide is None:
                t1 = (-uy, ux); dot = vf*t1[0] + vr*t1[1]
                if abs(dot) > 0.25*math.hypot(vf, vr): t = t1 if dot > 0 else (-t1[0], -t1[1])
                else:
                    prefer_right = (self.right_m - y) >= (y + self.left_m)
                    t = t1 if (t1[1] > 0) == prefer_right else (-t1[0], -t1[1])
                # remember the direction in the ENTRY frame so it does not flip when the drone yaws or moves
                ex, ey = t[0]*math.cos(th) - t[1]*math.sin(th), t[0]*math.sin(th) + t[1]*math.cos(th)
                self._slide = (ex, ey, now + SEARCH_SLIDE_HOLD_S)
            sf = self._slide[0]*math.cos(th) + self._slide[1]*math.sin(th)
            sr = -self._slide[0]*math.sin(th) + self._slide[1]*math.cos(th)
            nvf += SEARCH_SLIDE_SPEED*sf; nvr += SEARCH_SLIDE_SPEED*sr; status = "red_slide"
        s = math.hypot(nvf, nvr)
        if s > SEARCH_SPEED: nvf, nvr = nvf*SEARCH_SPEED/s, nvr*SEARCH_SPEED/s
        return nvf, nvr, status

    def step(self, pose, red, now) -> Optional[SearchCommand]:
        x, y, hd = pose
        while not self.done:
            wp = self.path[self.i]
            if math.hypot(wp.x-x, wp.y-y) < SEARCH_WP_TOL_M and abs(wrap180(wp.heading_deg-hd)) < SEARCH_HEADING_TOL_DEG:
                self.i += 1
            else: break
        if self.done: return None
        wp = self.path[self.i]; dx, dy = wp.x-x, wp.y-y; dist = math.hypot(dx, dy)
        speed = 0.0 if dist < 0.3 else _clamp(SEARCH_SPEED_KP*dist, SEARCH_MIN_SPEED, SEARCH_SPEED)
        vx_e, vy_e = (speed*dx/dist, speed*dy/dist) if dist > 1e-6 else (0.0, 0.0)
        yaw_err = wrap180(wp.heading_deg - hd)
        yaw = _clamp(SEARCH_YAW_KP*yaw_err, -SEARCH_YAW_MAX_DPS, SEARCH_YAW_MAX_DPS)
        if abs(yaw_err) > 45.0: vx_e, vy_e = 0.3*vx_e, 0.3*vy_e
        th = math.radians(hd)
        vf, vr = vx_e*math.cos(th) + vy_e*math.sin(th), -vx_e*math.sin(th) + vy_e*math.cos(th)
        vf, vr, status = self._avoid(vf, vr, red, y, hd, now)
        return SearchCommand(vf, vr, yaw, status)