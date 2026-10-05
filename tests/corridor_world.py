# tests/corridor_world.py — a tiny 3D "corridor" for testing without a drone.
# Draws what a forward camera would see from the drone's position/heading/height, using the
# SAME camera model as the flight code. World: north = +n, east = +e; the corridor runs north
# along e = 0 from n = 0 to n = length. Walls: orange at e = -W/2, green at e = +W/2.
import math
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from config.params import CORRIDOR_WIDTH, CORRIDOR_LENGTH_M
from vision.pixel_to_meters import PixelToMeters

NEAR = 0.1                         # anything closer than this to the lens is clipped


class CorridorWorldCamera:
    def __init__(self, veh, pitch_deg: float, width: int = 640, height: int = 360,
                 corridor_width: float = CORRIDOR_WIDTH, length: float = CORRIDOR_LENGTH_M,
                 wall_height: float = 2.0, obstacles: Sequence[Tuple[float, float, float, float]] = (),
                 hide: Sequence[str] = (), speckle: int = 15, seed: int = 0):
        self.veh, self.pitch = veh, pitch_deg
        self.w, self.h = width, height
        self.cw, self.length, self.wall_h = corridor_width, length, wall_height
        self.obstacles = list(obstacles)         # (n, e, size_n, size_e) boxes standing on the ground
        self.hide = set(hide)                    # {"left"} / {"right"}
        self.fx, self.fy = PixelToMeters.focal_lengths(width, height)
        rng = np.random.default_rng(seed)
        self._noise = rng.integers(-speckle, speckle + 1, (height, width, 1)).astype(np.int16) if speckle else None

    def _to_camera(self, n, e, z_up):
        v = self.veh
        psi = math.radians(v.heading)
        dn, de = n - v.n, e - v.e
        fwd = dn * math.cos(psi) + de * math.sin(psi)
        right = -dn * math.sin(psi) + de * math.cos(psi)
        down = v.altitude - z_up
        th = math.radians(self.pitch)
        return (right, -fwd * math.sin(th) + down * math.cos(th), fwd * math.cos(th) + down * math.sin(th))

    def _draw(self, img, world_pts: List[Tuple[float, float, float]], colour):
        cam = [self._to_camera(*p) for p in world_pts]
        clipped = []                                  # clip against the near plane
        for i, a in enumerate(cam):
            b = cam[(i + 1) % len(cam)]
            a_in, b_in = a[2] > NEAR, b[2] > NEAR
            if a_in:
                clipped.append(a)
            if a_in != b_in:
                t = (NEAR - a[2]) / (b[2] - a[2])
                clipped.append(tuple(a[k] + t * (b[k] - a[k]) for k in range(3)))
        if len(clipped) < 3:
            return
        px = np.array([[self.w / 2 + self.fx * x / z, self.h / 2 + self.fy * y / z] for x, y, z in clipped])
        px = np.clip(px, -20000, 20000).astype(np.int32)
        cv2.fillPoly(img, [px], colour)

    def capture_array(self):
        img = np.full((self.h, self.w, 3), 110, np.int16)
        if self._noise is not None:
            img += self._noise
        img = np.clip(img, 0, 255).astype(np.uint8)
        half = self.cw / 2.0
        n0, n1 = 0.0, self.length
        if "left" not in self.hide:
            self._draw(img, [(n0, -half, 0), (n1, -half, 0), (n1, -half, self.wall_h), (n0, -half, self.wall_h)],
                       (220, 120, 30))
        if "right" not in self.hide:
            self._draw(img, [(n0, half, 0), (n1, half, 0), (n1, half, self.wall_h), (n0, half, self.wall_h)],
                       (40, 170, 70))
        for (n, e, sn, se) in self.obstacles:         # dark box, 1 m tall
            a, b, c, d = (n, e), (n + sn, e), (n + sn, e + se), (n, e + se)
            self._draw(img, [(*p, 0) for p in (a, b, c, d)], (10, 10, 10))
            for p, q in ((a, b), (b, c), (c, d), (d, a)):
                self._draw(img, [(*p, 0), (*q, 0), (*q, 1.0), (*p, 1.0)], (10, 10, 10))
        return img