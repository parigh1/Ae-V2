# =============================================================================
# navigation/red_zone_avoidance.py — Team Vajra AeroTHON 2026
# Takes the velocity the search WANTS to fly and returns one that stays out of
# red zones:
#   * inside RED_STOP_M:  the part of the motion that points toward the zone is
#     removed completely;
#   * between RED_STOP_M and RED_INFLUENCE_M: that part is faded out;
#   * a gentle push away (RED_REPULSION_GAIN, capped at RED_REPULSION_MAX).
# The old plan only pushed (max 0.3 m/s) — that cannot stop a drone flying at
# 1.2 m/s, so the "remove the toward-motion" part is what really protects us.
# =============================================================================
import math
from typing import Optional, Tuple

from config.params import (
    RED_INFLUENCE_M, RED_STOP_M, RED_REPULSION_GAIN, RED_REPULSION_MAX,
)
from vision.red_zone import NearestRed


def adjust_velocity(vx: float, vy: float, nearest: Optional[NearestRed]) -> Tuple[float, float]:
    """vx = forward, vy = right (m/s, body frame)."""
    if nearest is None or nearest.distance_m >= RED_INFLUENCE_M:
        return vx, vy
    d = max(nearest.distance_m, 1e-3)
    ux, uy = nearest.forward_m / d, nearest.right_m / d          # unit vector toward the zone

    toward = vx * ux + vy * uy
    if toward > 0:
        fade = (d - RED_STOP_M) / (RED_INFLUENCE_M - RED_STOP_M)
        allowed = toward * max(0.0, min(1.0, fade))
        vx -= (toward - allowed) * ux
        vy -= (toward - allowed) * uy

    push = min(RED_REPULSION_MAX, RED_REPULSION_GAIN * (1.0 / d - 1.0 / RED_INFLUENCE_M))
    if push > 0:
        vx -= push * ux
        vy -= push * uy
    return vx, vy
