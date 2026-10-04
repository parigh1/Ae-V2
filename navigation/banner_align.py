# =============================================================================
# navigation/banner_align.py — Team Vajra AeroTHON 2026
# Turning law for lining the nose up with the banner. Pure maths, easy to test.
# offset_px > 0 means the banner is to the RIGHT -> turn clockwise (+ yaw rate).
# =============================================================================
from config.params import BANNER_YAW_KP, BANNER_YAW_MAX_DPS


def yaw_rate_for_offset(offset_px: float) -> float:
    """Degrees/second to turn (+ = clockwise), proportional to how far off we are."""
    rate = BANNER_YAW_KP * offset_px
    return max(-BANNER_YAW_MAX_DPS, min(BANNER_YAW_MAX_DPS, rate))
