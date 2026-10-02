# =============================================================================
#  config/params.py  —  Team Vajra AeroTHON 2026
#  Single source of truth for ALL constants. No magic numbers anywhere else.
# =============================================================================

# ── SITL / Connection ─────────────────────────────────────────────────────────
SITL_CONNECTION   = "udp:127.0.0.1:14550"   # DroneKit connects here
SITL_BAUD         = 115200
REAL_CONNECTION   = "/dev/ttyAMA0"          # Pixhawk TELEM2 on real Pi
REAL_BAUD         = 921600

# ── Altitude setpoints (metres) ───────────────────────────────────────────────
ALT_START_QR      = 5.0    # hover here to scan the start QR
ALT_CORRIDOR      = 3.0    # corridor flight altitude (~10 feet)
ALT_DELIVERY      = 10.0   # lawnmower search altitude
ALT_PAYLOAD_DROP  = 5.0    # descend to this before winch deploy
ALT_TOLERANCE     = 0.15   # ±0.15m = "altitude reached"
ALT_RTL           = 15.0   # return-to-launch clearance altitude

# ── Altitude PID ──────────────────────────────────────────────────────────────
ALT_KP            = 0.6
ALT_KI            = 0.008
ALT_KD            = 0.12
ALT_ICLAMP        = 2.0    # anti-windup: clamp integral output to ±2 m/s
ALT_VZ_MAX        = 1.5    # max climb/descend rate m/s
ALT_RANGEFINDER_MAX = 8.0  # below this: trust TF-Luna; above: trust baro

# ── Corridor navigation ───────────────────────────────────────────────────────
CORRIDOR_WIDTH        = 3.5    # metres
CORRIDOR_FWD_SPEED    = 0.4    # m/s forward
CORRIDOR_LAT_KP       = 0.25
CORRIDOR_LAT_KI       = 0.003
CORRIDOR_LAT_KD       = 0.04
CORRIDOR_LAT_VMAX     = 0.25   # m/s lateral clamp
CORRIDOR_OBSTACLE_MIN = 0.7    # m — stop fwd motion if lidar < this
CORRIDOR_TIMEOUT      = 60.0   # s — emergency RTL if corridor not cleared

# ── Lawnmower search ─────────────────────────────────────────────────────────
SEARCH_ZONE_W     = 30.0   # metres
SEARCH_ZONE_H     = 40.0   # metres
SEARCH_STRIP_W    = 4.0    # metres between passes
SEARCH_SPEED      = 1.2    # m/s — MUST be ≥1.2 to fit in 15-min window
SEARCH_TIMEOUT    = 90.0   # s — give up and RTL if QR not found

# ── Camera (Pi Camera v3 Wide) ────────────────────────────────────────────────
CAM_FOV_H_DEG     = 84.0
CAM_FOV_V_DEG     = 64.0
CAM_IMG_W         = 1280
CAM_IMG_H         = 720
CAM_FPS           = 10     # scan rate during search

# ── Visual servo (IBVS) ───────────────────────────────────────────────────────
SERVO_KP          = 0.35
SERVO_KD          = 0.08
SERVO_TOLERANCE   = 0.25   # m — centered when offset < this
SERVO_MAX_SPEED   = 0.30   # m/s
SERVO_TIMEOUT     = 20.0   # s
SERVO_CONFIRM_FRAMES = 3   # consecutive frames within tolerance to confirm

# ── QR detection ─────────────────────────────────────────────────────────────
QR_SCAN_TIMEOUT_A = 15.0   # s — start QR (Task A)
QR_CONFIRM_COUNT  = 2      # must decode same value N times before accepting

# ── Green banner detection ────────────────────────────────────────────────────
# HSV ranges — MUST be recalibrated outdoors with real banner
BANNER_H_LOW      = 38
BANNER_H_HIGH     = 85
BANNER_S_LOW      = 60
BANNER_S_HIGH     = 255
BANNER_V_LOW      = 60
BANNER_V_HIGH     = 255
BANNER_MIN_AREA   = 4000   # px² — ignore blobs smaller than this
BANNER_YAW_KP     = 0.03   # deg/s per pixel of horizontal offset
BANNER_ALIGN_TOL  = 40     # px — "aligned" when centroid offset < this
BANNER_ALIGN_FRAMES = 3    # consecutive aligned frames to confirm
BANNER_TIMEOUT    = 30.0   # s

# ── Red zone detection ────────────────────────────────────────────────────────
# Red wraps in HSV so we need two ranges
RED_H_LOW1, RED_H_HIGH1   = 0,   10    # lower red wrap
RED_H_LOW2, RED_H_HIGH2   = 170, 180   # upper red wrap
RED_S_LOW, RED_S_HIGH     = 80,  255
RED_V_LOW, RED_V_HIGH     = 80,  255
RED_MIN_AREA              = 3000  # px²
RED_REPULSION_GAIN        = 0.5   # potential field strength
RED_REPULSION_MAX         = 0.3   # m/s max repulsion velocity

# ── Payload winch ─────────────────────────────────────────────────────────────
WINCH_PWM_CHANNEL   = 9       # AUX channel on Pixhawk
GRIPPER_PWM_CHANNEL = 10
WINCH_LINE_LENGTH   = 5.5     # m — slightly longer than drop altitude
WINCH_LOWER_SPEED   = 0.4     # m/s
WINCH_LOWER_TIME    = WINCH_LINE_LENGTH / WINCH_LOWER_SPEED   # ~13.75s
WINCH_RETRACT_TIME  = WINCH_LOWER_TIME + 2.0   # extra 2s to ensure fully up
WINCH_DEPLOY_WINDOW = 30.0    # s — total allowed for full deploy sequence
PWM_GRIPPER_OPEN    = 1900    # µs
PWM_GRIPPER_CLOSED  = 1100    # µs
PWM_WINCH_LOWER     = 1700    # µs
PWM_WINCH_RAISE     = 1300    # µs
PWM_WINCH_STOP      = 1500    # µs (neutral)

# ── Safety / failsafe ─────────────────────────────────────────────────────────
BATTERY_RTL_PCT     = 25      # % — RTL below this (was 20, increased)
VOLTAGE_RTL_V       = 14.4    # V — RTL below this (4S: 3.6V/cell)
BATTERY_WARN_PCT    = 35      # % — print warning but continue
WATCHDOG_HZ         = 1       # Hz — battery watchdog thread rate

# ArduPilot parameters verified at startup (must match for mission to proceed)
REQUIRED_PARAMS = {
    "FS_THR_ENABLE":  1,   # throttle failsafe → RTL
    "FS_GCS_ENABLE":  1,   # datalink loss → RTL
    "FS_BATT_ENABLE": 2,   # battery failsafe → RTL
    "FENCE_ENABLE":   1,   # geofence active
    "FENCE_ACTION":   1,   # breach → RTL
}

# ── State machine timeouts (per state, seconds) ───────────────────────────────
STATE_TIMEOUTS = {
    "TAKEOFF":           30,
    "CLIMB_TO_START_QR": 20,
    "SCAN_START_QR":     20,   # includes QR_SCAN_TIMEOUT_A + margin
    "DETECT_BANNER_FWD": 35,
    "DESCEND_TO_CORRIDOR":15,
    "CORRIDOR_FORWARD":  65,
    "CLIMB_TO_DELIVERY": 25,
    "SEARCH_DELIVERY":   95,   # SEARCH_TIMEOUT + margin
    "CENTER_OVER_QR":    25,
    "DESCEND_TO_DROP":   20,
    "DEPLOY_PAYLOAD":    35,
    "CLIMB_AFTER_DROP":  20,
    "DETECT_BANNER_RTN": 35,
    "CORRIDOR_RETURN":   65,
    "DESCEND_TO_LAND":   30,
    "LAND":              30,
}
