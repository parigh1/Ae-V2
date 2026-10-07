# =============================================================================
# config/params.py — Team Vajra AeroTHON 2026
# Single source of truth for ALL constants. No magic numbers anywhere else.
#
# CHANGES vs previous version are tagged  # [NEW]  or  # [CHANGED].
# =============================================================================

# ── SITL / Connection ─────────────────────────────────────────────────────────
SITL_CONNECTION = "udp:127.0.0.1:14550"   # DroneKit connects here
SITL_BAUD = 115200
REAL_CONNECTION = "/dev/ttyAMA0"          # Pixhawk TELEM2 on real Pi
REAL_BAUD = 921600

# ── Control loop ──────────────────────────────────────────────────────────────
CONTROL_HZ = 10                           # [NEW] default loop rate for handlers

# ── Altitude setpoints (metres) ───────────────────────────────────────────────
ALT_START_QR = 5.0         # hover here to scan the start QR
ALT_CORRIDOR = 3.0         # corridor flight altitude (~10 feet)
ALT_DELIVERY = 10.0        # lawnmower search altitude
ALT_PAYLOAD_DROP = 5.0     # descend to this before winch deploy
ALT_TOLERANCE = 0.15       # ±0.15m = "altitude reached"
ALT_RTL = 15.0             # return-to-launch clearance altitude

# ── Altitude PID ──────────────────────────────────────────────────────────────
ALT_KP = 0.6
ALT_KI = 0.008
ALT_KD = 0.12
ALT_ICLAMP = 2.0           # anti-windup: clamp integral output to ±2 m/s
ALT_VZ_MAX = 1.5           # max climb/descend rate m/s
ALT_RANGEFINDER_MAX = 8.0  # below this: trust TF-Luna; above: trust baro
ALT_BLEND_HALF_WIDTH = 1.0 # [NEW] blend rangefinder->baro over MAX±this (7–9 m)

# ── Corridor navigation ───────────────────────────────────────────────────────
CORRIDOR_WIDTH = 3.5       # metres
CORRIDOR_FWD_SPEED = 0.4   # m/s forward
CORRIDOR_LAT_KP = 0.25
CORRIDOR_LAT_KI = 0.003
CORRIDOR_LAT_KD = 0.04
CORRIDOR_LAT_VMAX = 0.25   # m/s lateral clamp
CORRIDOR_OBSTACLE_MIN = 0.7  # m — stop fwd motion if lidar < this
CORRIDOR_TIMEOUT = 60.0    # s — emergency RTL if corridor not cleared

# ── Lawnmower search ─────────────────────────────────────────────────────────
# [CHANGED] Old values (4 m strips, 90 s) could not cover the zone:
#   30x40 m @ 4 m strips ≈ 340 m of flight ≈ 280 s at 1.2 m/s.
# At 10 m altitude the camera sees ~18 m x 12.5 m, so 8 m strips still overlap.
# 8 m strips -> 4 passes ≈ 184 m ≈ 153 s. REVISIT once QR size is confirmed.
SEARCH_ZONE_W = 30.0       # metres
SEARCH_ZONE_H = 40.0       # metres
SEARCH_STRIP_W = 8.0       # [CHANGED] metres between passes (was 4.0)
SEARCH_SPEED = 1.2         # m/s — MUST be ≥1.2 to fit in 15-min window
SEARCH_TIMEOUT = 220.0     # [CHANGED] s — give up and RTL (was 90.0)

# ── Camera (Pi Camera v3 Wide) ────────────────────────────────────────────────
CAM_FOV_H_DEG = 84.0
CAM_FOV_V_DEG = 64.0
CAM_IMG_W = 1280
CAM_IMG_H = 720
CAM_FPS = 10               # scan rate during search

# ── Camera SETUP (change ONE line to switch hardware layout) ─────────────────
# "single_fixed" : ONE camera, bolted at a fixed tilt, used for everything
# "dual"         : TWO cameras, one pointing down, one pointing forward
# "servo"        : ONE camera on a servo that tilts between down / forward
CAMERA_MODE = "single_fixed"

# Pitch = angle of the camera's centre line BELOW the horizon.
#   90 = pointing straight down      0 = pointing straight forward
# Straight-down is only visible in a tilted camera if
#   pitch + (CAM_FOV_V_DEG / 2) >= 90   ->  pitch >= 58 for a 64 deg lens.
# 65 is a PLACEHOLDER for single_fixed: test it on the real airframe (see guide).
CAM_MOUNTS = {
    "single_fixed": {"down": 65.0, "forward": 65.0},
    "dual":         {"down": 90.0, "forward": 10.0},
    "servo":        {"down": 90.0, "forward": 0.0},
}
CAM_MOUNT_PITCH = CAM_MOUNTS[CAMERA_MODE]

# Which driver reads each physical camera: "webcam" (laptop, USB) or "picamera2" (Pi).
# NOTE: a Raspberry Pi 4 has ONE camera (CSI) port. A second camera must be a USB one.
CAMERA_BACKEND = {"main": "webcam", "down": "webcam", "forward": "webcam"}
CAM_DEVICE_INDEX = {"main": 0, "down": 0, "forward": 1}   # webcam numbers

# If the camera is bolted on rotated (ribbon cable pointing sideways, etc.):
# degrees the image is rotated CLOCKWISE relative to "top of image = drone nose".
# Allowed: 0, 90, 180, 270.
CAM_IMAGE_ROTATION_DEG = 0

# Servo-tilt mode only (ignored otherwise)
CAM_SERVO_CHANNEL = 11          # AUX channel on the Pixhawk
CAM_SERVO_PWM_DOWN = 1100       # µs when pointing straight down  (calibrate!)
CAM_SERVO_PWM_FORWARD = 1900    # µs when pointing straight forward (calibrate!)
CAM_SERVO_SETTLE_S = 0.4        # wait after moving before taking a picture

# ── Visual servo (IBVS) ───────────────────────────────────────────────────────
SERVO_KP = 0.35
SERVO_KD = 0.08
SERVO_TOLERANCE = 0.25     # m — centered when offset < this
SERVO_MAX_SPEED = 0.30     # m/s
SERVO_TIMEOUT = 20.0       # s
SERVO_CONFIRM_FRAMES = 3   # consecutive frames within tolerance to confirm
SERVO_LOST_FRAMES_MAX = 10 # [NEW] consecutive frames without target -> give up

# ── QR detection ─────────────────────────────────────────────────────────────
QR_SCAN_TIMEOUT_A = 15.0   # s — start QR (Task A)
QR_CONFIRM_COUNT = 2       # must decode same value N times before accepting
START_QR_FORWARD_M = 1.0   # [NEW] rulebook 4.2.4: climb to 5 m, move ~1 m forward, then scan

# ── Green banner detection ────────────────────────────────────────────────────
# HSV ranges — MUST be recalibrated outdoors with real banner
BANNER_H_LOW = 38
BANNER_H_HIGH = 85
BANNER_S_LOW = 60
BANNER_S_HIGH = 255
BANNER_V_LOW = 60
BANNER_V_HIGH = 255
BANNER_MIN_AREA = 4000     # px² — ignore blobs smaller than this
BANNER_YAW_KP = 0.03       # deg/s per pixel of horizontal offset
BANNER_ALIGN_TOL = 40      # px — "aligned" when centroid offset < this
BANNER_ALIGN_FRAMES = 3    # consecutive aligned frames to confirm
BANNER_TIMEOUT = 30.0      # s

# ── Red zone detection ────────────────────────────────────────────────────────
# Red wraps in HSV so we need two ranges
RED_H_LOW1, RED_H_HIGH1 = 0, 10      # lower red wrap
RED_H_LOW2, RED_H_HIGH2 = 170, 180   # upper red wrap
RED_S_LOW, RED_S_HIGH = 80, 255
RED_V_LOW, RED_V_HIGH = 80, 255
RED_MIN_AREA = 3000        # px²
RED_REPULSION_GAIN = 0.5   # potential field strength
RED_REPULSION_MAX = 0.3    # m/s max repulsion velocity

# ── Payload winch ─────────────────────────────────────────────────────────────
WINCH_PWM_CHANNEL = 9      # AUX channel on Pixhawk
GRIPPER_PWM_CHANNEL = 10
WINCH_LINE_LENGTH = 5.5    # m — slightly longer than drop altitude
WINCH_LOWER_SPEED = 0.4    # m/s
WINCH_LOWER_TIME = WINCH_LINE_LENGTH / WINCH_LOWER_SPEED   # ~13.75s
WINCH_RETRACT_TIME = WINCH_LOWER_TIME + 2.0  # extra 2s to ensure fully up
WINCH_DEPLOY_WINDOW = 30.0 # s — total allowed for full deploy sequence
PWM_GRIPPER_OPEN = 1900    # µs
PWM_GRIPPER_CLOSED = 1100  # µs
PWM_WINCH_LOWER = 1700     # µs
PWM_WINCH_RAISE = 1300     # µs
PWM_WINCH_STOP = 1500      # µs (neutral)

# ── Safety / failsafe ─────────────────────────────────────────────────────────
BATTERY_RTL_PCT = 25       # % — RTL below this (was 20, increased)
VOLTAGE_RTL_V = 14.4       # V — RTL below this (4S: 3.6V/cell)
BATTERY_WARN_PCT = 35      # % — print warning but continue
WATCHDOG_HZ = 1            # Hz — battery watchdog thread rate

# ArduPilot parameters verified at startup (must match for mission to proceed)
REQUIRED_PARAMS = {
    "FS_THR_ENABLE": 1,    # throttle failsafe → RTL
    "FS_GCS_ENABLE": 1,    # datalink loss → RTL
    "FS_BATT_ENABLE": 2,   # battery failsafe → RTL
    "FENCE_ENABLE": 1,     # geofence active
    "FENCE_ACTION": 1,     # breach → RTL
}

# ── Mission clock & failure policy ────────────────────────────────────────────
# Rulebook: 15 minutes, counted from the moment takeoff throttle goes up.
MISSION_TIME_LIMIT_S = 900.0
MISSION_EMERGENCY_MARGIN_S = 60.0   # at LIMIT - MARGIN the drone gives up and does RTL
EMERGENCY_TIMEOUT_S = 120.0         # max time spent waiting for RTL to finish

# ── Payload release style ─────────────────────────────────────────────────────
# "gripper_drop": open the gripper at ALT_PAYLOAD_DROP (what the Phase 1 report says)
# "winch"       : lower on a line to the ground, then release (what rulebook
#                 Figure 3 describes). Flip this one line if inspectors ask for it.
PAYLOAD_MODE = "gripper_drop"

# ── Per-state time budget (seconds) ───────────────────────────────────────────
# What happens when a state runs out of time is in mission/states.py
# (TIMEOUT_FALLBACK). Worst case, all of these add up to ~720 s < 900 s.
STATE_TIMEOUTS = {
    "INIT": 10,
    "PREFLIGHT": 30,
    "ARM": 20,
    "TAKEOFF": 30,
    "MOVE_TO_QR_POINT": 10,
    "SCAN_START_QR": 20,
    "FIND_BANNER_FWD": 35,
    "DESCEND_TO_CORRIDOR": 15,
    "CORRIDOR_FORWARD": 65,
    "CLIMB_TO_DELIVERY": 25,
    "SEARCH_DELIVERY": 225,
    "CENTER_OVER_QR": 25,
    "DESCEND_TO_DROP": 20,
    "DEPLOY_PAYLOAD": 35,
    "CLIMB_AFTER_DROP": 20,
    "FIND_BANNER_RTN": 35,
    "DESCEND_TO_CORRIDOR_RTN": 15,
    "CORRIDOR_RETURN": 65,
    "RETURN_TO_HOME": 40,
    "LAND": 30,
}

# ── Phase 3 additions (state machine) ─────────────────────────────────────────
PREFLIGHT_MIN_BATTERY_PCT = 80      # do not take off below this
START_QR_MOVE_SPEED = 0.5           # m/s for the ~1 m hop before scanning the start QR
CENTER_RETRY_MAX = 2                # times we go back to searching after losing the target
HOME_RADIUS_M = 2.0                 # "back over the start point" when closer than this
PAYLOAD_RELEASE_WAIT_S = 1.5        # time allowed for the gripper to open fully

# ── Phase 4 additions (banner + red-zone vision) ──────────────────────────────
# Banner: reject blobs that are not a solid upright rectangle (grass is ragged).
BANNER_PROC_WIDTH = 640         # analyse a 640-px-wide copy of the frame (speed on the Pi)
BANNER_MORPH_CLOSE = 25         # px @1280 wide: fills the gaps left by the banner's text
BANNER_MORPH_OPEN = 5           # px @1280 wide: removes speckles
BANNER_MIN_SOLIDITY = 0.70      # blob area / convex-hull area
BANNER_MIN_FILL = 0.45          # blob area / bounding-box area
BANNER_YAW_MAX_DPS = 30.0       # never turn faster than this
BANNER_SEARCH_YAW_DPS = 15.0    # slow spin (clockwise) while the banner is not visible

# Red zone: a solid red rectangle on the ground; red QR codes / speckle must not count.
RED_MORPH_OPEN = 5              # px @1280 wide
RED_MIN_SOLIDITY = 0.80
RED_MIN_FILL = 0.80             # red pixels / area of the tightest (rotated) rectangle around the blob
RED_INFLUENCE_M = 6.0           # start steering away from a red zone closer than this
RED_STOP_M = 2.5                # never move toward a red zone closer than this

# ── Phase 5 additions (corridor navigation, camera only) ──────────────────────
CORRIDOR_LENGTH_M = 10.0            # rulebook Figure 3
CORRIDOR_EXIT_MARGIN_M = 1.5        # fly this much past the nominal end before leaving
CORRIDOR_END_WINDOW_M = 2.0         # near the end, "walls vanished" also means "we are out"
CORRIDOR_END_LOST_S = 1.0           # ...if they stay gone this long

# Wall finding: edges -> Hough lines -> project onto the ground -> keep lines along the corridor
CORRIDOR_PROC_WIDTH = 640
CORRIDOR_CANNY_LOW = 50
CORRIDOR_CANNY_HIGH = 150
CORRIDOR_HOUGH_THRESH = 40
CORRIDOR_MIN_LINE_PX = 40           # px at 640 wide
CORRIDOR_MAX_GAP_PX = 15            # px at 640 wide
CORRIDOR_MIN_GROUND_LEN_M = 0.5
CORRIDOR_MAX_LINE_ANGLE_DEG = 35    # a wall line may deviate this far from "straight ahead"
CORRIDOR_MIN_WALL_M = 0.4
CORRIDOR_MAX_WALL_M = 3.1
CORRIDOR_CLUSTER_M = 0.3            # lines this close together count as the same wall
CORRIDOR_NEAR_PREFERENCE = 0.4      # prefer the NEAREST well-supported line group (wall base)
CORRIDOR_MIN_SIDE_LENGTH_M = 1.0    # total line length needed to trust one wall

# Steering
CORRIDOR_YAW_KP = 0.6               # deg/s of turn per degree of heading error
CORRIDOR_YAW_MAX_DPS = 15.0
CORRIDOR_I_CLAMP = 0.1              # m/s cap on the sideways integral term
CORRIDOR_SLOW_LATERAL_M = 0.8       # slow down when this far off the centre line
CORRIDOR_WALL_MARGIN_M = 0.6        # never steer closer than this to a wall

# Obstacles (camera sees dark objects on the ground; a LiDAR can be added later)
OBSTACLE_V_MAX = 60                 # HSV "value" below this counts as dark
OBSTACLE_MIN_AREA = 800             # px² at 1280x720
OBSTACLE_MAX_RANGE_M = 8.0
CORRIDOR_OBSTACLE_SLOW_M = 3.5      # start slowing / side-stepping at this distance
CORRIDOR_OBSTACLE_STOP_M = 1.5      # no forward motion inside this distance
CORRIDOR_OBSTACLE_CLEAR_M = 0.8     # an obstacle this far off the centre line can be ignored
CORRIDOR_SIDESTEP_M = 0.8           # how far to shift away from an obstacle
CORRIDOR_OBSTACLE_PASS_M = 1.5      # keep the shifted line this much longer after first seeing it

# ── Phase 6 additions (lawnmower search) ──────────────────────────────────────
SEARCH_ENTRY_FROM_LEFT_M = 15.0   # distance from the zone's LEFT edge when entering (15 = middle of 30 m). SET from the Phase 2 layout/geofence.
SEARCH_MARGIN_M = 2.0
SEARCH_WP_TOL_M = 1.5
SEARCH_HEADING_TOL_DEG = 10.0
SEARCH_SPEED_KP = 0.8
SEARCH_MIN_SPEED = 0.3
SEARCH_YAW_KP = 0.8
SEARCH_YAW_MAX_DPS = 30.0
SEARCH_SLIDE_SPEED = 0.5
SEARCH_SLIDE_TRIGGER_M = 4.0
SEARCH_SLIDE_HOLD_S = 2.0