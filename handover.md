# Project Handover Document: AeroTHON 2026 Autonomous Drone Software (Team Vajra)

**Repo:** https://github.com/parigh1/Ae-V2 (public, branch `main`). **Local project:** `C:\Users\parig\AeV2Prog` (Windows, PyCharm, venv at `.\venv`, Python 3.11).
**Status at handover:** Phases 1–5 are applied, tested (130 tests passing) and pushed. Phase 6 (lawnmower search) is written in the previous assistant's sandbox but not yet delivered to the user. Nothing has flown yet: it is all mock/simulator tested.

---

## 0. How the next assistant should work with this user

- **User:** goes by "Captain", a B.Tech CS (AI & ML) student and the software developer on the team. They are a **complete beginner at Git, PyCharm and drone software**. Explain the smallest steps (for example, right-click folder → New → Python File).
- **Delivery format (explicit preference):** put code **inline in the reply**, with clear instructions on which file to edit and where. **Do not create zip files or many separate files**, because that burns the user's credits. For edits to existing files, say "find this line, change it to this".
- **Always keep doing:** run the full test suite on the code before handing it over, and report the result honestly. Mutation-check critical logic by breaking it on purpose and confirming the tests fail. After each phase give the git commands: `git add .`, `git commit -m "..."`, `git tag phase-N-name`, `git push`, `git push origin phase-N-name`.
- **Test command (user's machine):** `python -m pytest -v --ignore=tests/test_sitl_connect.py --ignore=tests/test_altitude_sitl.py`. The two ignored files are the user's old tests and need an ArduPilot SITL running. The last known result was **130 passed** in about 30 s.
- **Reading the repo:** the GitHub repo root and tree pages returned a **stale cached snapshot**. Individual file pages like `https://github.com/parigh1/Ae-V2/blob/main/<path>` worked. If in doubt, ask the user to paste a file.
- **Honesty rules:**
  - Say what is unverified on real hardware (camera colour order, corridor perception, HSV thresholds, decode range).
  - Never claim the drone "works". Say "passes mock/simulator tests".
  - Files appeared once in the working folder that the assistant did not author (see §5.6). If you ever see unexplained files or params, verify them against this document and don't trust them blindly.

---

## 1. Project overview and core objectives

**Competition:** SAEINDIA **AeroTHON 2026, Track 1 "Rotorcraft Systems Challenge (UAS)"**, a design-build-fly contest.

| Item | Value |
|---|---|
| Team | Team Vajra, VIT Bhopal University, Team ID AT2026RC49 |
| Phase 1 | Design report and presentation (done; the report PDF is submitted) |
| Phase 2 | Technical inspection (50 marks), Mission 1 manual (100), Mission 2 autonomous (100), 250 total |
| Our task | The **software** for **Mission 2 "SkyScan: Autonomous Rapid Delivery"**, flown fully autonomously on a Raspberry Pi 4 companion computer commanding a Pixhawk running ArduPilot. |

**Mission 2 in one paragraph.** The drone takes off autonomously and climbs to 5 m. It moves about 1 m forward and scans a **start QR code**, which gives the delivery location ID. It finds the **green AeroTHON banner** at the corridor entrance and aligns with it. It descends to about 3 m (10 ft) and flies a **3.5 m wide, ~10 m long corridor**, avoiding static obstacles. It climbs to about 10 m over the **40 m × 30 m delivery zone**, which holds several QR codes. It finds the QR matching the ID while avoiding **red no-fly zones** (−5 marks per violation). It descends to 5 m and drops the payload (100 g, 10×5×5 cm). It climbs to 10 m, finds the banner again, flies back through the corridor, returns to the takeoff point and **lands autonomously**. The whole mission has **15 minutes**, and the clock starts when takeoff throttle rises.

**Ultimate goal:** maximise Mission 2 marks (see §4.3) and pass technical inspection. Speed counts, because 15 marks are proportional to completion time.

**Audience and constraints:** the judges and inspectors, and Captain as the hands-on developer. The software must be **safe by default**: any failure should end in return-to-launch (RTL), never a crash.

---

## 2. Tech stack and architecture

### 2.1 Hardware (from the design report; the code must match it)
- **Airframe:** H-shaped quad, carbon-rod arms, wheelbase 438.88 mm. Take-off weight 1864 g without payload and 1964 g with it; limit 2000 g. Thrust-to-weight 3.65:1.
- **Propulsion:** 4× XING 2814 880KV motors, 9×4.5 props, 4× 40 A ESCs, **4S 4000 mAh 70C LiPo**, about 12 min hover endurance.
- **Brain:** **Pixhawk 2.4.8** (ArduPilot), **Raspberry Pi 4** (the BoM says 1 GB). They are linked over the MAVLink serial port TELEM2 (`/dev/ttyAMA0`, 921600 baud on the real drone).
- **Sensors:**
  - Raspberry Pi Camera Module v3 (Wide).
  - One TF-Luna LiDAR (downward rangefinder via the Pixhawk).
  - Pixhawk GPS.
  - A **forward LiDAR is NOT owned or used**; it is an optional backup only (see §5).
- **Payload:** SG90 gripper servo (the report calls it a "variable gripper"). The BoM also lists an MG995 camera servo. There is **no winch** in the BoM.
- **Camera setup is undecided.** Options are (a) one fixed tilted camera, (b) two cameras (forward + down), (c) one camera on a tilt servo. A **Pi 4 has only one CSI camera port**, so a second camera must be USB. **Current choice: single fixed camera, with the code switchable by one line** (`CAMERA_MODE`).
- The user has no LiDAR and no real corridor, so corridor perception has only been tested on a synthetic renderer.

### 2.2 Software stack
- **Python 3.11** (the Pi runs 3.11; the user's venv is 3.11). Python 3.14 breaks the pinned numpy, so don't use it.
- **Libraries** (`requirements.txt`): `dronekit==2.9.2`, `pymavlink==2.4.41`, `future` (needed by dronekit), `opencv-python==4.9.0.80`, `pyzbar==0.1.9`, `numpy==1.26.4`, `pyserial==3.5`, `pytest`, `qrcode[pil]`. Pi only: `picamera2`, `RPi.GPIO`.
- **DroneKit compatibility shim** (top of `hardware/vehicle.py`): re-adds `collections.MutableMapping` and similar, which were removed in Python 3.10+.
- **pyzbar on Windows** needs the Microsoft VC++ 2013 x64 redistributable if the zbar DLL is missing.
- **Tooling:** PyCharm, git/GitHub (tags per phase), ArduPilot SITL later.

### 2.3 Repo layout (as applied by the user, Phases 1–5)

```
Ae-V2/
├─ config/params.py            # ALL constants; no magic numbers elsewhere
├─ hardware/
│  ├─ vehicle.py               # Vehicle (DroneKit wrapper) + MockVehicle (same interface)
│  └─ camera.py                # CameraRig: single_fixed | dual | servo; WebcamCamera, PiCamera, FakeCamera
├─ vision/
│  ├─ preprocessing.py         # USER'S OWN file: 6 QR preprocessing variants (index 2 = CLAHE; expects RGB)
│  ├─ qr_scanner.py            # RobustQRScanner.decode_all(frame, want), QRDetection, QRConfirmer
│  ├─ qr_system.py             # QRSystem: scan_start_qr, scan_step, find_target -> TargetFix
│  ├─ pixel_to_meters.py       # pixel -> ground metres for a TILTED camera
│  ├─ color_masks.py           # green/red HSV masks, odd_kernel
│  ├─ banner_detector.py       # BannerDetector.detect -> BannerFix(offset_px, ...)
│  ├─ red_zone.py              # RedZoneDetector.detect / .nearest -> NearestRed
│  └─ corridor_detector.py     # CorridorDetector.detect -> CorridorFix (metres)
├─ navigation/
│  ├─ altitude.py              # AltitudeController (PID, rangefinder/baro blend)
│  ├─ visual_servo.py          # VisualServo (PD centring over a target)
│  ├─ banner_align.py          # yaw_rate_for_offset()
│  ├─ red_zone_avoidance.py    # adjust_velocity()
│  └─ corridor.py              # obstacle sensors + CorridorController + DistanceTracker
├─ mission/
│  ├─ states.py                # State enum (22), NEXT, ORDER, TERMINAL, TIMEOUT_FALLBACK, EXTRA_ALLOWED, allowed_next()
│  ├─ context.py               # MissionContext (shared mission data)
│  ├─ state_machine.py         # StateMachine.run() -> MissionResult
│  └─ handlers.py              # one handler per state + default_handlers()
├─ safety/watchdog.py          # Watchdog.check() -> event(action "stop"|"emergency", reason)
├─ payload/release.py          # PayloadReleaser (gripper_drop | winch)
├─ scripts/                    # camera_check.py, tune_hsv.py, detect_check.py, corridor_check.py
├─ tests/                      # pytest suite + sim helpers (below)
├─ opcv.py, test.py            # legacy root files from the first commit (planned to move to scripts/)
├─ requirements.txt, .gitignore (.idea/, __pycache__/, *.pyc, .venv/, venv/), README.md (STALE, rewrite in Phase 8)
```

**tests/:**
- `conftest.py` adds the project root to the path.
- `sim_helpers.py` holds `SimClock`, `SimVehicle` (kinematics in a north/east world), `FlightSim` (adds altitude, yaw and fake GPS) and `BannerWorldCamera`.
- `corridor_world.py` is a tiny 3D corridor renderer.
- The `test_*.py` files cover altitude, camera rig, tilt geometry, QR pipeline, visual servo, payload, state machine, banner/red zone, banner states and corridor.
- The two old SITL tests are `test_sitl_connect.py` and `test_altitude_sitl.py`.

---

## 3. Methodology and architectural decisions

### 3.1 Principles
1. **Companion-computer autonomy.** ArduPilot only stabilises. All decisions run in Python on the Pi, which sends velocity setpoints in **GUIDED** mode.
2. **One constants file** (`config/params.py`). Every threshold lives there.
3. **Hardware abstraction.** Nothing imports dronekit except `vehicle.py`. `MockVehicle` has an identical interface, so everything is testable without a drone.
4. **Everything is a simulator-testable pure function or tick handler.** Time (`clock`) and sleeping (`sleep`) are injected, so tests run in simulated time.
5. **Fail safe.**
   - Every state has a timeout and a fallback.
   - Any handler exception, illegal transition, low battery or out-of-time condition goes to EMERGENCY (RTL).
   - A pilot mode change means the Pi stops commanding entirely.
6. **Swappable hardware by config.** Camera layout, payload mode and (optionally) a LiDAR are configuration, not rewrites.
7. **Test discipline.** Unit tests, a kinematic simulator, synthetic renderers that use the same camera model as the flight code, and **mutation checks** (deliberately break signs and logic, and confirm the tests fail).

### 3.2 Conventions that must never be broken
- **Velocity frame:** `MAV_FRAME_BODY_OFFSET_NED` through `Vehicle.send_body_velocity(vx_fwd, vy_right, vz_down)`. **vz is NED: negative means climb.** All vision and corridor control is in the body frame. (`send_ned_velocity` is north/east and must not be used for vision control.)
- **Yaw rate:** positive means clockwise, in deg/s. `send_velocity_yawrate(vx, vy, vz, yaw_rate_dps)` sends velocity and yaw rate in one message.
- **Frames/images:** RGB uint8, `H×W×3`. The Pi camera is configured with format `"BGR888"`, which (per the picamera2 quirk) yields RGB in memory. **Verify on the Pi** with a red object.
- **Bodies/signs:** `forward_m > 0` means ahead, `right_m > 0` means to the right. Image `+x` is right and `+y` is down.
- **Camera pitch:** angle of the optical axis below the horizon (90 = straight down, 0 = straight forward). Image rotation parameter `CAM_IMAGE_ROTATION_DEG` (0/90/180/270, clockwise).
- **Ground projection (core of `PixelToMeters.offset_body`):**
  - `fx = (W/2)/tan(HFOV/2)`, `fy = (H/2)/tan(VFOV/2)`, `u = dx/fx`, `v = dy/fy` (after un-rotating the image).
  - `ray_fwd = cosθ − sinθ·v`, `ray_right = u`, `ray_down = sinθ + cosθ·v`.
  - If `ray_down < 0.05` the result is `None` (looking at or above the horizon). Otherwise `t = altitude/ray_down`, `forward_m = t·ray_fwd`, `right_m = t·ray_right`.
  - At θ = 90° this reduces to the plain nadir formula (100 px at 5 m ≈ 0.70 m).
  - Height is the rangefinder reading, or the barometer above about 8 m.
- **Tick-based state machine.** Handlers are `handler(ctx) -> Optional[State]`. They are called every tick (`CONTROL_HZ` = 10). Returning `None` means stay, and returning a State means go there. `ctx.sd` is per-state scratch, wiped on entry. Handlers are non-blocking.
- **Tilted fixed camera trade-off.**
  - Pitch placeholder is 65° with a 64° vertical FOV, so the ground is visible from 33° to 97° below the horizon.
  - Straight-down is visible only if pitch ≥ ~58°.
  - Ground visible ahead is limited to about 1.54× height (4.6 m at 3 m height, 7.7 m at 5 m, 15 m at 10 m).
  - Tilt must be chosen on the real airframe.

### 3.3 Why certain decisions were made
- **Body-frame control** was needed because LOCAL_NED velocity only works when the drone faces north.
- **Corridor perception projects Hough lines onto the ground** (instead of the report's "pixel midpoint"), because the pixel midpoint of vertical edges depends on unknown depth. The projection gives metric lateral offset and heading error. The base line is the nearest well-supported line group on each side, because wall tops project farther away.
- **Canny runs per colour channel**, because walls with the same brightness as grass were invisible to grey-scale Canny.
- **Red-zone avoidance removes the velocity component toward the zone** (fading between 6 m and 2.5 m), plus a slide along the edge. The report's potential-field push (max 0.3 m/s) cannot stop a 1.2 m/s drone.
- **Red zones are filtered by "fill ratio of the minimum-area rectangle ≥ 0.8"**, not by convex-hull solidity alone. A red/white checkerboard merges at its corners and passed the solidity test.
- **Obstacle side-step is latched** until the drone has passed the obstacle. Without the latch, the drone forgets the obstacle, swings back and hits it.
- **A timeout that falls back to EMERGENCY counts as a failure** (it was a bug that it reported success).
- **Distance tracking:** use GPS distance from the corridor entry when it agrees (within 4 m) with dead reckoning, otherwise the smaller of the two. Leaving late is safer than leaving early.
- **If the start QR can't be read:** continue flying for corridor and return marks, but skip the search and drop (`delivery_id = None`).
- **If the banner is never found (35 s):** the policy is EMERGENCY/RTL. The user has been told and was not asked to change it.

---

## 4. Knowledge distilled from the PDFs

### 4.1 Rulebook: hard constraints
- **Vehicle:** multirotor only, take-off weight under 2 kg, payload capacity 100 g, electric, communications range at least 1 km.
- **Payload:** 10×5×5 cm with a top lifting eye.
- **Mission 2 sequence (rulebook 4.2.4):**
  1. Take off autonomously from the start point.
  2. Ascend to **5 m**, move forward about **1 m**, scan the QR that holds the delivery info.
  3. Detect the **"AeroTHON 2026 Banner with Green Background"** at the corridor entrance and align.
  4. Descend to **10 ft (≈3 m)** and fly the **3.5 m wide** corridor, avoiding obstacles and walls.
  5. In the delivery zone, ascend to about **10 m**. Multiple QR codes are present and the matching one must be identified. Avoid **red zones**.
  6. Descend to **5 m** and deliver the payload.
  7. Ascend to 10 m, detect the banner again (return lap), fly back through the corridor, return to the take-off point and **land safely without manual intervention**.
- **Geofence:** coordinates are provided to teams **in Phase 2**; program them into the ground station. The Phase 1 appendix says only that they will be shared.
- **Flight data:** the UAS must be equipped to **record flight data** and share it with the jury.
- **Time:** 15 minutes per team. The clock starts when take-off throttle increases. The slot is lost if the team isn't ready within 5 minutes of being called.
- **Retry rule:** if the first flight lasts under 2 minutes (failed takeoff, immediate landing or crash), the team may overhaul and retry.
- **Manual override:** Mission 1 must be fully manual. Mission 2 must be fully autonomous after takeoff.

### 4.2 Figure 3 (Mission 2 layout), my reading of the figure
- **Layout, left to right:** blue **Take Off / Landing Zone**, with the start QR ("Target A: each team will get a different location as target location") beside it. Next comes a **corridor 10 m long**, drawn as a **green strip on top and an orange strip below**. Each strip is labelled 3.5 m, "Corridor Height 10 feet", and "Static Obstacles" (black bars, drawn mostly in the orange strip). Then comes the **delivery zone: a green 40 m × 30 m rectangle** with ~5–6 colourful QR codes and a **red zone rectangle** (white border) toward the right.
- **Legend:** green arrow = Forward Lap, orange arrow = Return Lap.
- **Callout:** *"The payload must be lowered using a controlled pulley mechanism from a height of 5 meters and released only after reaching the ground at the correct target location."* "Flight path and layout are subject to change on event day!"
- **⚠ UNRESOLVED:** the figure suggests **two parallel 3.5 m lanes (green for the forward lap, orange for the return lap)**, but the text describes a single 3.5 m corridor flown twice. Wall construction and appearance are unknown. Our corridor code assumes **one 3.5 m channel bounded by two walls whose bases meet the floor with a visible contrast**. Confirm with the organisers or on site.

### 4.3 Mission 2 scoring (preliminary; may be updated after Phase 1)

| # | Item | Marks |
|---|---|---|
| 1 | Autonomous takeoff and start-QR decode | 10 |
| 2 | Corridor navigation (≈3 m, 3.5 m wide) | 15 |
| 3 | Delivery-zone QR identification at about 10 m | 10 |
| 4 | Red zone avoidance (**−5 per violation**) | 10 |
| 5 | Payload delivery accuracy (drop accuracy, altitude compliance, stability) | 15 |
| 6 | Autonomous return through the corridor | 20 |
| 7 | Safe autonomous landing | 5 |
| 8 | Completion within 15 min (**proportional to speed**) | 15 |

Technical inspection (50 marks) checks:
- Dimensions against the 2D drawing.
- Same components as the report (motor KV, ESC amps, prop size, battery V/mAh/C, flight controller, radio).
- **Take-off weight within 50 g of the report for full 10 marks** (reported 1964 g with payload).
- Structural integrity.
- **Failsafes configured:** RTL on low battery, RTL on datalink loss, geofence.
- Any deviation from the report needs a **Change Request** (Appendix B), and the jury assesses penalties.

### 4.4 Project report: software methodology and what the team committed to
- **State machine on the Pi.** ArduPilot stabilises only.
- **QR:** 6 preprocessing variants (raw, grey, CLAHE, adaptive Gaussian threshold 11×11, 3×3 sharpen, morphological close). pyzbar is primary and OpenCV's detector is the fallback. The report says "WeChat"; the code uses `cv2.QRCodeDetector`. **Two consecutive identical decodes** are required.
- **Banner:** HSV green H 38–85, S 60–255, V 60–255. Closing then opening. Largest contour above 4000 px. The centroid drives a proportional yaw controller until within **40 px of centre**.
- **Search:** lawnmower. The report claims 4 m strips at 0.8 m/s ≈ 100 s, which is **mathematically wrong** (≈320 m ⇒ 400 s). Our values are 8 m strips at 1.2 m/s (check the real value in params; see §6.1).
- **Centring:** IBVS to ±25 cm. Descend to 5 m (±0.1 m) with the altitude PID active.
- **Red zone:** dual-range HSV (H 0–10 and 170–180). The report's potential field is replaced by "remove the toward-motion".
- **Corridor:** forward camera at 640×480, Gaussian blur, Canny, probabilistic Hough, **vertical lines (>70° from horizontal)** classed left/right. Lateral offset is converted to metres by an FOV formula at 3 m. A forward LiDAR is used as a bumper, plus dead reckoning cross-checked with GPS at the entry. We replaced the pixel-midpoint method with ground projection (§3.3) and made LiDAR optional.
- **Payload:** the report says the **gripper servo opens at 5 m**. The rulebook describes a pulley winch and release on the ground. **The user decided to follow the report (direct drop)**, with `PAYLOAD_MODE = "winch"` as a one-line switch. Risk: the "altitude compliance" scoring and a Change Request if inspectors object.
- **Safety table:** RTL, battery fail-safe, telemetry fail-safe, RC fail-safe, pre-arm checks, geofence, and a Pi-side mission abort.

---

## 5. Current state: what is complete (Phases 1–5, ~65% of the code)

**All components below are written, mock/simulator-tested and pushed. None has flown.**

### 5.1 Foundation
- **`config/params.py`**
  - **Altitude set-points:** 5 m start QR, 3 m corridor, 10 m delivery, 5 m drop, RTL 15 m.
  - **Altitude PID:** kp 0.6, ki 0.008, kd 0.12, max vertical speed 1.5 m/s. The rangefinder is trusted below ~8 m, with a ±1 m blend to the barometer.
  - **Camera:** FOV 84°×64°, 1280×720, `CAMERA_MODE = "single_fixed"`, `CAM_MOUNTS` (single_fixed 65/65, dual 90/10, servo 90/0).
  - **Mission clock and failsafes:** `MISSION_TIME_LIMIT_S` 900, `MISSION_EMERGENCY_MARGIN_S` 60, `EMERGENCY_TIMEOUT_S`, `STATE_TIMEOUTS` (20 entries; the worst-case sum must stay under 840 s, which a test enforces). Battery RTL below 25 % or 14.4 V. `REQUIRED_PARAMS` (FS_THR_ENABLE=1, FS_GCS_ENABLE=1, FS_BATT_ENABLE=2, FENCE_ENABLE=1, FENCE_ACTION=1).
  - Values are as last written; **always re-read the actual file** before editing.
- **`hardware/vehicle.py`** is the only dronekit user. It provides arming, takeoff (blocking and non-blocking), body/NED velocity, yaw-rate velocity, servo output, rangefinder (`None` if invalid), battery, heading, GPS, `goto`, `distance_to_m` (haversine), parameter verification, RTL and land. `MockVehicle` has the same interface (a landed or RTL mock disarms).
- **`navigation/altitude.py`:** `compute()` returns vz (NED) without sending. State handlers combine it with lateral commands in one message. `update()` also sends.

### 5.2 Camera layer
`hardware/camera.py` gives `CameraRig` with `view("down"|"forward")` returning `CameraView(capture_array(), pitch_deg)`.
- **single_fixed:** both views come from one camera.
- **dual:** one camera per view.
- **servo:** a `CameraServo` tilts the camera on view changes (AUX channel 11 PWM mapping, to be calibrated).
- Backends are `webcam` (OpenCV, converted to RGB) and `picamera2`.

### 5.3 Vision
- **QR:** `RobustQRScanner.decode_all(frame, want=None)` returns every QR with its pixel centre. It stops early when `want` is found, and uses an OpenCV fallback. `QRConfirmer` needs `QR_CONFIRM_COUNT` consecutive identical reads. `QRSystem.scan_step` runs one frame of the start scan. `find_target()` returns a `TargetFix` (ground offset) for the QR matching `delivery_id`, using the camera's tilt.
- **Speed warning:** a frame with no QR costs ≈175 ms on a laptop (≈72 ms with one). The Pi will be slower, so profile it in Phase 8.
- **Banner:** a green mask, closed then opened, and the largest contour that passes the area, solidity and fill checks. It returns `offset_px` (positive = banner to the right).
- **Red zone:** HSV dual-range mask, opened (no closing), then the area, solidity and fill-ratio filters. `nearest()` ground-projects the contour points and returns the nearest edge (forward, right, distance).
- **Corridor:** described in §3.3. Wall window 0.4–3.1 m, lines within ±35° of straight ahead. With one wall visible, the centre is derived from the known 3.5 m width. Output is `CorridorFix(center_offset_m, heading_error_deg, left_m, right_m, walls, confidence)`.

### 5.4 Navigation controllers
- **VisualServo:** PD in the body frame with vector speed limiting.
- **Banner align:** `yaw_rate_for_offset` is proportional (0.03 °/s per px, capped at 30°/s).
- **Corridor controller:** lateral PID (kp 0.25, clamp 0.25 m/s) on the centre offset, yaw proportional to heading error, forward speed 0.4 m/s. Speed is halved when more than 0.8 m off centre and ×0.7 with a single wall. It stops completely if no fix.
- **Obstacles:** a pluggable list of sensors. The camera sensor sees dark objects on the ground. A LiDAR wrapper exists (`LidarObstacleSensor` takes anything with `distance_m()`). **No TF-Luna driver has been written yet.** `default_sensors(lidar=driver)` adds one later.
- **Red avoidance:** removes velocity toward the zone (full stop inside 2.5 m, fading to 6 m) plus a capped push.

### 5.5 Mission layer
- **State machine** (`state_machine.py`), each tick:
  1. Ask the watchdog. "stop" means the pilot took over, so return immediately with no more commands. "emergency" goes to EMERGENCY.
  2. Check the mission clock (900 − 60 s).
  3. Check the state's clock, which leads to its fallback.
  4. Run the handler. Any exception goes to EMERGENCY.
  5. Reject illegal transitions (`allowed_next`). Anything not in NEXT, the timeout fallback or EXTRA_ALLOWED is a bug, so go to EMERGENCY.
  - It returns a `MissionResult` (final state, success, reason, payload_released, elapsed, history).
- **States (22):** INIT → PREFLIGHT → ARM → TAKEOFF → MOVE_TO_QR_POINT → SCAN_START_QR → FIND_BANNER_FWD → DESCEND_TO_CORRIDOR → CORRIDOR_FORWARD → CLIMB_TO_DELIVERY → SEARCH_DELIVERY → CENTER_OVER_QR → DESCEND_TO_DROP → DEPLOY_PAYLOAD → CLIMB_AFTER_DROP → FIND_BANNER_RTN → DESCEND_TO_CORRIDOR_RTN → CORRIDOR_RETURN → RETURN_TO_HOME → LAND → DONE (terminal) and EMERGENCY.
- **Retry logic:**
  - A SEARCH_DELIVERY timeout falls back to FIND_BANNER_RTN (go home).
  - CENTER_OVER_QR can retry SEARCH_DELIVERY up to `CENTER_RETRY_MAX` times, then FIND_BANNER_RTN.
  - The exact fallback table is in `mission/states.py` (**read the file; this document is from memory**).
- **Real handlers:** all of the following are real.
  - PREFLIGHT (battery ≥ 80 %, failsafe params, gripper closed).
  - Takeoff (starts the mission clock and records home GPS).
  - The 1 m hop.
  - Start-QR scan.
  - Banner alignment (spin clockwise at 15°/s until seen, then P-turn; aligned after 3 frames within 40 px).
  - Altitude changes (DESCEND_TO_DROP keeps the target centred).
  - CENTER_OVER_QR (servo tick logic).
  - DEPLOY_PAYLOAD.
  - CORRIDOR_* (camera detector + controller + distance tracker; ends at length 10 m + 1.5 m margin, or "walls gone for 1 s within 2 m of the end").
  - RETURN_TO_HOME (`goto` home, then LAND within `HOME_RADIUS_M`).
  - LAND and EMERGENCY (RTL, then DONE once disarmed).
- **SEARCH_DELIVERY is currently a placeholder.** It hovers and looks, going to CENTER_OVER_QR if `find_target()` sees the target and to FIND_BANNER_RTN if there's no ID. This is what Phase 6 replaces.
- **Payload:** `PayloadReleaser.step(now)` is non-blocking. `gripper_drop` opens the gripper (channel 10, PWM 1900 µs) and waits `PAYLOAD_RELEASE_WAIT_S`. `winch` lowers, stops, opens, retracts and stops.

### 5.6 Unexplained files, adopted or discarded
- `mission/states.py`, `mission/context.py` and `safety/watchdog.py` (plus a rewritten `params.py` with `PAYLOAD_MODE` and 22-state timeouts) appeared in the assistant's working folder without being authored in this conversation. They were reviewed, **adopted** and are in the user's repo.
- A duplicate corridor implementation (`navigation/corridor_nav.py`, `vision/obstacle_detector.py`, a `sim/` folder and a second corridor params block) was **discarded**. If these ever appear, ignore them.
- The watchdog raises "stop" when the flight mode leaves GUIDED, and "emergency" on low battery or voltage. **Re-read `safety/watchdog.py` before changing it.**

### 5.7 Testing and simulation assets
- `FlightSim` simulates body-frame motion with first-order lag, yaw rate, altitude and fake GPS (`lat = n/111320`, `lon = e/111320`), all in simulated time.
- `BannerWorldCamera` draws a green banner according to heading.
- `CorridorWorldCamera` draws a 3D corridor (walls, optional dark boxes) with the flight code's camera model.
- Mission-level tests stub the corridor handlers (`sim_handlers()` in `tests/test_state_machine.py`), because the mission-level fake camera has no corridor.
- Closed-loop corridor tests fly from 5 start poses, and test obstacle side-stepping and one-wall flight.

### 5.8 Helper scripts (for the user to try with a webcam)
- `scripts/camera_check.py` shows QR detection and ground offsets.
- `scripts/tune_hsv.py` gives colour sliders. Press `p` to print the numbers and `q` to quit.
- `scripts/detect_check.py --detect banner|red` shows what the real detectors do.
- `scripts/corridor_check.py --height H [--image file]` shows the corridor fix.

The user said the live webcam check of `corridor_check` was confusing. It only makes sense with a real hallway or two rows of boxes, so it is safe to postpone until a physical corridor exists.

---

## 6. Current work (Phase 6) and remaining ~35–40%

### 6.1 Phase 6: lawnmower search with red-zone avoidance (**in progress**)

**Written in the previous sandbox only. The user has NOT received or applied any of it.** Recreate and deliver it inline:

1. **`config/params.py`**
   - Add the block below.
   - Change `SEARCH_TIMEOUT` to 220.
   - Change `STATE_TIMEOUTS["SEARCH_DELIVERY"]` from 175 to **225**. The full pattern needs ~190 s, and the worst-case sum becomes ~770 s, which is under 840 s (the existing test checks this).
   - **Check the real `SEARCH_STRIP_W`.** My notes assumed 8 m (which gives 4 strips). The default path generated in the sandbox came out as **3 strips (6 waypoints, 134 m, ~112 s at 1.2 m/s)**, which implies the params file has 10 m, not 8 m. Read the file. Strip width must also be validated against the real **QR decode range** (see §6.3).
   ```python
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
   ```
2. **New file `navigation/search.py`** (pure logic). Appendix A has the core code.
   - **`LocalFrame(lat0, lon0, heading0)`:** `.pose(lat, lon, heading)` returns `(x ahead, y right, heading relative to entry)`. Latitude and longitude convert at 111,320 m/deg, with a cos(lat) factor for longitude.
   - **`lawnmower_path(length, left, right, strip, margin, start_y)`:**
     - `n = ceil((left + right − 2·margin)/strip)` strips with even centre-lines.
     - Start at the end strip nearest the drone's `y`.
     - Waypoints `(x0, y_i, 0°) → (x1, y_i, 0°)` then `(x1, y_{i+1}, 180°) → (x0, y_{i+1}, 180°)` and so on.
     - Return strips have heading 180°, because the tilted forward camera must face the direction of travel.
   - **`SearchPlanner.step(pose, nearest_red, now)`:** returns a body-frame `SearchCommand(vx, vy, yaw_rate, status)`, or `None` when the path is finished.
     - A waypoint is reached within 1.5 m and 10° of heading.
     - It flies toward the waypoint with a proportional speed (capped at `SEARCH_SPEED`).
     - It yaws toward the waypoint heading (a heading error above 45° cuts translation to 30 %).
     - It converts the entry-frame vector to the body frame.
     - It then runs the red-zone avoidance: `adjust_velocity`, plus a **slide** along the zone's edge when it is within 4 m and in the way. The tangent is chosen toward the goal. If head-on, it takes the roomier side, and holds the choice for 2 s.
3. **New file `tests/arena_world.py`** (a tiny fake delivery zone). `ArenaWorldCamera(veh, pitch_deg, qrs=[(text, n, e, size)], red_zones=[(n0, n1, e0, e1)])` renders by **inverse mapping** each pixel to the ground in the entry frame. It paints grass, red rectangles and QR bitmaps, with the sky above the horizon. `fix_for(n, e)` is a fake perfect QR finder (visible if inside the image and within `decode_range_m`, 12 m slant by default). Real decoding is far too slow for the sim (≈175 ms per frame), so tests patch `ctx.qr.find_target` with this.
4. **Replace `h_search_delivery` in `mission/handlers.py`** with a tick-based real version:
   - If `delivery_id is None` → log, return FIND_BANNER_RTN.
   - On first tick: set the altitude target to `ALT_DELIVERY` (10 m). Create `LocalFrame` from the current GPS and heading. Create `SearchPlanner`. Create `ctx.rig.view("down")` and `RedZoneDetector()` (keep both in `ctx.sd`, no new context fields).
   - Every tick: if `ctx.qr.find_target()` sees the target → return CENTER_OVER_QR. Otherwise compute the pose, capture a frame, run `red.nearest(frame, height, pitch)` and call `planner.step`. If it returns `None` (path finished without finding the target) → return FIND_BANNER_RTN. Else `vehicle.send_velocity_yawrate(cmd.vx, cmd.vy, ctx.altitude.compute(), cmd.yaw_rate_dps)`.
   - Imports needed: `LocalFrame`, `SearchPlanner` from `navigation.search`, and `RedZoneDetector` from `vision.red_zone`.
5. **Tests to write:**
   - Path geometry: strip count and coverage, stays within margins, alternating headings, nearest end first, total time under the 225 s state budget.
   - `LocalFrame` with a rotated entry heading.
   - Closed loop in `FlightSim` + `ArenaWorldCamera`: finds a target placed in the zone; **never enters a red zone** (check the position against the rectangle expanded by ~0.3 m every tick) while still finishing or finding the target when a red zone blocks a strip; stays inside the zone bounds; with no target, finishes the path and returns FIND_BANNER_RTN before the timeout.
   - Re-run the whole suite (the existing mission tests, whose fake `find_target` is always visible, must still pass), plus mutation checks (slide sign, avoidance removed, waypoint order).
6. Deliver inline with beginner steps. Then give the commit and tag commands: `git tag phase-6-search`.

### 6.2 Remaining phases (~35–40% of the whole project)

| Phase | Work |
|---|---|
| 7 | **Flight data recorder** (rulebook requirement: record and share flight data with the jury). Extra safety: a battery/datalink/geofence watchdog thread, pre-arm checks, program the geofence (coordinates arrive in Phase 2). |
| 8 | `main.py` entry point and wiring (`MissionContext` with real hardware or mock), a Pi setup guide (UART TELEM2 wiring, picamera2, serial permissions), **profile QR decode speed on the Pi** and trim the decoder if needed, requirements for the Pi, and a README rewrite. |
| 9 | **Simulation:** ArduPilot SITL (the user's old tests connect to `udp:127.0.0.1:14550`) running the full state machine against a real flight model. Add a synthetic camera driven by the SITL pose, combining the arena, corridor and banner renderers so the whole vision pipeline runs in SITL. |
| 10 | Bench tests with props off, tethered hover, field tuning: **re-tune HSV outdoors** (banner, red zone; grass is the main false-positive risk), choose the real camera tilt and check the banner is visible, **validate corridor perception on real walls**, measure the **QR decode range at 10 m**, tune speeds for the time score, enter the real geofence, rehearse the technical inspection (weight within 50 g, failsafe checks, **Change Request** if needed). |

### 6.3 Open risks and decisions that need humans
1. **Camera layout.** Fixed 65° tilt trades forward visibility against straight-down visibility. The banner must be within ~7.7 m ahead at 5 m height. Decide among single, dual (USB second camera) and servo after real tests.
2. **QR printed size is not stated in the rulebook.** At 10 m altitude a 1 m QR is only about 60 px wide at 1280 px, which is marginal for decoding. This drives strip spacing and altitude.
3. **Corridor geometry ambiguity** (two lanes or one; wall appearance). See §4.2.
4. **Payload mechanism vs. rulebook** (gripper drop at 5 m as in the report, vs. pulley lowered to the ground). The user chose the report's method.
5. **Where the drone enters the delivery zone** (`SEARCH_ENTRY_FROM_LEFT_M`) and the true zone orientation. Derive these from the Phase 2 layout and geofence.
6. **Pi camera colour order and picamera2 behaviour are untested.**
7. **Banner never found → RTL.** This policy gives up the corridor and return marks. Offer the user the "fly on with the current heading" alternative if they prefer.
8. **A forward LiDAR is deferred.** It is only needed if the camera-only corridor proves unreliable. A TF-Luna driver (9-byte frame `59 59 dist_L dist_H str_L str_H temp_L temp_H checksum`, distance in cm, reject strength < 100 or 0xFFFF) was drafted and tested once but **removed because the hardware isn't available**.
9. **README is stale** (it describes the old flat layout and a dual-LiDAR plan).
10. **Dev environment mismatch:** the user's laptop is Windows with a Python 3.11 venv; the Pi runs Linux with Python 3.11. Keep dependencies compatible.

---

## Appendix A: `navigation/search.py` core (recreate this, then add tests)

```python
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

    def _avoid(self, vf, vr, red: Optional[NearestRed], y, now):
        if red is None or red.distance_m >= RED_INFLUENCE_M:
            self._slide = None; return vf, vr, "ok"
        nvf, nvr = adjust_velocity(vf, vr, red)
        d = max(red.distance_m, 1e-3); ux, uy = red.forward_m/d, red.right_m/d
        toward = vf*ux + vr*uy; status = "red_slowing"
        if d < SEARCH_SLIDE_TRIGGER_M and toward > 0.1:
            if self._slide is None or now > self._slide[2]:
                t1 = (-uy, ux); dot = vf*t1[0] + vr*t1[1]
                if abs(dot) > 0.25*math.hypot(vf, vr): t = t1 if dot > 0 else (-t1[0], -t1[1])
                else:
                    prefer_right = (self.right_m - y) >= (y + self.left_m)
                    t = t1 if (t1[1] > 0) == prefer_right else (-t1[0], -t1[1])
                self._slide = (t[0], t[1], now + SEARCH_SLIDE_HOLD_S)
            nvf += SEARCH_SLIDE_SPEED*self._slide[0]; nvr += SEARCH_SLIDE_SPEED*self._slide[1]; status = "red_slide"
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
        vf, vr, status = self._avoid(vf, vr, red, y, now)
        return SearchCommand(vf, vr, yaw, status)
```

## Appendix B: git and release routine
Tags so far: phases 1–5 were committed and pushed. Per the user's convention the tags are `phase-3-mission-skeleton`, `phase-4-banner-redzone`, `phase-5-corridor`. The Phase 1 and 2 tags may not exist yet, so suggest `git log --oneline` and `git tag phase-2-vision <hash>`. Next tag: `phase-6-search`.

## Appendix C: first actions in the new chat
1. Ask the user to confirm the repo state: run the pytest command and report the count (expect 130), and paste `config/params.py`, `mission/states.py`, `mission/handlers.py` and `safety/watchdog.py` (or provide blob URLs).
2. Check `SEARCH_STRIP_W` and the actual fallback table in `states.py`.
3. Deliver Phase 6 inline, in this order: params edits, then `navigation/search.py`, then `tests/arena_world.py`, then the handler replacement, then the tests. Test each piece in a sandbox before sending.
4. Then proceed with Phases 7–10 as above.