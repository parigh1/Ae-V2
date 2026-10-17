# Team Vajra — AeroTHON 2026, Mission 2 "SkyScan: Autonomous Rapid Delivery"

Autonomous-flight software for the Team Vajra quadcopter (VIT Bhopal University, Team ID AT2026RC49), SAEINDIA AeroTHON 2026, Track 1 "Rotorcraft Systems Challenge (UAS)".
A **Raspberry Pi 4** makes every decision; a **Pixhawk 2.4.8 running ArduPilot** does the flying.

> **Status: written and simulator-tested, not yet flown.** The full mission logic, vision, safety layer and flight recorder exist and pass the automated test suite on a Windows laptop and on the Raspberry Pi.
> The Pi camera has been checked on the bench. **Nothing has been connected to the Pixhawk or flown.** Read "Not yet verified on real hardware" before trusting any number in this repo.

---

## 1. The mission (rulebook 4.2.4)

1. Take off autonomously and climb to **5 m** (the 15-minute clock starts when the take-off throttle rises).
2. Move about **1 m** forward and scan the **start QR**: it gives the delivery location ID.
3. Find the **green AeroTHON banner** at the corridor entrance and line up with it.
4. Descend to **about 3 m (10 ft)** and fly the **3.5 m wide, ~10 m long corridor**, avoiding static obstacles.
5. Climb to **about 10 m** over the **40 m x 30 m delivery zone**. Find the QR that matches the ID, avoiding the **red no-fly zones** (-5 marks each).
6. Centre over it, descend to **5 m** and drop the **100 g payload**.
7. Climb to 10 m, find the banner again, fly back through the corridor, return to the take-off point and **land by itself**.

Marks (preliminary): start QR 10, corridor 15, zone QR 10, red-zone avoidance 10, payload accuracy 15, return through corridor 20, landing 5, speed 15. Technical inspection is 50 marks (it checks the failsafes in section 4).

---

## 2. How it works

The Pi runs one Python program. It reads the camera and the Pixhawk's telemetry, decides what to do, and sends **velocity commands** to the Pixhawk in `GUIDED` mode. ArduPilot only stabilises.

```
 Camera ──> vision (QR, banner, red zone, corridor) ─┐
                                                     ├─> MissionContext ─> StateMachine ─> velocity commands ─> Pixhawk
 Pixhawk telemetry ─> Vehicle wrapper ───────────────┘          │
                                                                ├─ Watchdog        (battery, link, GPS, geofence, pilot takeover)
                                                                └─ FlightRecorder  (logs/ folder)
```

Only `hardware/vehicle.py` imports DroneKit. Everything else talks to the `Vehicle` / `MockVehicle` interface, so the whole mission can be tested with no drone.

### The 22 states, in flying order

`INIT` > `PREFLIGHT` > `ARM` > `TAKEOFF` > `MOVE_TO_QR_POINT` > `SCAN_START_QR` > `FIND_BANNER_FWD` > `DESCEND_TO_CORRIDOR` > `CORRIDOR_FORWARD` > `CLIMB_TO_DELIVERY` > `SEARCH_DELIVERY` > `CENTER_OVER_QR` > `DESCEND_TO_DROP` > `DEPLOY_PAYLOAD` > `CLIMB_AFTER_DROP` > `FIND_BANNER_RTN` > `DESCEND_TO_CORRIDOR_RTN` > `CORRIDOR_RETURN` > `RETURN_TO_HOME` > `LAND` > `DONE`, plus `EMERGENCY` (reachable from anywhere).

Altitudes: 5 m start QR, 3 m corridor, 10 m search, 5 m drop, 15 m RTL clearance.

---

## 3. Camera: one fixed camera at the front

The mission uses **one Raspberry Pi Camera Module v2 (IMX219), fixed at the front, tilted 72° below the horizon** (design value; 90° = straight down, 0° = straight forward). `CAMERA_MODE = "single_fixed"`; the `dual` and `servo` modes still exist in the code but are not used.

**Sensor mode.** Left alone, picamera2 turns a 1280x720 request into a narrow centre crop (about 39° wide). `CAM_SENSOR_MODE = (1640, 1232)` forces the full-width binned mode, and the picture is **1280 x 960 (4:3)**:

| | Value | Where it came from |
|---|---|---|
| Field of view | **62.2° horizontal x 48.7° vertical** | estimated by `scripts/fov_check.py` from the sensor crop (ruler check still to do) |
| Picture | 1280 x 960, RGB | `CAM_IMG_W`, `CAM_IMG_H` |
| Frame rate | about 30 fps | measured on the Pi |

**Why 72°.** A fixed camera has to serve both the forward-looking stages and the straight-down drop. Calculated for this camera (not yet measured on the airframe):

| Stage | Height | Ground covered, metres ahead of the point under the drone | Note |
|---|---|---|---|
| Start QR | 5 m | −0.6 to 4.6 | QR must be within about 4 m |
| Banner | 5 m | base visible up to **4.6 m** | the banner must be within about 4.5 m |
| Corridor | 3 m | −0.3 to 2.7, picture ±1.9 m wide | walls (±1.75 m) just fit; short look-ahead |
| Search | 10 m | −1.1 to 9.1 | fine |
| Centre and drop | 5 m | a 1 m QR directly below stays fully in view | needs pitch of at least 71.4° |

Consequences you should know about:

- The drop needs a steep angle (a QR directly below the drone only stays fully visible at 71.4° or more for a 1 m code, 68.5° for 0.5 m). A flatter angle sees the banner farther away but leaves the drone hovering short of the target.
- The corridor look-ahead is only about 2.7 m. Near the end of the corridor the walls run out of view before the corridor does, so the drone **creeps out on distance alone** (`CORRIDOR_COAST_M`, `CORRIDOR_COAST_SPEED`).
- A single visible wall is only trusted if it is the **near** one (within about 2 m to the side). A wall seen alone from farther away is in view for too short a stretch.

**Measuring the real mount angle.** The software only knows the angle you tell it. Hold the drone level with the camera exactly 1.00 m above the floor. Put a small marker on the floor straight ahead of the lens until it appears in the exact centre of the picture, and measure its distance d from the lens: pitch = atan(1.00 / d). For 72° that is d = 0.32 m (70°: 0.36 m, 75°: 0.27 m). Put the measured value in `CAM_MOUNTS["single_fixed"]` in `config/params.py`. With a phone: lay it flat against the camera board; for 72° the board is tilted 18° from horizontal.

---

## 4. Safe by default

- **Every state has a time limit and a fallback** (`mission/states.py`, `STATE_TIMEOUTS`). A test checks that the worst-case total stays under the 15-minute limit.
- **Any handler crash, illegal state jump, low battery, or "14 minutes used" goes to `EMERGENCY`:** the Pi switches the Pixhawk to `RTL`, retries if that fails, and falls back to `LAND`.
- **The pilot always wins.** If the RC mode switch leaves GUIDED / RTL / LAND, the Pi stops sending commands immediately.
- **Pre-flight refuses to arm** unless: battery level is reported and at least 80 %, the flight controller heartbeat is alive, GPS has a 3D fix with at least 8 satellites, the EKF has settled, the failsafe parameters are correct (`FS_THR_ENABLE`, `FS_GCS_ENABLE`, `FS_BATT_ENABLE`, `FENCE_ENABLE`, `FENCE_ACTION`), and the flight log works.
- **The watchdog watches during flight:** pilot takeover, Pixhawk heartbeat, battery level and voltage (missing data is never treated as "full"), GPS fix, and a software geofence (distance from take-off and height; **placeholder limits** until the real geofence is known). Each check is debounced so one glitch cannot abort a flight.
- **Policies you should know about:**
  - Start QR unreadable: skip the search and drop, but still fly the corridor and come home.
  - Banner never found within 35 s: abort and return (this gives up the corridor marks).
  - Target QR not found in the zone: skip the drop and go home.
  - Lost in the corridor (walls not visible): the drone stops and waits for the state timeout, then returns.

---

## 5. Hardware (must match the design report)

- H-frame quad, wheelbase 438.88 mm, take-off weight about 1964 g with payload (limit 2000 g).
- 4x XING 2814 880KV motors, 9x4.5 props, 4x 40 A ESCs, 4S 4000 mAh LiPo.
- **Pixhawk 2.4.8** (ArduPilot) linked to a **Raspberry Pi 4** over TELEM2 (`/dev/ttyAMA0`, 921600 baud).
- **Raspberry Pi Camera Module v2 (IMX219)**, one fixed camera at the front. The design report lists the Camera Module 3 Wide, so a Change Request or a corrected report may be needed before inspection.
- One TF-Luna downward rangefinder, Pixhawk GPS. SG90 gripper servo for the payload drop. No winch, no forward LiDAR.

---

## 6. Repository layout

```
config/params.py            ALL constants. Nothing else contains magic numbers.
hardware/vehicle.py         Vehicle (DroneKit wrapper) + MockVehicle (same interface, no hardware)
hardware/camera.py          CameraRig, Webcam / PiCamera / Fake cameras (PiCamera forces the sensor mode)
vision/                     QR scanner/system, pixel->metres, banner, red zone, corridor detectors
navigation/                 altitude PID, visual servo, banner align, red-zone avoidance, corridor, lawnmower search
mission/                    states.py, context.py, state_machine.py, handlers.py
safety/watchdog.py          in-flight health checks
payload/release.py          gripper drop (or winch) sequence
telemetry/recorder.py       flight data recorder + CommandTap
scripts/                    camera_check.py, tune_hsv.py, detect_check.py, corridor_check.py, fov_check.py
tests/                      pytest suite + simulators (sim_helpers.py, corridor world, arena world)
logs/                       flight logs (created at run time, ignored by git)
```

---

## 7. Setup

### Laptop (Windows, PyCharm)

Python **3.11** in a virtual environment (Python 3.14 breaks the pinned numpy).

```
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

`pyzbar` needs the Microsoft Visual C++ 2013 x64 redistributable if it complains about a missing zbar DLL.

### Raspberry Pi 4 (Raspberry Pi OS / Debian 13, Python 3.13)

```
sudo apt install git python3-venv python3-picamera2 libzbar0
git clone https://github.com/parigh1/Ae-V2.git
cd Ae-V2
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

- `--system-site-packages` lets the venv see `picamera2` (installed by apt).
- The first `pip install` takes several minutes because numpy 1.26.4 is built from source on Python 3.13.
- **Always run `source .venv/bin/activate` first.** Without it, `pip` stops with `error: externally-managed-environment`.
- The `types-seaborn ... requires matplotlib` lines at the end of the install are harmless.
- The camera driver is picked automatically (`picamera2` on the Pi, a webcam on a laptop).

**Quick camera check on the Pi** (saves a photo; copy it to the laptop with `scp raspberrypi@raspberrypi:~/Ae-V2/test.jpg .` and look at it):

```
python -c "import time, cv2; from hardware.camera import PiCamera; c = PiCamera(); time.sleep(2); f = c.capture_array(); c.close(); cv2.imwrite('test.jpg', cv2.cvtColor(f, cv2.COLOR_RGB2BGR)); print(f.shape, f.dtype)"
```

Expected: `(960, 1280, 3) uint8`. Check that red looks red, that things at the distances you care about are sharp (the v2 has a fixed lens), and measure the field of view with a ruler on a wall at exactly 1.00 m.

Wiring and serial setup between the Pi and the Pixhawk (TELEM2) will be documented with `main.py` in Phase 8.

---

## 8. Run the tests

```
python -m pytest -v --ignore=tests/test_sitl_connect.py --ignore=tests/test_altitude_sitl.py
```

All tests must pass (about 20 s on the laptop, about 70 s on the Pi). The two ignored files are older tests that need an ArduPilot SITL running on `udp:127.0.0.1:14550`.

---

## 9. Settings you will change

Everything lives in `config/params.py`. The ones that depend on the real world:

| Setting | What it is |
|---|---|
| `CAM_MOUNTS["single_fixed"]` | the measured camera tilt below the horizon (design value 72) |
| `CAM_SENSOR_MODE` | forces the v2's full-width sensor mode; `None` lets picamera2 crop to a narrow view |
| `CAM_FOV_H_DEG`, `CAM_FOV_V_DEG`, `CAM_IMG_W`, `CAM_IMG_H` | measured field of view and picture size; every ground-distance calculation uses them |
| `BANNER_*`, `RED_*` HSV ranges | **must be re-tuned outdoors** with the real banner and red zones (`scripts/tune_hsv.py`) |
| `SEARCH_ENTRY_FROM_LEFT_M`, `SEARCH_STRIP_W` | where the drone enters the 40 x 30 m zone, and the gap between search passes |
| `PY_FENCE_RADIUS_M`, `PY_FENCE_MAX_ALT_M` | software geofence, **placeholders** until the real geofence is known |
| `PAYLOAD_MODE` | `"gripper_drop"` (as in the design report) or `"winch"` (as the rulebook figure describes) |
| `REQUIRED_PARAMS` | ArduPilot failsafe parameters checked before arming |

---

## 10. Flight logs

Each flight writes three files into `logs/`, named by start time (for example `flight_20261007_143005`):

- `.csv`: one row per control tick (10 per second): mission time, state, flight mode, position, heights, heading, battery, the last velocity command, delivery ID.
- `_events.txt`: every line the mission prints (state changes, decisions, warnings).
- `_summary.json`: the result (final state, success or reason for failure, elapsed time, state history) and a copy of every setting in `params.py`.

The recorder can never stop the flight: any disk problem simply switches it off with one warning. Copy the `logs/` folder off the Pi after each flight; it is not stored in git. The Pixhawk also keeps its own log on its SD card (check that one is fitted and logging).

---

## 11. Helper scripts (try them with the camera, no drone needed)

- `scripts/camera_check.py`: live QR detection and ground offsets.
- `scripts/tune_hsv.py`: colour sliders for the banner and red zone (`p` prints the numbers, `q` quits).
- `scripts/detect_check.py --detect banner|red`: shows what the real detectors see.
- `scripts/corridor_check.py --height H [--image file]`: shows the corridor measurement (only meaningful with a real corridor).
- `scripts/fov_check.py`: run on the Pi; estimates the field of view and frame rate for several camera set-ups.

---

## 12. Development status

Software phases (not the competition's Phase 1 / Phase 2):

| Phase | Work | Status |
|---|---|---|
| 1-2 | Vehicle wrapper, camera layer, QR pipeline, altitude PID, visual servo, payload release | done |
| 3 | Mission state machine | done |
| 4 | Banner and red-zone vision | done |
| 5 | Corridor navigation (camera only) | done |
| 6 | Lawnmower search with red-zone avoidance, skipping unreachable waypoints | done |
| 7 | Flight data recorder, pre-flight checks, watchdog health checks, persistent emergency RTL | done |
| 7+ | Camera: full-width sensor mode, 62.2 x 48.7 deg picture, single fixed camera, creep-out at the corridor end | done (simulation and Pi bench only) |
| 8 | `main.py`, Pi setup guide, Pi to Pixhawk wiring notes, profiling on the Pi | **next** |
| 9 | ArduPilot SITL with a synthetic camera | planned |
| 10 | Bench tests (props off), tethered hover, outdoor tuning, inspection rehearsal | planned |

### What remains

- **Phase 8:** `main.py` (builds everything, asks for a deliberate start, closes the log, hands the drone to RTL if anything goes wrong), a setup guide for the UART between the Pi and the Pixhawk, and measuring per-frame processing time on the Pi.
- **Phase 9:** run the whole mission in ArduPilot SITL against the real `Vehicle` class and a synthetic camera that combines the arena, corridor and banner renderers. This is the first time the real `Vehicle` health readings (GPS fix, heartbeat age, EKF, battery) and the real velocity frames and yaw signs are exercised.
- **Phase 10 (hardware):** ArduPilot parameter set-up (failsafes, battery monitor, serial port, rangefinder, geofence), RC and mode-switch rehearsal, props-off bench test, tethered hover, tuning, outdoor HSV tuning, corridor perception on real walls, QR decode range at 10 m, payload mechanism test, weight check, inspection rehearsal.
- **Decisions and paperwork:** payload method (see below), camera Change Request, real geofence coordinates, banner and QR distances from the rulebook figure.

---

## 13. Not yet verified on real hardware

- **Everything involving the Pixhawk.** The real `Vehicle` class has never run against an autopilot (real or simulated), including the Phase 7 health readings.
- **Camera angle.** The design value is 72° below the horizon. The banner is only visible to about 4.6 m ahead from 5 m height, so the real start-to-banner distance must be checked. The lens-to-gripper offset has not been measured.
- **Field of view.** 62.2 x 48.7 degrees is an estimate from the sensor crop; confirm it with the ruler test.
- **QR decoding range.** The QR print size is not in the rulebook (1 m x 1 m is assumed). From 10 m a 1 m code is about 100 px wide with this lens; measure on the real camera. The v2 has a fixed lens, so check focus at long range.
- **Speed on the Pi:** a frame with no QR code took about 175 ms on a laptop; the Pi is slower. Profiling is part of Phase 8.
- **Corridor perception on real walls.** Only tested on a synthetic renderer. The rulebook figure may show two lanes; the code assumes one 3.5 m channel with visible wall bases.
- **HSV thresholds outdoors.** Grass is the main false-positive risk.
- **Payload method.** The design report says the gripper opens at 5 m; the rulebook figure says the payload is lowered to the ground on a pulley. We follow the report. This may need a Change Request.
- **Geofence:** the real coordinates arrive in Phase 2 of the competition; the Python limits are placeholders.
- **Battery reporting:** pre-flight refuses to arm if the Pixhawk does not report battery level and voltage. Typical fix on the ArduPilot side: set `BATT_MONITOR` for the power module and `BATT_CAPACITY` to the battery size (confirm in Mission Planner).

---

## 14. Conventions that must not be broken

- Velocity commands use `MAV_FRAME_BODY_OFFSET_NED` through `Vehicle.send_body_velocity(vx_forward, vy_right, vz_down)`. **vz is NED: negative means climb.** Yaw rate is in deg/s, positive = clockwise.
- Camera frames are RGB `uint8`, `H x W x 3`. Image +x is right, +y is down. `forward_m > 0` is ahead, `right_m > 0` is to the right.
- Camera pitch is the angle of the optical axis below the horizon (90 = straight down, 0 = straight forward).
- Handlers are non-blocking `handler(ctx) -> Optional[State]`, called every tick (`CONTROL_HZ` = 10). Time and sleeping are injected, so tests run in simulated time.
- Never add a number to the code that belongs in `config/params.py`.