# Team Vajra — AeroTHON 2026, Mission 2 "SkyScan: Autonomous Rapid Delivery"

Autonomous-flight software for the Team Vajra quadcopter (VIT Bhopal University, Team ID AT2026RC49), SAEINDIA AeroTHON 2026, Track 1 "Rotorcraft Systems Challenge (UAS)".
A **Raspberry Pi 4** makes every decision; a **Pixhawk 2.4.8 running ArduPilot** does the flying.

> **Status: simulator-tested only.** 201 automated tests pass on a Windows laptop (Python 3.11) and on the Raspberry Pi (Debian 13, Python 3.13).
> **Nothing has flown yet.** See "Not yet verified on real hardware" below before trusting any number in this repo.

---

## The mission (rulebook 4.2.4)

1. Take off autonomously and climb to **5 m** (the 15-minute clock starts when the take-off throttle rises).
2. Move about **1 m** forward and scan the **start QR**: it gives the delivery location ID.
3. Find the **green AeroTHON banner** at the corridor entrance and line up with it.
4. Descend to **about 3 m (10 ft)** and fly the **3.5 m wide, ~10 m long corridor**, avoiding static obstacles.
5. Climb to **about 10 m** over the **40 m x 30 m delivery zone**. Find the QR that matches the ID, avoiding the **red no-fly zones** (-5 marks each).
6. Centre over it, descend to **5 m** and drop the **100 g payload**.
7. Climb to 10 m, find the banner again, fly back through the corridor, return to the take-off point and **land by itself**.

Marks (preliminary): start QR 10, corridor 15, zone QR 10, red-zone avoidance 10, payload accuracy 15, return through corridor 20, landing 5, speed 15. Technical inspection is 50 marks (it checks the failsafes below).

---

## How it works

The Pi runs one Python program. It reads the camera and the Pixhawk's telemetry, decides what to do, and sends **velocity commands** to the Pixhawk in `GUIDED` mode. ArduPilot only stabilises.

```
 Camera ──> vision (QR, banner, red zone, corridor) ─┐
                                                     ├─> MissionContext ─> StateMachine ─> velocity commands ─> Pixhawk
 Pixhawk telemetry ─> Vehicle wrapper ───────────────┘          │
                                                                ├─ Watchdog        (battery, link, GPS, geofence, pilot takeover)
                                                                └─ FlightRecorder  (logs/ folder)
```

### The 22 states, in flying order

`INIT` > `PREFLIGHT` > `ARM` > `TAKEOFF` > `MOVE_TO_QR_POINT` > `SCAN_START_QR` > `FIND_BANNER_FWD` > `DESCEND_TO_CORRIDOR` > `CORRIDOR_FORWARD` > `CLIMB_TO_DELIVERY` > `SEARCH_DELIVERY` > `CENTER_OVER_QR` > `DESCEND_TO_DROP` > `DEPLOY_PAYLOAD` > `CLIMB_AFTER_DROP` > `FIND_BANNER_RTN` > `DESCEND_TO_CORRIDOR_RTN` > `CORRIDOR_RETURN` > `RETURN_TO_HOME` > `LAND` > `DONE`, plus `EMERGENCY` (reachable from anywhere).

### Safe by default

- **Every state has a time limit and a fallback** (`mission/states.py`, `STATE_TIMEOUTS` in `config/params.py`). The worst-case total is checked by a test and stays under the 15-minute limit.
- **Any handler crash, illegal state jump, low battery, or "14 minutes used" goes to `EMERGENCY`:** the Pi switches the Pixhawk to `RTL`, retries if that fails, and falls back to `LAND`.
- **The pilot always wins.** If the RC mode switch leaves GUIDED / RTL / LAND, the Pi stops sending commands immediately.
- **Pre-flight refuses to arm** unless: battery level is reported and at least 80 %, the flight controller heartbeat is alive, GPS has a 3D fix with at least 8 satellites, the EKF has settled, the failsafe parameters are correct (`FS_THR_ENABLE`, `FS_GCS_ENABLE`, `FS_BATT_ENABLE`, `FENCE_ENABLE`, `FENCE_ACTION`), and the flight log works.
- **The watchdog watches during flight:** pilot takeover, Pixhawk heartbeat, battery level/voltage (missing data is never treated as "full"), GPS fix, and a software geofence (distance from take-off and height; **placeholder limits** until the real geofence is known). Each check is debounced so one glitch cannot abort a flight.
- **Policies you should know about:**
  - Start QR unreadable: skip the search and drop, but still fly the corridor and come home.
  - Banner never found within 35 s: abort and return (this gives up the corridor marks).
  - Target QR not found in the zone: skip the drop and go home.

---

## Hardware (must match the design report)

- H-frame quad, wheelbase 438.88 mm, take-off weight about 1964 g with payload (limit 2000 g).
- 4x XING 2814 880KV motors, 9x4.5 props, 4x 40 A ESCs, 4S 4000 mAh LiPo.
- **Pixhawk 2.4.8** (ArduPilot) linked to a **Raspberry Pi 4** over TELEM2 (`/dev/ttyAMA0`, 921600 baud).
- **Raspberry Pi Camera Module v3 (Wide)**, one TF-Luna downward rangefinder, Pixhawk GPS.
- SG90 gripper servo for the payload drop. No winch, no forward LiDAR.

Camera layout is one line in `config/params.py`: `CAMERA_MODE = "single_fixed" | "dual" | "servo"`.

---

## Repository layout

```
config/params.py            ALL constants. Nothing else contains magic numbers.
hardware/vehicle.py         Vehicle (DroneKit wrapper) + MockVehicle (same interface, no hardware)
hardware/camera.py          CameraRig: single_fixed | dual | servo; Webcam, PiCamera, Fake cameras
vision/                     QR scanner/system, pixel->metres, banner, red zone, corridor detectors
navigation/                 altitude PID, visual servo, banner align, red-zone avoidance, corridor, lawnmower search
mission/                    states.py, context.py, state_machine.py, handlers.py
safety/watchdog.py          in-flight health checks
payload/release.py          gripper drop (or winch) sequence
telemetry/recorder.py       flight data recorder + CommandTap
scripts/                    camera_check.py, tune_hsv.py, detect_check.py, corridor_check.py
tests/                      pytest suite + simulators (sim_helpers.py, corridor_world.py)
logs/                       flight logs (created at run time, ignored by git)
```

Only `hardware/vehicle.py` imports DroneKit. Everything else talks to the `Vehicle` / `MockVehicle` interface, so the whole mission can be tested with no drone.

---

## Setup

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

- `--system-site-packages` is needed so the venv can see `picamera2` (installed by apt).
- The first `pip install` takes several minutes because numpy 1.26.4 is built from source on Python 3.13.
- **Always run `source .venv/bin/activate` first.** Without it, `pip` stops with `error: externally-managed-environment`.
- The `types-seaborn ... requires matplotlib` lines at the end of the install are harmless.
- Set `CAMERA_BACKEND` to `"picamera2"` in `config/params.py` on the Pi (the default is `"webcam"` for laptops).

Wiring and serial setup between the Pi and the Pixhawk (TELEM2) will be documented with `main.py` in Phase 8.

---

## Run the tests

```
python -m pytest -v --ignore=tests/test_sitl_connect.py --ignore=tests/test_altitude_sitl.py
```

Expected: **201 passed** (about 20 s on the laptop, about 66 s on the Pi). The two ignored files are older tests that need an ArduPilot SITL running on `udp:127.0.0.1:14550`.

---

## Settings you will change

Everything lives in `config/params.py`. The ones that depend on the real world:

| Setting | What it is |
|---|---|
| `CAMERA_MODE`, `CAM_MOUNTS`, `CAMERA_BACKEND` | camera layout, tilt angle, driver (`webcam` on a laptop, `picamera2` on the Pi) |
| `CAM_FOV_H_DEG`, `CAM_FOV_V_DEG` | **must be measured on the real camera**; every ground-distance calculation uses them |
| `BANNER_*`, `RED_*` HSV ranges | **must be re-tuned outdoors** with the real banner and red zones (use `scripts/tune_hsv.py`) |
| `SEARCH_ENTRY_FROM_LEFT_M`, `SEARCH_STRIP_W` | where the drone enters the 40 x 30 m zone, and the gap between search passes |
| `PY_FENCE_RADIUS_M`, `PY_FENCE_MAX_ALT_M` | software geofence, **placeholders** until the Phase 2 geofence is known |
| `PAYLOAD_MODE` | `"gripper_drop"` (as in the design report) or `"winch"` (as the rulebook figure describes) |
| `REQUIRED_PARAMS` | ArduPilot failsafe parameters checked before arming |

---

## Flight logs

Each flight writes three files into `logs/`, named by start time (for example `flight_20261007_143005`):

- `.csv`: one row per control tick (10 per second): mission time, state, flight mode, position, heights, heading, battery, the last velocity command, delivery ID.
- `_events.txt`: every line the mission prints (state changes, decisions, warnings).
- `_summary.json`: the result (final state, success or reason for failure, elapsed time, state history) and a copy of every setting in `params.py`.

The recorder can never stop the flight: any disk problem simply switches it off with one warning. Copy the `logs/` folder off the Pi after each flight; it is not stored in git.

---

## Helper scripts (try them with the camera, no drone needed)

- `scripts/camera_check.py`: live QR detection and ground offsets.
- `scripts/tune_hsv.py`: colour sliders for the banner and red zone (`p` prints the numbers, `q` quits).
- `scripts/detect_check.py --detect banner|red`: shows what the real detectors see.
- `scripts/corridor_check.py --height H [--image file]`: shows the corridor measurement (only meaningful with a real corridor).

---

## Development status

Software phases (not the competition's Phase 1 / Phase 2):

| Phase | Work | Status |
|---|---|---|
| 1-2 | Vehicle wrapper, camera layer, QR pipeline, altitude PID, visual servo, payload release | done |
| 3 | Mission state machine | done |
| 4 | Banner and red-zone vision | done |
| 5 | Corridor navigation (camera only) | done |
| 6 | Lawnmower search with red-zone avoidance | done |
| 7 | Flight data recorder, pre-flight checks, watchdog health checks, persistent emergency RTL | done |
| 8 | `main.py`, Pi setup guide, profiling on the Pi, README | in progress |
| 9 | ArduPilot SITL with a synthetic camera | planned |
| 10 | Bench tests (props off), tethered hover, outdoor tuning, inspection rehearsal | planned |

Each phase is tagged in git (`git tag` lists them).

---

## Not yet verified on real hardware

- **Pi camera:** colour order (picamera2 `BGR888` gives RGB in memory), real field of view, tilt angle, banner visibility.
- **QR decoding range.** The QR print size is not in the rulebook (1 m x 1 m is assumed). A 1 m QR is only about 50-70 px wide from 10 m, which is marginal.
- **Speed on the Pi:** a frame with no QR code takes about 175 ms on a laptop; the Pi is slower. Profiling is part of Phase 8.
- **Corridor perception on real walls.** Only tested on a synthetic renderer. The rulebook figure may show two lanes; the code assumes one 3.5 m channel with visible wall bases.
- **HSV thresholds outdoors.** Grass is the main false-positive risk.
- **The Phase 7 health readings** (GPS fix, heartbeat age, EKF, battery) have never run against a real or simulated autopilot.
- **Payload method.** The design report says the gripper opens at 5 m; the rulebook figure says the payload is lowered to the ground on a pulley. We follow the report. This may need a Change Request.
- **Geofence:** the real coordinates arrive in Phase 2 of the competition; the Python limits are placeholders.

---

## Conventions that must not be broken

- Velocity commands use `MAV_FRAME_BODY_OFFSET_NED` through `Vehicle.send_body_velocity(vx_forward, vy_right, vz_down)`. **vz is NED: negative means climb.** Yaw rate is in deg/s, positive = clockwise.
- Camera frames are RGB `uint8`, `H x W x 3`. Image +x is right, +y is down. `forward_m > 0` is ahead, `right_m > 0` is to the right.
- Camera pitch is the angle of the optical axis below the horizon (90 = straight down, 0 = straight forward).
- Handlers are non-blocking `handler(ctx) -> Optional[State]`, called every tick (`CONTROL_HZ` = 10). Time and sleeping are injected, so tests run in simulated time.
- Never add a number to the code that belongs in `config/params.py`.