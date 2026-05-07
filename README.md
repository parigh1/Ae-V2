# AeroTHON 2026 — Autonomous UAS System
### SAEINDIA Rotorcraft Systems Challenge | Track 1 | Team Entry

> **Theme:** Aerial Surveillance and Rapid Delivery  
> **Competition:** SAEINDIA AeroTHON 2026 — Design, Build and Fly Contest  
> **Category:** Micro UAS (MTOW < 2 kg) | Multirotor | Autonomous + Manual

---

## Table of Contents

- [Overview](#overview)
- [Mission Profiles](#mission-profiles)
- [System Architecture](#system-architecture)
- [Software Stack](#software-stack)
- [Algorithms](#algorithms)
- [Hardware Stack](#hardware-stack)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Running the Code](#running-the-code)
- [Testing Without a Drone](#testing-without-a-drone)
- [Configuration](#configuration)
- [Flight Safety](#flight-safety)
- [Development Roadmap](#development-roadmap)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This repository contains the complete software stack for our AeroTHON 2026 competition entry. The system enables a sub-2 kg multirotor UAS to autonomously complete a rapid delivery mission involving QR code scanning, corridor navigation, precision payload delivery via a controlled pulley mechanism, and autonomous return — all within a 15-minute flight window.

The codebase is structured around a **finite state machine** running on a Raspberry Pi 4 companion computer, communicating with an ArduPilot-based flight controller over MAVLink. Computer vision, navigation control, payload actuation, and safety monitoring each run as independent, testable modules.

**Key capabilities:**
- Autonomous takeoff, waypoint navigation, and precision landing
- Real-time QR code detection and decoding at 5m and 10m altitude using multi-library fallback pipeline
- Autonomous corridor navigation with dual LiDAR wall-centering PID
- Delivery zone search using a lawnmower pattern with live image-based visual servoing
- Controlled winch-and-release payload delivery mechanism
- Full failsafe suite: RTL on low battery, RTL on datalink loss, geofence enforcement

---

## Mission Profiles

### Mission 1 — Eyes in the Sky (Manual)

The pilot manually navigates the UAS through a structured obstacle course — hurdles, hoops, open tunnels, double gates, and a chicane — while avoiding marked red/restricted zones. Mid-course, the drone hovers at designated observation zones and the pilot verbally reports identified objects to the jury. The mission concludes with precision payload placement at a ground target, followed by a return to the launch point.

| Parameter | Value |
|-----------|-------|
| Operation mode | Fully manual |
| Time limit | 15 minutes |
| Payload mass | 100 g |
| Payload dimensions | 10 × 5 × 5 cm |

### Mission 2 — SkyScan (Autonomous)

The UAS operates with zero manual intervention from takeoff to landing. The full sequence:

```
Autonomous takeoff
    ↓
Ascend to 5m → scan start QR code → decode delivery location ID
    ↓
Detect AeroTHON 2026 green banner → align with corridor entrance
    ↓
Descend to 3m → navigate 10m corridor (3.5m wide, with static obstacles)
    ↓
Enter delivery zone → ascend to 10m
    ↓
Lawnmower search → identify matching target QR among multiple codes
    ↓
Avoid red zones → visual servo to center over target QR
    ↓
Descend to 5m → lower payload via controlled pulley → release at ground
    ↓
Ascend to 10m → detect green banner (return lap)
    ↓
Navigate corridor in reverse → return to takeoff point → land
```

| Parameter | Value |
|-----------|-------|
| Operation mode | Fully autonomous |
| Time limit | 15 minutes |
| Corridor width | 3.5 m |
| Corridor altitude | ~3 m (10 feet) |
| QR scan altitude | 5 m (start), 10 m (target identification) |
| Payload drop altitude | 5 m |
| Delivery zone area | ~30 m × 40 m |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  COMPANION COMPUTER                      │
│                   Raspberry Pi 4B                        │
│                                                          │
│  ┌──────────────────────────────────────────────────┐   │
│  │           Mission State Machine                   │   │
│  │     (Central autonomous execution loop)           │   │
│  └──────────┬─────────────┬──────────────┬──────────┘   │
│             │             │              │               │
│  ┌──────────▼──┐  ┌───────▼──────┐  ┌───▼──────────┐   │
│  │   Vision    │  │  Navigation  │  │   Payload    │   │
│  │   Module    │  │   Module     │  │   Module     │   │
│  │             │  │              │  │              │   │
│  │ QR detect   │  │ Altitude PID │  │ Winch ctrl   │   │
│  │ Banner det. │  │ Corridor nav │  │ Servo release│   │
│  │ Red zone    │  │ Visual servo │  │              │   │
│  │ detection   │  │ GPS waypoint │  │              │   │
│  └──────────┬──┘  └───────┬──────┘  └───┬──────────┘   │
│             └─────────────┼─────────────┘               │
│                           │                             │
│  ┌────────────────────────▼────────────────────────┐   │
│  │              Safety Manager                      │   │
│  │   (Background thread — battery, failsafe,        │   │
│  │    geofence monitoring)                          │   │
│  └────────────────────────┬────────────────────────┘   │
│                           │                             │
│                    MAVLink / DroneKit                   │
│                     921600 baud UART                    │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────┐
│              FLIGHT CONTROLLER                           │
│         Pixhawk / Cube Orange + ArduPilot                │
│   PID loops · Attitude estimation · ESC · RTL failsafe   │
└─────────────────────────────────────────────────────────┘
```

---

## Software Stack

| Layer | Technology |
|-------|-----------|
| Flight firmware | ArduPilot Copter |
| Ground station | Mission Planner |
| Companion OS | Raspberry Pi OS (64-bit) |
| Companion language | Python 3.11 |
| MAVLink bridge | DroneKit-Python |
| Computer vision | OpenCV 4.x + pyzbar + picamera2 |
| QR decoding | pyzbar (ZBar) primary, OpenCV QRCodeDetector fallback |
| LiDAR interface | Serial UART (TF-Luna) |
| Simulation | ArduPilot SITL + Gazebo |

---

## Algorithms

### QR Code Detection Pipeline

Every captured frame passes through a layered preprocessing and multi-library decode stack:

```
Raw frame
  → CLAHE (contrast normalization for outdoor lighting)
  → Adaptive threshold (handles partial shadows)
  → Sharpening kernel (counters motion blur)
  → Morphological close (fills sensor noise gaps)
  → pyzbar on all variants (fast path)
  → OpenCV QRCodeDetector fallback (handles perspective distortion)
  → Validation layer
  → Decoded data + pixel center coordinates
```

**Task A — Start QR (5 m altitude):** Single QR on ground, decode delivery location ID. Up to 15 s window.

**Task B — Target QR (10 m altitude):** Multiple QR codes in delivery zone. Match decoded content against stored delivery ID, then compute real-world offset for visual servoing.

**Pixel-to-metres conversion:**
```
ground_width = 2 × altitude × tan(FOV_H / 2)
m_per_pixel  = ground_width / image_width
offset_m     = offset_px × m_per_pixel
```

### Altitude Control

Fused rangefinder + barometer PID controller. Rangefinder (TF-Luna) is trusted below 8 m; barometer takes over above.

```
error     = target_altitude - current_altitude
velocity  = Kp×error + Ki×∫error + Kd×(Δerror/Δt)
velocity  = clamp(velocity, -1.5, +1.5)  # m/s
```

Altitude setpoints for Mission 2: `5m → 3m → 10m → 5m → 10m → land`

### Corridor Navigation

Dual lateral TF-Luna LiDAR sensors (left + right) maintain centerline position inside the 3.5 m corridor. A third forward-facing sensor provides obstacle detection.

```
error_lateral = right_distance - left_distance
vy_correction = Kp×error + Ki×∫error + Kd×(Δerror/Δt)
send_velocity(vx=forward_speed, vy=vy_correction, vz=0)
```

### Delivery Zone Search

Systematic lawnmower pattern traverses the 30 × 40 m delivery zone. The vision module scans every frame during movement — no stop-and-hover required. When the matching QR is found mid-traverse, the search halts and visual servoing begins.

### Visual Servo (Image-Based)

Once the target QR is located, a PD controller drives the drone until the QR is within ±25 cm of the frame center:

```
vx = Kp×dy_m + Kd×(d_dy/dt)
vy = Kp×dx_m + Kd×(d_dx/dt)
```

### Green Banner Detection

HSV color segmentation isolates the AeroTHON 2026 green banner on a forward-facing camera. Yaw rate correction aligns the drone heading before corridor entry, on both the forward and return laps.

### Payload Delivery

Controlled winch motor lowers the 100 g payload at ~0.4 m/s from 5 m altitude. Line length slightly exceeds 5 m to guarantee ground contact before release. Gripper servo opens on confirmation. Winch retracts before return flight.

---

## Hardware Stack

| Component | Model | Purpose |
|-----------|-------|---------|
| Flight controller | Cube Orange / Pixhawk 6C | ArduPilot, PID, ESC |
| Companion computer | Raspberry Pi 4B (4 GB) | Vision, navigation, ML |
| Downward camera | Pi Camera v3 Wide (102° FOV) | QR detection |
| Forward camera | Pi Camera v2 / USB webcam | Banner + red zone detection |
| Lateral LiDAR ×2 | TF-Luna (UART) | Corridor wall centering |
| Forward LiDAR | TF-Luna (UART) | Obstacle detection |
| Downward rangefinder | TF-Luna (UART) | Precision altitude hold |
| Telemetry | RFD900x / SiK radio | GCS link + RTL trigger |
| Frame | Custom carbon fibre quadrotor | <2 kg MTOW |
| Battery | 4S LiPo, capacity TBD | Endurance ≥15 min |
| Payload mechanism | Brushed DC winch + servo gripper | Controlled pulley delivery |

---

## Repository Structure

```
aerothon-2026/
│
├── mission/
│   ├── state_machine.py        # Central mission execution loop
│   ├── states.py               # MissionState enum definitions
│   └── handlers.py             # Per-state handler functions
│
├── vision/
│   ├── qr_scanner.py           # pyzbar + OpenCV decode pipeline
│   ├── preprocessing.py        # CLAHE, threshold, sharpen stack
│   ├── banner_detector.py      # Green banner HSV segmentation
│   ├── red_zone_detector.py    # Red zone detection + repulsion
│   └── pixel_to_meters.py      # FOV-based offset conversion
│
├── navigation/
│   ├── altitude_control.py     # Rangefinder-fused PID altitude hold
│   ├── corridor_nav.py         # Dual LiDAR wall-centering PID
│   ├── visual_servo.py         # Image-based visual servoing
│   ├── search_pattern.py       # Lawnmower delivery zone search
│   └── gps_nav.py              # Waypoint navigation + haversine
│
├── payload/
│   └── winch_controller.py     # Winch motor + gripper servo control
│
├── safety/
│   └── safety_manager.py       # Battery monitor, failsafe verification
│
├── tests/
│   ├── qr_test.py              # Standalone webcam QR test (no drone needed)
│   ├── altitude_test.py        # SITL altitude hold test
│   ├── corridor_sim.py         # Simulated corridor with mock LiDAR values
│   └── banner_test.py          # Webcam banner detection test
│
├── config/
│   └── params.py               # All tunable constants in one place
│
├── docs/
│   ├── wiring_diagram.pdf
│   ├── flight_test_log.md
│   └── sitl_setup.md
│
├── requirements.txt
└── README.md
```

---

## Installation

### Prerequisites

- Python 3.10 or 3.11
- Git
- On Raspberry Pi: Raspberry Pi OS 64-bit (Bookworm)
- On Windows/macOS/Linux: any OS for simulation and vision testing

### Clone the repository

```bash
git clone https://github.com/your-org/aerothon-2026.git
cd aerothon-2026
```

### Create a virtual environment

```bash
python -m venv .venv

# Windows
.venv\Scripts\activate

# Linux / macOS / Raspberry Pi
source .venv/bin/activate
```

### Install dependencies

```bash
pip install -r requirements.txt
```

**requirements.txt:**
```
dronekit==2.9.2
dronekit-sitl==3.3.0
pymavlink==2.4.41
opencv-python==4.9.0.80
pyzbar==0.1.9
numpy==1.26.4
pyserial==3.5
RPi.GPIO==0.7.1         # Raspberry Pi only
picamera2==0.3.19       # Raspberry Pi only
```

### Linux system dependencies (Raspberry Pi / Ubuntu)

```bash
# ZBar library (required by pyzbar)
sudo apt-get install libzbar0

# Camera support
sudo apt-get install python3-picamera2

# LiDAR serial access
sudo usermod -aG dialout $USER
```

### Windows system dependencies

```bash
# pyzbar on Windows requires the ZBar DLL
# Download libzbar-64.dll and place it in the project root
# or install via conda:
conda install -c conda-forge zbar
```

---

## Running the Code

### 1. QR scanner test (any computer, no drone required)

This validates the full vision pipeline using your laptop webcam. Point any QR code at the camera.

```bash
python tests/qr_test.py
```

Expected output:
```
[OK] pyzbar imported
[OK] OpenCV version 4.9.0
[OK] Camera opened at index 0
     Camera native resolution: 640x480
[STEP 2] Live QR scan running

==================================================
[QR DECODED]
  Data         : 'TARGET_A3'
  Method       : pyzbar/original
  Frame #      : 47
  Offset metres: (+0.182m, -0.064m)
  Drone action : move right 0.18m, back 0.06m
==================================================
```

### 2. ArduPilot SITL simulation

Install SITL on Ubuntu/WSL:
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter
```

Launch SITL:
```bash
cd ArduPlane
sim_vehicle.py -v ArduCopter --console --map
```

Run the mission against SITL:
```bash
python mission/state_machine.py --connect tcp:127.0.0.1:5760 --sim
```

### 3. Full autonomous mission (real drone)

```bash
# Connect via telemetry radio or direct USB
python mission/state_machine.py --connect /dev/ttyUSB0 --baud 57600
```

> **Safety requirement:** Always arm the drone with props removed for first connection tests. Verify all failsafes pass the startup check before any outdoor flight.

---

## Testing Without a Drone

Every module can be validated independently before real hardware integration.

| Test | What it validates | Hardware needed |
|------|-------------------|-----------------|
| `tests/qr_test.py` | Full QR pipeline, pixel→metres math | Webcam only |
| `tests/banner_test.py` | Green banner HSV detection, yaw correction output | Webcam only |
| `tests/altitude_test.py` | Altitude PID with simulated rangefinder | SITL |
| `tests/corridor_sim.py` | Corridor centering PID with mock LiDAR values | None |

**Simulated altitude testing without a drone:**

Hold your webcam on a tripod at 1 m above a 12 cm printed QR code. This replicates the same angular size as a 60 cm QR code at 5 m altitude, allowing you to validate decode rate and pixel offset accuracy before any flight.

---

## Configuration

All tunable constants live in `config/params.py`:

```python
# ── Altitude setpoints ────────────────────────────────────────────
ALT_QR_SCAN      = 5.0    # m — start QR scan
ALT_CORRIDOR     = 3.0    # m — corridor navigation (~10 feet)
ALT_DELIVERY     = 10.0   # m — delivery zone QR identification
ALT_PAYLOAD_DROP = 5.0    # m — payload lowering

# ── Altitude PID ──────────────────────────────────────────────────
ALT_KP = 0.6
ALT_KI = 0.008
ALT_KD = 0.12
ALT_TOLERANCE    = 0.15   # m — within this = altitude reached

# ── Corridor navigation ───────────────────────────────────────────
CORRIDOR_WIDTH        = 3.5    # m
CORRIDOR_FWD_SPEED    = 0.4    # m/s
CORRIDOR_LAT_KP       = 0.25
CORRIDOR_LAT_KI       = 0.003
CORRIDOR_LAT_KD       = 0.04
CORRIDOR_OBSTACLE_MIN = 0.7    # m — stop if forward LiDAR < this

# ── Visual servo ─────────────────────────────────────────────────
SERVO_KP          = 0.35
SERVO_KD          = 0.08
SERVO_TOLERANCE   = 0.25  # m — centering complete threshold
SERVO_MAX_SPEED   = 0.30  # m/s

# ── Lawnmower search ─────────────────────────────────────────────
SEARCH_ZONE_W     = 30.0  # m
SEARCH_ZONE_H     = 40.0  # m
SEARCH_STRIP_W    = 4.0   # m — gap between lawnmower passes
SEARCH_SPEED      = 0.5   # m/s

# ── Camera (Pi Camera v3 Wide) ────────────────────────────────────
CAM_FOV_H_DEG     = 84.0
CAM_FOV_V_DEG     = 64.0
CAM_IMG_W         = 1280
CAM_IMG_H         = 720

# ── Payload winch ─────────────────────────────────────────────────
WINCH_CHANNEL     = 9
RELEASE_CHANNEL   = 10
WINCH_LINE_LENGTH = 5.5    # m — slightly longer than drop altitude
WINCH_LOWER_SPEED = 0.4    # m/s

# ── Safety ───────────────────────────────────────────────────────
BATTERY_RTL_PCT   = 20     # % — RTL below this
VOLTAGE_RTL_V     = 14.0   # V — RTL below this (4S: 3.5V/cell)

# ── MAVLink failsafe params (verified at startup) ─────────────────
REQUIRED_PARAMS = {
    'FS_THR_ENABLE':  1,   # throttle loss → RTL
    'FS_GCS_ENABLE':  1,   # datalink loss → RTL
    'FS_BATT_ENABLE': 2,   # battery → RTL
    'FENCE_ENABLE':   1,   # geofence active
    'FENCE_ACTION':   1,   # breach → RTL
}
```

---

## Flight Safety

### Pre-flight checklist (run before every flight)

- [ ] Battery voltage above 15.8 V (4S full charge)
- [ ] Startup safety check prints all `✓` — no `✗ MISMATCH`
- [ ] Geofence coordinates loaded in Mission Planner
- [ ] RTL altitude set to 15 m minimum
- [ ] Props balanced and securely fastened
- [ ] Payload attached and winch line not tangled
- [ ] LiDAR sensors not obstructed
- [ ] Camera lens clean and unobstructed
- [ ] Ground station telemetry link confirmed
- [ ] Pilot has manual override ready on RC transmitter at all times

### Emergency procedures

| Condition | Automatic action | Manual override |
|-----------|-----------------|-----------------|
| Battery < 20% | RTL | Switch to LOITER, land manually |
| Datalink loss | RTL | N/A (link is lost) |
| Geofence breach | RTL | Switch to STABILIZE |
| Mission state timeout | EMERGENCY → RTL | Kill switch on transmitter |
| Obstacle < 0.7 m ahead | Hover and wait | RTL or manual land |

### ArduPilot parameter requirements

These must be set via Mission Planner before any autonomous flight:

```
FS_THR_ENABLE  = 1    (throttle failsafe → RTL)
FS_GCS_ENABLE  = 1    (GCS failsafe → RTL)
FS_BATT_ENABLE = 2    (battery failsafe → RTL)
FENCE_ENABLE   = 1    (geofence active)
FENCE_ACTION   = 1    (geofence breach → RTL)
RTL_ALT        = 1500 (RTL altitude 15m in cm)
WPNAV_SPEED    = 300  (max waypoint speed 3 m/s)
```

---

## Development Roadmap

### Phase 1 — Software foundation ✅
- [x] Mission state machine with all state transitions
- [x] QR detection pipeline (pyzbar + OpenCV fallback)
- [x] Preprocessing stack (CLAHE + adaptive threshold + sharpen)
- [x] Pixel-to-metres conversion with FOV math
- [x] Altitude PID controller
- [x] SITL integration and first test flights
- [x] Standalone webcam QR test tool

### Phase 2 — Navigation modules 🔄
- [ ] Corridor navigation with dual LiDAR wall centering
- [ ] Lawnmower search pattern
- [ ] Image-based visual servo centering
- [ ] Green banner detection and yaw alignment
- [ ] Red zone detection and avoidance

### Phase 3 — Hardware integration 📋
- [ ] TF-Luna LiDAR UART interface on Raspberry Pi
- [ ] Pi Camera v3 Wide integration (replace webcam)
- [ ] Dual camera setup (downward + forward)
- [ ] Winch motor PWM control via servo channel
- [ ] End-to-end bench test (motors off, props off)

### Phase 4 — Flight testing 📋
- [ ] Hover and altitude hold test
- [ ] QR decode at 5 m altitude on printed codes
- [ ] Corridor traverse at 3 m
- [ ] Delivery zone search and target QR identification
- [ ] Full Mission 2 rehearsal end-to-end
- [ ] Timing optimization (target < 12 min)

---

## Contributing

This is a competition project developed by the team. If you are part of the team:

1. Create a branch from `main` named `feature/your-module`
2. Write and test your module independently before merging
3. All new functions must have a corresponding test in `tests/`
4. Never commit with broken imports or untested hardware calls
5. Keep `config/params.py` as the single source of truth for all constants — no magic numbers anywhere else in the codebase

---

## Acknowledgements

- [ArduPilot](https://ardupilot.org/) — open-source flight firmware
- [DroneKit-Python](https://dronekit.io/) — MAVLink Python SDK
- [ZBar](http://zbar.sourceforge.net/) — QR and barcode scanning library
- [OpenCV](https://opencv.org/) — computer vision framework
- [SAEINDIA](https://saeindia.org/) — AeroTHON 2026 organising committee

---

## License

This project is developed for the SAEINDIA AeroTHON 2026 competition. All code is original work by the team members. See `LICENSE` for terms.

---

<div align="center">

**Built for AeroTHON 2026 — SAEINDIA Aerospace Forum**

*Autonomous · Precise · Safe*

</div>
