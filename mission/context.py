# =============================================================================
# mission/context.py — Team Vajra AeroTHON 2026
#
# MissionContext = the "backpack" every state handler receives. It holds the
# hardware/software parts and the facts learned during the flight.
# =============================================================================

import time
from typing import Any, Callable, Dict, List, Optional

from config.params import MISSION_TIME_LIMIT_S


class MissionContext:
    def __init__(self, vehicle, rig=None, qr=None, altitude=None, servo=None,
                 safety=None, payload=None, banner=None, corridor=None, obstacle_sensors=None,
                 clock: Callable = time.monotonic,
                 sleep: Callable = time.sleep,
                 recorder=None):
        self.vehicle = vehicle
        self.rig = rig                    # hardware.camera.CameraRig
        self.qr = qr                      # vision.qr_system.QRSystem
        self.altitude = altitude          # navigation.altitude.AltitudeController
        self.servo = servo                # navigation.visual_servo.VisualServo
        self.safety = safety              # safety.watchdog.Watchdog
        self.payload = payload            # payload.release.PayloadReleaser
        self.banner = banner              # vision.banner_detector.BannerDetector
        self.corridor = corridor          # vision.corridor_detector.CorridorDetector
        self.obstacle_sensors = obstacle_sensors  # navigation.corridor.default_sensors()
        self.clock = clock
        self.sleep = sleep
        self.recorder = recorder          # telemetry.recorder.FlightRecorder (optional)

        # facts learned while flying
        self.delivery_id: Optional[str] = None
        self.home = None                  # GPS point where we took off
        self.payload_released = False
        self.data: Dict[str, Any] = {}    # scratch space for handlers

        # timing / bookkeeping (managed by the state machine)
        self.mission_t0: Optional[float] = None    # set when TAKEOFF starts
        self.state_t0: float = clock()
        self.events: List[str] = []
        self.state = None                 # current State (set by the state machine)
        self.sd: Dict[str, Any] = {}      # scratch for the CURRENT state; wiped on every entry

    # ── time ──────────────────────────────────────────────────────────────────
    def state_age(self) -> float:
        return self.clock() - self.state_t0

    def mission_elapsed(self) -> float:
        return 0.0 if self.mission_t0 is None else self.clock() - self.mission_t0

    def mission_time_left(self) -> float:
        return MISSION_TIME_LIMIT_S - self.mission_elapsed()

    # ── logging ───────────────────────────────────────────────────────────────
    def log(self, message: str):
        line = f"[T+{self.mission_elapsed():6.1f}s] {message}"
        self.events.append(line)
        print(line)

        if self.recorder is not None:
            try:
                self.recorder.event(line)
            except Exception:         # noqa: BLE001 - the log must never stop the flight
                pass
