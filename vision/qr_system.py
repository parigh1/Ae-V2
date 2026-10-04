# =============================================================================
# vision/qr_system.py — Team Vajra AeroTHON 2026
#
# Mission-level QR logic. DETECTION ONLY — it never moves the drone.
# Centering over the target is navigation/visual_servo.py.
#
# CHANGES vs previous version
#   [FIX] missing `pyzbar_decode` import (crashed on first target scan).
#   [FIX] no raw DroneKit access (vehicle.rangefinder / vehicle.location);
#         uses the Vehicle wrapper (rangefinder_distance, altitude).
#   [FIX] removed self.nav._send_ned_velocity (did not exist).
#   [MOVED] center_over_target() -> navigation/visual_servo.py (gains come
#           from params, body-frame commands, sign handled in PixelToMeters).
#   [NEW] scan_start_qr() requires QR_CONFIRM_COUNT matching reads in a row.
#   [NEW] find_target() returns a TargetFix: offset in the BODY frame
#         (forward_m, right_m) — directly usable by VisualServo.
#   `camera` must provide capture_array() -> RGB uint8 frame, and may provide
#   `.pitch_deg` (see hardware/camera.py: rig.view("down")).
#   [NEW] tilted-camera support (pitch_deg) via PixelToMeters.
# =============================================================================

import time
from dataclasses import dataclass
from typing import Optional

from config.params import QR_SCAN_TIMEOUT_A, QR_CONFIRM_COUNT, CAM_FPS
from vision.pixel_to_meters import PixelToMeters
from vision.qr_scanner import RobustQRScanner, QRConfirmer


@dataclass
class TargetFix:
    """Where the matching QR is, relative to the drone (body frame)."""
    forward_m: float      # + = target is ahead of the drone
    right_m: float        # + = target is to the right
    dx_px: float          # pixel offset from image centre (+ right)
    dy_px: float          # pixel offset from image centre (+ down)
    altitude_m: float     # height used for the pixel→metre scale
    method: str


class QRSystem:
    def __init__(self, camera, vehicle, sleep=time.sleep, clock=time.monotonic):
        self.camera = camera
        self.vehicle = vehicle
        # How the camera is tilted (90 = straight down). A CameraView from
        # hardware/camera.py provides this; a bare test camera defaults to 90.
        self.pitch_deg = getattr(camera, "pitch_deg", 90.0)
        self.scanner = RobustQRScanner()
        self.delivery_id: Optional[str] = None
        self._sleep = sleep
        self._clock = clock

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _height_agl(self) -> float:
        """Height above ground for pixel scaling: rangefinder if valid, else baro."""
        rf = self.vehicle.rangefinder_distance
        return rf if rf is not None else self.vehicle.altitude

    # ── Task A: scan the start QR (drone hovering at ALT_START_QR) ───────────
    def scan_start_qr(self, timeout: float = QR_SCAN_TIMEOUT_A) -> Optional[str]:
        """
        Returns the delivery ID once it has been read QR_CONFIRM_COUNT times
        in a row, or None on timeout.
        """
        confirmer = QRConfirmer(QR_CONFIRM_COUNT)
        deadline = self._clock() + timeout
        period = 1.0 / CAM_FPS
        frames = 0

        while self._clock() < deadline:
            frame = self.camera.capture_array()
            frames += 1
            confirmed = confirmer.update(self.scanner.decode_frame(frame))
            if confirmed:
                print(f"[QR] Start QR confirmed after {frames} frames: '{confirmed}'")
                self.delivery_id = confirmed
                return confirmed
            if frames % 30 == 0:
                print(f"[QR] Still scanning... {frames} frames")
            self._sleep(period)

        print(f"[QR] FAILED: no confirmed start QR in {timeout}s ({frames} frames)")
        self.scanner.stats()
        return None

    # ── Task B: look for the matching QR in one frame ─────────────────────────
    def find_target(self, frame=None) -> Optional[TargetFix]:
        """
        Analyse one frame (grabs one if not supplied). Returns a TargetFix if a
        QR whose payload equals delivery_id is visible, else None.
        Cheap to call every loop: stops decoding as soon as the target is seen.
        """
        if self.delivery_id is None:
            raise RuntimeError("scan_start_qr() must succeed before find_target()")

        if frame is None:
            frame = self.camera.capture_array()
        h, w = frame.shape[:2]

        for det in self.scanner.decode_all(frame, want=self.delivery_id):
            if det.data != self.delivery_id:
                continue
            dx_px = det.cx - w / 2.0
            dy_px = det.cy - h / 2.0
            alt = self._height_agl()
            ground = PixelToMeters.offset_body(dx_px, dy_px, alt, w, h,
                                               pitch_deg=self.pitch_deg)
            if ground is None:          # looking at/above the horizon: not on the ground
                continue
            fwd, right = ground
            return TargetFix(fwd, right, dx_px, dy_px, alt, det.method)
        return None
