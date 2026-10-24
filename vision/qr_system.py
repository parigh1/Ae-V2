# =============================================================================
# vision/qr_system.py — Team Vajra AeroTHON 2026
#
# Mission-level QR logic. DETECTION ONLY — it never moves the drone.
# Centering over the target is navigation/visual_servo.py.
#
# Phase 8 update: the organizers confirmed the QR codes carry NO decodable text.
# They are visual patterns only. The primary identification path is now:
#
#   scan_start_qr()  → captures and confirms the start-QR pattern
#   find_target()    → finds the delivery-zone QR whose pattern matches
#
# Text decoding (pyzbar / OpenCV) is kept as a secondary path for simulation
# and testing, where the codes happen to carry text. The pattern matcher always
# runs and is authoritative on the real field.
# =============================================================================

import time
from dataclasses import dataclass
from typing import Optional

from config.params import QR_SCAN_TIMEOUT_A, CAM_FPS, QR_PATTERN_CONFIRM_MIN
from vision.pixel_to_meters import PixelToMeters
from vision.qr_scanner import RobustQRScanner, QRConfirmer
from vision.qr_pattern import QRTemplate, QRMatcher, locate_all_qrs


@dataclass
class TargetFix:
    """Where the matching QR is, relative to the drone (body frame)."""
    forward_m: float      # + = target is ahead of the drone
    right_m: float        # + = target is to the right
    dx_px: float          # pixel offset from image centre (+ right)
    dy_px: float          # pixel offset from image centre (+ down)
    altitude_m: float     # height used for the pixel→metre scale
    method: str           # "pattern" | "text" | "text+pattern"


class QRSystem:
    def __init__(self, camera, vehicle, sleep=time.sleep, clock=time.monotonic):
        self.camera = camera
        self.vehicle = vehicle
        self.pitch_deg = getattr(camera, "pitch_deg", 90.0)
        self.scanner = RobustQRScanner()
        self.delivery_id: Optional[str] = None
        self._sleep = sleep
        self._clock = clock
        self._tmpl = QRTemplate()
        self._matcher = QRMatcher()

    def _height_agl(self) -> float:
        rf = self.vehicle.rangefinder_distance
        return rf if rf is not None else self.vehicle.altitude

    def scan_step(self, confirmer: QRConfirmer) -> Optional[str]:
        """ONE frame of the start-QR scan (the state machine calls this every tick).
        Returns a delivery ID token once the pattern is confirmed, else None.
        Works even when the code carries no decodable text (returns 'QR_PATTERN'
        as the ID in that case, since the pattern template is what matters)."""
        frame = self.camera.capture_array()
        w, h = frame.shape[1], frame.shape[0]

        # Try text decoding first (works in simulation)
        text = self.scanner.decode_frame(frame)
        confirmed_text = confirmer.update(text)

        # Always update the pattern template
        dets = locate_all_qrs(frame)
        if dets:
            self._tmpl.add(dets[0]["patch"])
        if self._tmpl.confirmed and not self._matcher.has_template():
            self._matcher.update_template(self._tmpl)

        if confirmed_text:
            self.delivery_id = confirmed_text
            return confirmed_text
        if self._tmpl.confirmed:
            if self.delivery_id is None:
                self.delivery_id = "QR_PATTERN"
            return self.delivery_id
        return None

    def scan_start_qr(self, timeout: float = QR_SCAN_TIMEOUT_A) -> Optional[str]:
        """Blocking scan. Returns the delivery ID or None on timeout."""
        confirmer = QRConfirmer()
        deadline = self._clock() + timeout
        frames = 0
        while self._clock() < deadline:
            result = self.scan_step(confirmer)
            frames += 1
            if result:
                print(f"[QR] Start QR confirmed after {frames} frames: '{result}'")
                return result
            if frames % 30 == 0:
                print(f"[QR] Still scanning... {frames} frames")
            self._sleep(1.0 / CAM_FPS)
        print(f"[QR] FAILED: no confirmed start QR in {timeout}s ({frames} frames)")
        return None

    def find_target(self, frame=None) -> Optional[TargetFix]:
        """Analyse one frame. Returns a TargetFix if the matching QR is visible."""
        if self.delivery_id is None:
            raise RuntimeError("scan_start_qr() must succeed before find_target()")

        if frame is None:
            frame = self.camera.capture_array()
        h, w = frame.shape[:2]
        alt = self._height_agl()

        # Path 1: pattern matching (primary; works without text)
        if self._matcher.has_template():
            res = self._matcher.find_best(frame, alt, self.pitch_deg, w, h)
            if res is not None:
                dx_px = res["cx"] - w / 2
                dy_px = res["cy"] - h / 2
                method = "text+pattern" if res["data"] else "pattern"
                return TargetFix(res["forward_m"], res["right_m"],
                                 dx_px, dy_px, alt, method)

        # Path 2: text matching fallback (simulation / testing)
        if self.delivery_id != "QR_PATTERN":
            for det in self.scanner.decode_all(frame, want=self.delivery_id):
                if det.data != self.delivery_id:
                    continue
                dx_px = det.cx - w / 2
                dy_px = det.cy - h / 2
                ground = PixelToMeters.offset_body(
                    dx_px, dy_px, alt, w, h, pitch_deg=self.pitch_deg)
                if ground is None:
                    continue
                fwd, right = ground
                return TargetFix(fwd, right, dx_px, dy_px, alt, "text")
        return None