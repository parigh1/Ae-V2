import time

import cv2
import numpy as np

from pixtopix import PixelToMeters
from pyzbr import RobustQRScanner
from qrpipe import preprocess_for_qr


class QRSystem:
    def __init__(self, camera, vehicle, nav_module):
        self.camera = camera
        self.vehicle = vehicle
        self.nav = nav_module
        self.scanner = RobustQRScanner()
        self.p2m = PixelToMeters()
        self.delivery_id = None

    # ─────── Task A: scan the start QR at 5m ───────
    def scan_start_qr(self, timeout=15.0):
        """
        Called from state SCAN_START_QR.
        Drone is hovering at 5m. Tries every 100ms.
        Returns delivery_id string, or None on timeout.
        """
        deadline = time.time() + timeout
        attempt = 0

        while time.time() < deadline:
            frame = self.camera.capture_array()
            result = self.scanner.decode_frame(frame)
            attempt += 1

            if result:
                print(f"[QR] Start QR decoded on attempt {attempt}: '{result}'")
                self.delivery_id = result
                return result

            # Every 3 seconds of failure, log the attempt count
            # so you know during testing how long it's taking
            if attempt % 30 == 0:
                elapsed = timeout - (deadline - time.time())
                print(f"[QR] Still scanning... {elapsed:.0f}s elapsed, {attempt} frames")

            time.sleep(0.1)  # 10fps scan rate

        print(f"[QR] FAILED after {attempt} attempts in {timeout}s")
        self.scanner.stats()
        return None

    # ─────── Task B: find target QR during lawnmower search ───────
    def scan_for_target(self):
        """
        Called continuously during lawnmower search pattern.
        Returns (dx_m, dy_m, True) if matching QR found and centered.
        Returns (0, 0, False) if no match in current frame.

        dx_m, dy_m = displacement from frame center in meters.
        Positive dx_m = QR is to the right, drone must move right.
        Positive dy_m = QR is below center, drone must move forward.
        """
        if self.delivery_id is None:
            raise RuntimeError("scan_start_qr() must succeed before scan_for_target()")

        frame = self.camera.capture_array()
        variants = preprocess_for_qr(frame)
        alt = self.vehicle.rangefinder.distance or \
              self.vehicle.location.global_relative_frame.alt

        img_cx = PixelToMeters.IMG_W // 2
        img_cy = PixelToMeters.IMG_H // 2

        # Try pyzbar first (returns all QRs visible in frame at once)
        for img in variants:
            results = pyzbar_decode(img)
            for obj in results:
                data = obj.data.decode('utf-8').strip()
                if data == self.delivery_id:
                    # Found the match — compute pixel offset from center
                    r = obj.rect
                    qr_cx = r.left + r.width // 2
                    qr_cy = r.top + r.height // 2
                    dx_px = qr_cx - img_cx
                    dy_px = qr_cy - img_cy
                    dx_m, dy_m = self.p2m.offset_at_altitude(dx_px, dy_px, alt)
                    print(f"[QR] Target match! Pixel offset ({dx_px},{dy_px}) "
                          f"→ Real offset ({dx_m:.2f}m, {dy_m:.2f}m)")
                    return dx_m, dy_m, True

        # pyzbar missed — try OpenCV (returns only one QR)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        data, points, _ = self.scanner.cv_detector.detectAndDecode(frame_gray)
        if data and data.strip() == self.delivery_id and points is not None:
            corners = points[0]
            qr_cx = int(np.mean(corners[:, 0]))
            qr_cy = int(np.mean(corners[:, 1]))
            dx_px = qr_cx - img_cx
            dy_px = qr_cy - img_cy
            dx_m, dy_m = self.p2m.offset_at_altitude(dx_px, dy_px, alt)
            return dx_m, dy_m, True

        return 0.0, 0.0, False

    # ─────── Visual servo: center drone over target QR ───────
    def center_over_target(self, tolerance_m=0.25, timeout=20.0):
        """
        Once target QR is found during search, this centers the drone
        directly over it before descending for the drop.
        Uses a PD controller to prevent oscillation.
        """
        kp = 0.35
        kd = 0.08
        prev_dx, prev_dy = 0.0, 0.0
        dt = 0.1
        deadline = time.time() + timeout

        while time.time() < deadline:
            dx_m, dy_m, found = self.scan_for_target()

            if not found:
                # QR temporarily lost — hover and retry
                self.nav._send_ned_velocity(0, 0, 0)
                time.sleep(0.2)
                continue

            # Check if we're close enough
            if abs(dx_m) < tolerance_m and abs(dy_m) < tolerance_m:
                self.nav._send_ned_velocity(0, 0, 0)
                print(f"[QR] Centered! Residual: ({dx_m:.3f}m, {dy_m:.3f}m)")
                return True

            # PD control — derivative term damps oscillation
            d_dx = (dx_m - prev_dx) / dt
            d_dy = (dy_m - prev_dy) / dt

            # Camera +x right = drone +East = NED vy
            # Camera +y down  = drone +South = NED -vx (check your mounting)
            vy = kp * dx_m + kd * d_dx
            vx = kp * dy_m + kd * d_dy

            # Clamp to safe speed
            vx = max(-0.3, min(0.3, vx))
            vy = max(-0.3, min(0.3, vy))

            self.nav._send_ned_velocity(vx, vy, 0)
            prev_dx, prev_dy = dx_m, dy_m
            time.sleep(dt)

        print("[QR] Centering timed out")
        return False