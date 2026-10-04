# =============================================================================
# hardware/camera.py — Team Vajra AeroTHON 2026
#
# One place that knows HOW the camera(s) are fitted. The rest of the code only
# says what it wants to LOOK at:
#
#       rig  = build_camera_rig(vehicle)
#       down = rig.view("down")        # for QR codes / red zones / drop target
#       fwd  = rig.view("forward")     # for the green banner / corridor walls
#       frame = down.capture_array()   # RGB image (numpy array, H x W x 3)
#       down.pitch_deg                 # how that view is tilted (90 = straight down)
#
# Switching hardware layout = change CAMERA_MODE in config/params.py:
#   "single_fixed" : one camera, fixed tilt, BOTH views come from the same camera
#   "dual"         : two cameras (down + forward)
#   "servo"        : one camera, a servo tilts it before each view is captured
# No other file has to change.
#
# All frames returned are RGB order.
# =============================================================================

import time
from typing import Callable, Dict, Optional

import numpy as np

from config.params import (
    CAMERA_MODE, CAM_MOUNTS, CAMERA_BACKEND, CAM_DEVICE_INDEX,
    CAM_IMG_W, CAM_IMG_H,
    CAM_SERVO_CHANNEL, CAM_SERVO_PWM_DOWN, CAM_SERVO_PWM_FORWARD,
    CAM_SERVO_SETTLE_S,
)

VIEWS = ("down", "forward")


# ── Physical camera drivers (all provide capture_array() -> RGB) ──────────────
class WebcamCamera:
    """Laptop / USB webcam via OpenCV."""

    def __init__(self, index: int = 0, width: int = CAM_IMG_W, height: int = CAM_IMG_H):
        import cv2
        self._cv2 = cv2
        self._cap = cv2.VideoCapture(index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"[Camera] Could not open webcam #{index}. Is another program "
                f"using it? Try another number in CAM_DEVICE_INDEX.")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def capture_array(self) -> np.ndarray:
        ok, bgr = self._cap.read()
        if not ok:
            raise RuntimeError("[Camera] Webcam returned no frame")
        return self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)

    def close(self):
        self._cap.release()


class PiCamera:
    """Raspberry Pi Camera (v3 Wide) via picamera2. Only works on the Pi."""

    def __init__(self, width: int = CAM_IMG_W, height: int = CAM_IMG_H):
        from picamera2 import Picamera2        # only exists on the Pi
        self._cam = Picamera2()
        # picamera2 naming is backwards: "BGR888" gives pixels stored as R,G,B.
        # VERIFY ON THE PI with scripts/camera_check.py (hold a RED object:
        # it must look red, not blue).
        cfg = self._cam.create_video_configuration(
            main={"size": (width, height), "format": "BGR888"})
        self._cam.configure(cfg)
        self._cam.start()

    def capture_array(self) -> np.ndarray:
        return self._cam.capture_array()

    def close(self):
        self._cam.stop()


class FakeCamera:
    """For tests / simulation: returns a fixed image, or whatever a function returns."""

    def __init__(self, frame_or_fn):
        self._src = frame_or_fn

    def capture_array(self) -> np.ndarray:
        return self._src() if callable(self._src) else self._src

    def close(self):
        pass


def make_camera(role: str, backend: Optional[str] = None):
    backend = backend or CAMERA_BACKEND[role]
    if backend == "webcam":
        return WebcamCamera(CAM_DEVICE_INDEX[role])
    if backend == "picamera2":
        return PiCamera()
    raise ValueError(f"Unknown camera backend '{backend}' (use 'webcam' or 'picamera2')")


# ── Servo that tilts the camera (servo mode only) ─────────────────────────────
class CameraServo:
    def __init__(self, vehicle, sleep: Callable = time.sleep):
        self.vehicle = vehicle
        self._sleep = sleep

    @staticmethod
    def pwm_for_pitch(pitch_deg: float) -> int:
        """Linear map: 90 deg (down) -> PWM_DOWN, 0 deg (forward) -> PWM_FORWARD."""
        frac = max(0.0, min(1.0, pitch_deg / 90.0))
        return int(round(CAM_SERVO_PWM_FORWARD
                         + (CAM_SERVO_PWM_DOWN - CAM_SERVO_PWM_FORWARD) * frac))

    def point(self, pitch_deg: float):
        self.vehicle.set_servo(CAM_SERVO_CHANNEL, self.pwm_for_pitch(pitch_deg))
        self._sleep(CAM_SERVO_SETTLE_S)


# ── The rig ───────────────────────────────────────────────────────────────────
class CameraView:
    """What the rest of the code holds on to: capture_array() + pitch_deg."""

    def __init__(self, rig: "CameraRig", name: str):
        self._rig = rig
        self.name = name
        self.pitch_deg = rig.pitch_for(name)

    def capture_array(self) -> np.ndarray:
        return self._rig.capture(self.name)


class CameraRig:
    def __init__(self, mode: str, pitches: Dict[str, float], cameras: Dict[str, object],
                 servo: Optional[CameraServo] = None):
        self.mode = mode
        self._pitches = pitches
        self._cameras = cameras
        self._servo = servo
        self._servo_at: Optional[str] = None

    def pitch_for(self, view: str) -> float:
        return self._pitches[view]

    def view(self, name: str) -> CameraView:
        if name not in VIEWS:
            raise ValueError(f"view must be one of {VIEWS}, got '{name}'")
        return CameraView(self, name)

    def capture(self, view: str) -> np.ndarray:
        cam = self._cameras.get(view) or self._cameras["main"]
        if self._servo is not None and self._servo_at != view:
            self._servo.point(self._pitches[view])
            self._servo_at = view
        return cam.capture_array()

    def close(self):
        seen = set()
        for cam in self._cameras.values():
            if id(cam) not in seen:
                seen.add(id(cam))
                cam.close()


def build_camera_rig(vehicle=None, mode: Optional[str] = None,
                     cameras: Optional[Dict[str, object]] = None) -> CameraRig:
    """
    Build the rig described by params.CAMERA_MODE.
    `cameras` lets tests/simulation inject fake cameras instead of opening hardware:
        single_fixed / servo -> {"main": cam}      dual -> {"down": cam, "forward": cam}
    """
    mode = mode or CAMERA_MODE
    if mode not in CAM_MOUNTS:
        raise ValueError(f"CAMERA_MODE must be one of {list(CAM_MOUNTS)}, got '{mode}'")
    pitches = CAM_MOUNTS[mode]

    if mode == "dual":
        cams = cameras or {"down": make_camera("down"), "forward": make_camera("forward")}
        return CameraRig(mode, pitches, cams)

    cams = cameras or {"main": make_camera("main")}
    if mode == "servo":
        if vehicle is None:
            raise ValueError("servo mode needs the vehicle (it drives the servo)")
        return CameraRig(mode, pitches, cams, servo=CameraServo(vehicle))
    return CameraRig(mode, pitches, cams)
