# =============================================================================
# scripts/camera_check.py — beginner camera check (no drone needed)
#
# What it does: opens the camera set up in config/params.py, finds QR codes in
# the picture, and prints where each one is on the ground relative to the drone
# (metres ahead / to the right), using the tilt of the chosen view.
#
# Run from the project root (PyCharm Terminal):
#     python -m scripts.camera_check --height 5
# Hold a QR code (phone screen is fine) in front of the camera. Press  q  to quit.
#
#   --height 5        pretend the camera is 5 m above the ground
#   --view down       "down" or "forward"
#   --no-gui          process one synthetic picture and exit (self-test)
# =============================================================================
import argparse
import time

import cv2
import numpy as np

from config.params import CAMERA_MODE, CAM_IMG_W, CAM_IMG_H
from hardware.camera import build_camera_rig, FakeCamera
from vision.pixel_to_meters import PixelToMeters
from vision.qr_scanner import RobustQRScanner


def synthetic_frame():
    import qrcode
    frame = np.full((CAM_IMG_H, CAM_IMG_W, 3), 255, np.uint8)
    q = np.array(qrcode.make("HELLO_VAJRA", border=2).convert("L").resize((220, 220)))
    frame[250:470, 800:1020] = q[..., None]
    return frame


def analyse(frame, view, height_m, scanner):
    h, w = frame.shape[:2]
    results = []
    for det in scanner.decode_all(frame):
        ground = PixelToMeters.offset_body(det.cx - w / 2, det.cy - h / 2, height_m,
                                           w, h, pitch_deg=view.pitch_deg)
        results.append((det, ground))
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=5.0)
    ap.add_argument("--view", default="down", choices=["down", "forward"])
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    if args.no_gui:
        rig = build_camera_rig(cameras={"main": FakeCamera(synthetic_frame()),
                                        "down": FakeCamera(synthetic_frame()),
                                        "forward": FakeCamera(synthetic_frame())})
    else:
        rig = build_camera_rig()
    view = rig.view(args.view)
    scanner = RobustQRScanner()
    print(f"[check] mode={CAMERA_MODE}  view={args.view}  pitch={view.pitch_deg} deg  "
          f"height={args.height} m")

    while True:
        frame = view.capture_array()
        found = analyse(frame, view, args.height, scanner)
        for det, ground in found:
            if ground is None:
                print(f"  '{det.data}' is above the horizon - cannot place on the ground")
            else:
                print(f"  '{det.data}'  forward {ground[0]:+.2f} m   right {ground[1]:+.2f} m")
        if args.no_gui:
            print("[check] self-test done")
            return
        shown = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        for det, _ in found:
            cv2.circle(shown, (int(det.cx), int(det.cy)), 8, (0, 255, 0), 2)
            cv2.putText(shown, det.data, (int(det.cx) + 10, int(det.cy)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        h, w = shown.shape[:2]
        cv2.drawMarker(shown, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.imshow("camera_check  (q = quit)", shown)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(0.05)
    rig.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
