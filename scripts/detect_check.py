# =============================================================================
# scripts/detect_check.py — see what the REAL banner / red-zone detectors do
#
#     python -m scripts.detect_check --detect banner
#     python -m scripts.detect_check --detect red --height 10
#     python -m scripts.detect_check --detect banner --image photo.jpg
#
# Draws a box on what it found and prints: how far left/right the banner is (and
# how fast the drone would turn), or where the nearest red edge is (metres) and
# what the avoidance would do to a forward flight at 1.2 m/s. q = quit.
#   --no-gui   run once on a synthetic picture and exit (self-test)
# =============================================================================
import argparse
import time

import cv2
import numpy as np

from config.params import CAMERA_MODE, SEARCH_SPEED
from navigation.banner_align import yaw_rate_for_offset
from navigation.red_zone_avoidance import adjust_velocity
from vision.banner_detector import BannerDetector
from vision.red_zone import RedZoneDetector


def synthetic(kind):
    img = np.full((720, 1280, 3), 100, np.uint8)
    if kind == "banner":
        img[200:450, 800:1100] = (30, 160, 60)
    else:
        img[250:500, 500:800] = (220, 30, 30)
    return img


def run_once(kind, frame, height, pitch):
    h, w = frame.shape[:2]
    if kind == "banner":
        fix = BannerDetector().detect(frame)
        if fix is None:
            print("  no banner")
            return None
        print(f"  banner at x={fix.cx:.0f}  offset {fix.offset_px:+.0f} px  "
              f"-> turn {yaw_rate_for_offset(fix.offset_px):+.1f} deg/s (+ = clockwise)")
        return ("box", fix.bbox)
    near = RedZoneDetector().nearest(frame, height, pitch)
    blobs = RedZoneDetector().detect(frame)
    if near is None:
        print("  no red zone")
        return None
    vx, vy = adjust_velocity(SEARCH_SPEED, 0.0, near)
    print(f"  nearest red edge: forward {near.forward_m:+.1f} m, right {near.right_m:+.1f} m "
          f"(distance {near.distance_m:.1f} m)  | flying forward at {SEARCH_SPEED} m/s "
          f"would become forward {vx:+.2f}, right {vy:+.2f}")
    return ("box", blobs[0].bbox) if blobs else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detect", choices=["banner", "red"], required=True)
    ap.add_argument("--height", type=float, default=10.0)
    ap.add_argument("--image")
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    rig = None
    pitch = 90.0
    if args.no_gui:
        frame_fn = lambda: synthetic(args.detect)
    elif args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"Could not read image: {args.image}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        frame_fn = lambda: rgb
    else:
        from hardware.camera import build_camera_rig
        rig = build_camera_rig()
        view = rig.view("forward" if args.detect == "banner" else "down")
        pitch = view.pitch_deg
        frame_fn = view.capture_array
    print(f"[detect] mode={CAMERA_MODE} detect={args.detect} pitch={pitch} height={args.height} m")

    while True:
        frame = frame_fn()
        shown_box = run_once(args.detect, frame, args.height, pitch)
        if args.no_gui:
            print("[detect] self-test done")
            return
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if shown_box:
            x, y, bw, bh = shown_box[1]
            cv2.rectangle(bgr, (x, y), (x + bw, y + bh), (0, 255, 0), 3)
        h, w = bgr.shape[:2]
        cv2.drawMarker(bgr, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.imshow("detect_check  (q = quit)", cv2.resize(bgr, (960, 540)))
        if cv2.waitKey(1) & 0xFF == ord("q") or args.image:
            if args.image:
                cv2.waitKey(0)
            break
        time.sleep(0.1)
    if rig:
        rig.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
