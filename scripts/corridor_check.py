# scripts/corridor_check.py — see what the corridor detector makes of a camera or a photo
#   python -m scripts.corridor_check --height 3                 (live camera)
#   python -m scripts.corridor_check --height 3 --image hall.jpg
#   python -m scripts.corridor_check --no-gui                   (self-test)
# --height = how high the CAMERA is above the floor, in metres (3 for the real flight).
# Wall lines it trusts are drawn blue (left) and red (right). It prints:
#   centre offset (+ = the middle of the corridor is to your RIGHT)
#   heading error (+ = the corridor points to the right of your nose)
import argparse
import time

import cv2

from config.params import CAMERA_MODE
from navigation.corridor import VisionObstacleSensor
from vision.corridor_detector import CorridorDetector


def report(det, frame, height, pitch):
    fix = det.detect(frame, height, pitch)
    ob = VisionObstacleSensor().measure(frame, height, pitch)
    if fix is None:
        print("  no walls found")
    else:
        print(f"  walls={fix.walls:5s} centre offset {fix.center_offset_m:+.2f} m   "
              f"heading error {fix.heading_error_deg:+.1f} deg   (left {fix.left_m}, right {fix.right_m})")
    if ob is not None:
        print(f"  OBSTACLE {ob.forward_m:.1f} m ahead, nearest edge {ob.right_m:+.1f} m sideways")
    return fix


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--height", type=float, default=3.0)
    ap.add_argument("--image")
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    det, rig, pitch = CorridorDetector(), None, 65.0
    if args.no_gui:
        from tests.corridor_world import CorridorWorldCamera
        from tests.sim_helpers import SimClock, FlightSim
        veh = FlightSim(SimClock())
        veh.altitude, veh.n, veh.e = 3.0, -1.0, 0.5
        report(det, CorridorWorldCamera(veh, pitch).capture_array(), 3.0, pitch)
        print("[corridor] self-test done")
        return
    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"Could not read image: {args.image}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        grab = lambda: rgb
    else:
        from hardware.camera import build_camera_rig
        rig = build_camera_rig()
        view = rig.view("forward")
        pitch, grab = view.pitch_deg, view.capture_array
    print(f"[corridor] mode={CAMERA_MODE} pitch={pitch} deg, camera height {args.height} m")

    while True:
        frame = grab()
        report(det, frame, args.height, pitch)
        shown = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        for seg in det.debug.left:
            cv2.line(shown, (int(seg[0]), int(seg[1])), (int(seg[2]), int(seg[3])), (255, 80, 0), 3)
        for seg in det.debug.right:
            cv2.line(shown, (int(seg[0]), int(seg[1])), (int(seg[2]), int(seg[3])), (0, 0, 255), 3)
        cv2.imshow("corridor_check  (q = quit)", cv2.resize(shown, (960, 540)))
        if args.image:
            cv2.waitKey(0)
            break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(0.1)
    if rig:
        rig.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()