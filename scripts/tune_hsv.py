# =============================================================================
# scripts/tune_hsv.py — pick the colour numbers for the banner / red zone
#
# Run from the project root (PyCharm terminal):
#     python -m scripts.tune_hsv --target green        # banner
#     python -m scripts.tune_hsv --target red          # red zone
#     python -m scripts.tune_hsv --target green --image photo.jpg   # a saved photo instead
#
# Two windows open: the camera picture and a black/white MASK (white = "counted as
# green/red"). Move the sliders until ONLY your banner / red paper is white and
# everything else is black. Then press  p  to print the numbers, copy them into
# config/params.py, and press  q  to quit.
# Do this again OUTDOORS in real sunlight — indoor numbers will not carry over.
# =============================================================================
import argparse

import cv2
import numpy as np

from config import params
from vision.color_masks import hsv_range_mask


def settings_for(target):
    if target == "green":
        return [("H low", params.BANNER_H_LOW, 179), ("H high", params.BANNER_H_HIGH, 179),
                ("S low", params.BANNER_S_LOW, 255), ("S high", params.BANNER_S_HIGH, 255),
                ("V low", params.BANNER_V_LOW, 255), ("V high", params.BANNER_V_HIGH, 255)]
    return [("H low 1", params.RED_H_LOW1, 179), ("H high 1", params.RED_H_HIGH1, 179),
            ("H low 2", params.RED_H_LOW2, 179), ("H high 2", min(params.RED_H_HIGH2, 179), 179),
            ("S low", params.RED_S_LOW, 255), ("S high", params.RED_S_HIGH, 255),
            ("V low", params.RED_V_LOW, 255), ("V high", params.RED_V_HIGH, 255)]


def mask_from(values, target, rgb):
    if target == "green":
        h0, h1, s0, s1, v0, v1 = values
        return hsv_range_mask(rgb, h0, h1, s0, s1, v0, v1)
    a0, a1, b0, b1, s0, s1, v0, v1 = values
    return cv2.bitwise_or(hsv_range_mask(rgb, a0, a1, s0, s1, v0, v1),
                          hsv_range_mask(rgb, b0, b1, s0, s1, v0, v1))


def printable(values, target):
    if target == "green":
        names = ["BANNER_H_LOW", "BANNER_H_HIGH", "BANNER_S_LOW", "BANNER_S_HIGH",
                 "BANNER_V_LOW", "BANNER_V_HIGH"]
        return "\n".join(f"{n} = {v}" for n, v in zip(names, values))
    a0, a1, b0, b1, s0, s1, v0, v1 = values
    return (f"RED_H_LOW1, RED_H_HIGH1 = {a0}, {a1}\nRED_H_LOW2, RED_H_HIGH2 = {b0}, {b1}\n"
            f"RED_S_LOW, RED_S_HIGH = {s0}, {s1}\nRED_V_LOW, RED_V_HIGH = {v0}, {v1}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", choices=["green", "red"], default="green")
    ap.add_argument("--image", help="use a saved picture instead of the camera")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    cfg = settings_for(args.target)

    if args.selftest:
        img = np.full((720, 1280, 3), 100, np.uint8)
        img[200:400, 300:600] = (30, 160, 60) if args.target == "green" else (220, 30, 30)
        m = mask_from([v for _, v, _ in cfg], args.target, img)
        print(f"[tune] selftest: {cv2.countNonZero(m)} white pixels (expected 60000)")
        print(printable([v for _, v, _ in cfg], args.target))
        return

    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"Could not read image: {args.image}")
        source = lambda: cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    else:
        from hardware.camera import build_camera_rig
        view = build_camera_rig().view("forward")
        source = view.capture_array

    win = f"tune_hsv {args.target}  (p = print numbers, q = quit)"
    cv2.namedWindow(win)
    for name, start, top in cfg:
        cv2.createTrackbar(name, win, int(start), top, lambda _: None)

    while True:
        rgb = source()
        values = [cv2.getTrackbarPos(name, win) for name, _, _ in cfg]
        mask = mask_from(values, args.target, rgb)
        small = lambda im: cv2.resize(im, (640, 360))
        both = np.hstack([small(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)),
                          small(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))])
        cv2.imshow(win, both)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("p"):
            print("\n--- copy these into config/params.py ---")
            print(printable(values, args.target))
        elif key == ord("q"):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
