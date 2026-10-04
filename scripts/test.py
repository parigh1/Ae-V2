import cv2
import numpy as np
import time
import sys

print("=" * 50)
print("QR SYSTEM TEST — AeroTHON 2026")
print("=" * 50)

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    print("[OK] pyzbar imported")
except ImportError as e:
    print(f"[FAIL] pyzbar: {e}")
    sys.exit(1)

print(f"[OK] OpenCV version {cv2.__version__}")
print()

# ── Camera open ────────────────────────────────────────────────────
print("[STEP 1] Opening camera...")

camera = None
for idx in [0, 1, 2]:
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        cap.release()
        continue
    ret, test_frame = cap.read()
    if ret and test_frame is not None:
        camera = cap
        print(f"[OK] Camera opened at index {idx}")
        break
    cap.release()

if camera is None:
    print("[FAIL] No camera found")
    sys.exit(1)

# ── Read ACTUAL resolution — do NOT force 1280x720 ─────────────────
# Just use whatever the camera gives us natively
ret, sample = camera.read()
if not ret:
    print("[FAIL] Cannot read first frame")
    sys.exit(1)

IMG_H, IMG_W = sample.shape[:2]
print(f"     Camera native resolution: {IMG_W}x{IMG_H}")
print()


# ── Preprocessing ──────────────────────────────────────────────────
def preprocess_for_qr(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    equalized = clahe.apply(gray)

    adaptive = cv2.adaptiveThreshold(
        equalized, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )

    kernel = np.array([[ 0, -1,  0],
                       [-1,  5, -1],
                       [ 0, -1,  0]])
    sharpened = cv2.filter2D(adaptive, -1, kernel)

    morph_k = np.ones((2, 2), np.uint8)
    closed = cv2.morphologyEx(sharpened, cv2.MORPH_CLOSE, morph_k)

    return [frame, gray, equalized, adaptive, sharpened, closed]


# ── Decode one frame ───────────────────────────────────────────────
def decode_frame(frame, cv_detector):
    variants = preprocess_for_qr(frame)
    names    = ["original", "gray", "clahe",
                "adaptive", "sharpened", "closed"]

    # pyzbar first (fast)
    for img, name in zip(variants, names):
        results = pyzbar_decode(img)
        if results:
            data = results[0].data.decode("utf-8").strip()
            if data:
                return data, f"pyzbar/{name}"

    # OpenCV fallback (handles distortion better)
    for img, name in zip([variants[0], variants[2]], ["original", "clahe"]):
        gray_img = img if len(img.shape) == 2 \
                       else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        data, _, _ = cv_detector.detectAndDecode(gray_img)
        if data and len(data) > 0:
            return data.strip(), f"opencv/{name}"

    return None, None


# ── Pixel offset → meters ──────────────────────────────────────────
def pixel_offset_to_meters(dx_px, dy_px, altitude_m,
                            img_w, img_h,
                            fov_h_deg=84.0, fov_v_deg=64.0):
    fov_h      = np.radians(fov_h_deg)
    fov_v      = np.radians(fov_v_deg)
    ground_w   = 2 * altitude_m * np.tan(fov_h / 2)
    ground_h   = 2 * altitude_m * np.tan(fov_v / 2)
    return (dx_px * ground_w / img_w,
            dy_px * ground_h / img_h)


# ── Main loop ──────────────────────────────────────────────────────
def run_test():
    cv_detector  = cv2.QRCodeDetector()
    SIM_ALTITUDE = 5.0   # metres — simulates start QR scenario

    print("[STEP 2] Live QR scan running")
    print("         Point webcam at any QR code")
    print("         Press Q to quit | S to save snapshot")
    print("-" * 50)

    frame_count  = 0
    decode_count = 0
    last_result  = None
    last_method  = None
    fps_start    = time.time()
    fps_frames   = 0

    while True:
        ret, frame = camera.read()

        # ── Guard: skip bad frames instead of crashing ─────────────
        if not ret or frame is None or frame.size == 0:
            print("[WARN] Bad frame skipped")
            time.sleep(0.05)
            continue

        # ── Resize if camera gave wrong size ───────────────────────
        h, w = frame.shape[:2]
        if h != IMG_H or w != IMG_W:
            frame = cv2.resize(frame, (IMG_W, IMG_H))

        frame_count += 1
        fps_frames  += 1

        result, method = decode_frame(frame, cv_detector)

        if result:
            decode_count += 1

            if result != last_result:
                # ── New QR — print full details ────────────────────
                print(f"\n{'='*50}")
                print(f"[QR DECODED]")
                print(f"  Data         : '{result}'")
                print(f"  Method       : {method}")
                print(f"  Frame #      : {frame_count}")
                rate = 100 * decode_count / frame_count
                print(f"  Decode rate  : {decode_count}/{frame_count} ({rate:.1f}%)")

                # Find pixel position and draw
                for img in preprocess_for_qr(frame):
                    objs = pyzbar_decode(img)
                    if objs:
                        r     = objs[0].rect
                        qr_cx = r.left + r.width  // 2
                        qr_cy = r.top  + r.height // 2
                        dx_px = qr_cx - IMG_W // 2
                        dy_px = qr_cy - IMG_H // 2
                        dx_m, dy_m = pixel_offset_to_meters(
                            dx_px, dy_px, SIM_ALTITUDE, IMG_W, IMG_H)

                        print(f"  QR center    : ({qr_cx}, {qr_cy}) px")
                        print(f"  Offset px    : ({dx_px:+d}, {dy_px:+d})")
                        print(f"  Offset metres: ({dx_m:+.3f}m, {dy_m:+.3f}m)")
                        print(f"  Drone action : move "
                              f"{'right' if dx_m>0 else 'left'} {abs(dx_m):.2f}m, "
                              f"{'fwd' if dy_m>0 else 'back'} {abs(dy_m):.2f}m")
                        print(f"{'='*50}")

                        # Draw green box + center dot on frame
                        pts = np.array([
                            [r.left,          r.top],
                            [r.left+r.width,  r.top],
                            [r.left+r.width,  r.top+r.height],
                            [r.left,          r.top+r.height]
                        ])
                        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                        cv2.circle(frame, (qr_cx, qr_cy), 6, (0, 255, 0), -1)
                        cv2.circle(frame, (IMG_W//2, IMG_H//2), 4, (0, 255, 255), -1)
                        cv2.line(frame,
                                 (IMG_W//2, IMG_H//2),
                                 (qr_cx, qr_cy),
                                 (0, 200, 255), 1)
                        break

            last_result = result
            last_method = method

        else:
            # No QR — periodic status print
            if frame_count % 90 == 0:
                elapsed = time.time() - fps_start
                fps     = fps_frames / elapsed if elapsed > 0 else 0
                print(f"[SCAN] Frame {frame_count:4d} | "
                      f"{fps:4.1f} fps | "
                      f"Decodes: {decode_count} | "
                      f"No QR")
                fps_frames = 0
                fps_start  = time.time()

        # ── HUD overlay ────────────────────────────────────────────
        cx, cy = IMG_W // 2, IMG_H // 2
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 255, 255), 1)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 255, 255), 1)

        cv2.putText(frame,
                    f"Decoded: {decode_count}/{frame_count}  "
                    f"({100*decode_count/max(frame_count,1):.0f}%)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)

        if last_result:
            cv2.putText(frame, f"{last_result[:45]}",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            cv2.putText(frame, f"via {last_method}",
                        (10, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

        cv2.imshow("AeroTHON QR Test  [Q=quit  S=save]", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"snapshot_{frame_count}.jpg"
            cv2.imwrite(fname, frame)
            print(f"[SAVED] {fname}")

    # ── Summary ────────────────────────────────────────────────────
    print("\n" + "=" * 50)
    print("SESSION SUMMARY")
    print(f"  Frames processed : {frame_count}")
    print(f"  Successful decodes : {decode_count}")
    print(f"  Overall rate     : "
          f"{100*decode_count/max(frame_count,1):.1f}%")
    if last_result:
        print(f"  Last decoded data: '{last_result}'")
        print(f"  Last method used : {last_method}")
    print("=" * 50)

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_test()