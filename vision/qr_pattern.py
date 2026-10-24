# =============================================================================
# vision/qr_pattern.py — Team Vajra AeroTHON 2026
#
# Pattern-based QR matching for the AeroTHON delivery task.
#
# The organizers confirmed (Phase 2 call) that the QR codes carry no text:
# they are patterns to match, not codes to decode. We still try to decode them
# (pyzbar works when the code happens to be valid), but the primary path is now:
#
#   1. Locate all QRs in the frame (using pyzbar / OpenCV detector).
#   2. Rectify each one to a fixed NxN grayscale patch.
#   3. Score it against the stored start-QR template with normalised cross-
#      correlation (NCC), trying all four 90° rotations.
#   4. Accept the one with the highest score, provided it is above QR_MATCH_MIN
#      AND beats every other candidate by QR_MATCH_MARGIN.
#
# The corner-square mask removes the three finder patterns every QR shares —
# they carry no pattern-specific information and inflate scores for different
# codes.
#
# This sits on top of qr_scanner.py so the "codes carry text" path still works
# if the codes happen to be valid (which helps in simulation / testing).
# =============================================================================

import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode, ZBarSymbol
from typing import List, Optional

from config.params import (QR_PATTERN_SIZE, QR_PATTERN_CORNER_MASK,
                           QR_MATCH_MIN, QR_MATCH_MARGIN,
                           QR_UNDECODED_MATCH_MIN, QR_PATTERN_CONFIRM_MIN)


def _make_corner_mask(size: int, frac: float) -> np.ndarray:
    mask = np.ones((size, size), dtype=bool)
    c = int(round(size * frac))
    for ys in (slice(0, c), slice(size - c, size)):
        for xs in (slice(0, c), slice(size - c, size)):
            mask[ys, xs] = False
    return mask


_CORNER_MASK = _make_corner_mask(QR_PATTERN_SIZE, QR_PATTERN_CORNER_MASK)


def _order_corners(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, np.float32).reshape(4, 2)
    c = pts.mean(axis=0)
    angle = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(angle)]


def _rectify(gray: np.ndarray, corners: np.ndarray,
             size: int = QR_PATTERN_SIZE) -> np.ndarray:
    src = _order_corners(corners)
    dst = np.float32([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]])
    M = cv2.getPerspectiveTransform(src, dst)
    patch = cv2.warpPerspective(gray, M, (size, size), flags=cv2.INTER_AREA)
    patch = patch.astype(np.float32)
    lo, hi = np.percentile(patch, 5), np.percentile(patch, 95)
    patch = np.clip((patch - lo) / max(hi - lo, 1.0) * 2.0 - 1.0, -1.2, 1.2)
    return patch


def _ncc(a: np.ndarray, b: np.ndarray, mask: np.ndarray) -> float:
    a = a[mask]; b = b[mask]
    a = a - a.mean(); b = b - b.mean()
    denom = float(np.sqrt((a * a).sum() * (b * b).sum()))
    return 0.0 if denom < 1e-6 else float((a * b).sum() / denom)


def _best_ncc(template: np.ndarray, candidate: np.ndarray,
              mask: np.ndarray = _CORNER_MASK) -> float:
    return max(_ncc(template, np.rot90(candidate, k), mask) for k in range(4))


def locate_all_qrs(frame: np.ndarray) -> List[dict]:
    """Return a list of dicts: {data: str | None, corners: ndarray (4,2), patch: ndarray}."""
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) if frame.ndim == 3 else frame
    results = []
    seen_corners = []

    def _add(data, corners):
        if corners is None or len(corners) < 4:
            return
        corners = np.asarray(corners, np.float32).reshape(-1, 2)[:4]
        c = corners.mean(axis=0)
        for prev_c in seen_corners:
            if np.linalg.norm(c - prev_c) < 20:
                return
        seen_corners.append(c)
        results.append({"data": data or None, "corners": corners,
                        "patch": _rectify(gray, corners)})

    for obj in pyzbar_decode(gray, symbols=[ZBarSymbol.QRCODE]):
        if obj.polygon and len(obj.polygon) == 4:
            pts = np.array([[p.x, p.y] for p in obj.polygon], np.float32)
            data = obj.data.decode("utf-8", errors="replace").strip() or None
            _add(data, pts)

    det = cv2.QRCodeDetector()
    try:
        if hasattr(det, "detectAndDecodeMulti"):
            ok, infos, points, _ = det.detectAndDecodeMulti(gray)
            if ok and points is not None:
                for info, pts in zip(infos, points):
                    _add(info.strip() or None, pts)
        else:
            info, pts, _ = det.detectAndDecode(gray)
            if pts is not None:
                _add(info.strip() or None, pts[0])
    except cv2.error:
        pass

    return results


class QRTemplate:
    """Stores the start QR as a rectified pattern. Built from the first scan."""

    def __init__(self):
        self._patches: List[np.ndarray] = []
        self.confirmed: bool = False

    def reset(self):
        self._patches.clear(); self.confirmed = False

    def add(self, patch: np.ndarray):
        if self._patches:
            score = _best_ncc(self._patches[0], patch)
            if score >= QR_PATTERN_CONFIRM_MIN:
                self._patches.append(patch)
                if len(self._patches) >= 2:
                    self.confirmed = True
            else:
                self.reset(); self._patches.append(patch)
        else:
            self._patches.append(patch)

    @property
    def template(self) -> Optional[np.ndarray]:
        if not self._patches:
            return None
        return np.mean(self._patches, axis=0).astype(np.float32)


class QRMatcher:
    """Finds the delivery-zone QR whose pattern best matches the stored start QR."""

    def __init__(self):
        self._template: Optional[np.ndarray] = None

    def update_template(self, tmpl: QRTemplate):
        self._template = tmpl.template

    def has_template(self) -> bool:
        return self._template is not None

    def find_best(self, frame: np.ndarray, height_m: float,
                  pitch_deg: float, img_w: int, img_h: int) -> Optional[dict]:
        """Return the best-matching QR or None.
        Result dict: {score, cx, cy, corners, data, forward_m, right_m}"""
        if self._template is None:
            return None
        detections = locate_all_qrs(frame)
        if not detections:
            return None

        scores = [(
            _best_ncc(self._template, d["patch"]),
            QR_UNDECODED_MATCH_MIN if d["data"] is None else QR_MATCH_MIN,
            d
        ) for d in detections]
        scores.sort(key=lambda x: x[0], reverse=True)
        best_s, best_thresh, best_d = scores[0]

        if best_s < best_thresh:
            return None
        if len(scores) > 1 and best_s - scores[1][0] < QR_MATCH_MARGIN:
            return None

        cx = float(best_d["corners"][:, 0].mean())
        cy = float(best_d["corners"][:, 1].mean())
        from vision.pixel_to_meters import PixelToMeters
        ground = PixelToMeters.offset_body(cx - img_w / 2, cy - img_h / 2,
                                           height_m, img_w, img_h, pitch_deg)
        if ground is None:
            return None
        fwd, right = ground
        return {"score": best_s, "cx": cx, "cy": cy,
                "corners": best_d["corners"], "data": best_d["data"],
                "forward_m": fwd, "right_m": right}