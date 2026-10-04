# =============================================================================
# vision/qr_scanner.py — Team Vajra AeroTHON 2026
#
# Layered QR decoding: pyzbar over several preprocessing variants (fast path),
# then OpenCV's QR detector as fallback.
#
# CHANGES vs previous version
#   [NEW] decode_all(frame, want=None) -> list[QRDetection]
#         Every QR in the frame WITH its pixel centre. Needed for the delivery
#         zone, where many QR codes are visible and only one matches.
#         `want` = stop early as soon as that payload is found (saves CPU on the Pi).
#   [NEW] QRConfirmer: accept a value only after QR_CONFIRM_COUNT consecutive
#         identical decodes (params.QR_CONFIRM_COUNT) — rejects one-frame misreads.
#   [FIX] pyzbar restricted to QR symbols (no false 1-D barcode hits).
#   [FIX] OpenCV fallback also uses detectAndDecodeMulti when available.
#   decode_frame() keeps its old behaviour (first decoded string or None).
#
# Frame convention: RGB uint8 (picamera2 native). For OpenCV webcams (BGR) the
# result is still fine for QR, but convert with cv2.COLOR_BGR2RGB for consistency.
# =============================================================================

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
from pyzbar.pyzbar import decode as pyzbar_decode, ZBarSymbol

from config.params import QR_CONFIRM_COUNT
from vision.preprocessing import preprocess_for_qr

_QR_ONLY = [ZBarSymbol.QRCODE]
_CLAHE_IDX = 2   # preprocess_for_qr(): [frame, gray, equalized(CLAHE), ...]


@dataclass
class QRDetection:
    data: str
    cx: float                      # centre x, pixels
    cy: float                      # centre y, pixels
    method: str                    # which decoder found it
    corners: Optional[np.ndarray] = None   # (4,2) if available


class RobustQRScanner:
    def __init__(self):
        self.cv_detector = cv2.QRCodeDetector()
        self._decode_attempts = 0
        self._pyzbar_hits = 0
        self._opencv_hits = 0

    # ── Single value (Task A: start QR) ───────────────────────────────────────
    def decode_frame(self, frame) -> Optional[str]:
        """First decoded payload in the frame, or None."""
        dets = self.decode_all(frame)
        return dets[0].data if dets else None

    # ── All codes with positions (Task B: delivery zone) ──────────────────────
    def decode_all(self, frame, want: Optional[str] = None) -> List[QRDetection]:
        """
        Every QR found, de-duplicated by payload (first hit wins).
        If `want` is given, returns as soon as a detection with that payload
        exists (the list then contains everything found up to that point).
        """
        self._decode_attempts += 1
        found = {}

        def add(det: QRDetection) -> bool:
            if det.data and det.data not in found:
                found[det.data] = det
            return want is not None and want in found

        variants = preprocess_for_qr(frame)

        # Pass 1: pyzbar on every variant
        for img in variants:
            for obj in pyzbar_decode(img, symbols=_QR_ONLY):
                data = obj.data.decode("utf-8", errors="replace").strip()
                r = obj.rect
                det = QRDetection(
                    data=data,
                    cx=r.left + r.width / 2.0,
                    cy=r.top + r.height / 2.0,
                    method="pyzbar",
                    corners=np.array([[p.x, p.y] for p in obj.polygon], dtype=float)
                    if obj.polygon else None,
                )
                if add(det):
                    self._pyzbar_hits += 1
                    return list(found.values())
        if found:
            self._pyzbar_hits += 1
            if want is None:
                # still try OpenCV below only if nothing found; pyzbar is enough
                return list(found.values())

        # Pass 2: OpenCV on original + CLAHE (handles perspective distortion)
        for img in (variants[0], variants[_CLAHE_IDX]):
            for det in self._opencv_detect(img):
                if add(det):
                    self._opencv_hits += 1
                    return list(found.values())
        if found:
            self._opencv_hits += 1
        return list(found.values())

    def _opencv_detect(self, img) -> List[QRDetection]:
        out: List[QRDetection] = []
        try:
            if hasattr(self.cv_detector, "detectAndDecodeMulti"):
                ok, infos, points, _ = self.cv_detector.detectAndDecodeMulti(img)
                if ok and points is not None:
                    for data, pts in zip(infos, points):
                        if data:
                            out.append(self._cv_det(data, pts))
                    if out:
                        return out
            data, pts, _ = self.cv_detector.detectAndDecode(img)
            if data and pts is not None:
                out.append(self._cv_det(data, pts[0]))
        except cv2.error:
            pass
        return out

    @staticmethod
    def _cv_det(data, pts) -> QRDetection:
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        return QRDetection(
            data=data.strip(),
            cx=float(pts[:, 0].mean()),
            cy=float(pts[:, 1].mean()),
            method="opencv",
            corners=pts,
        )

    def stats(self):
        total = self._decode_attempts
        print(f"Decode attempts: {total}")
        print(f"pyzbar hits: {self._pyzbar_hits} ({100 * self._pyzbar_hits / max(total, 1):.0f}%)")
        print(f"OpenCV hits: {self._opencv_hits} ({100 * self._opencv_hits / max(total, 1):.0f}%)")


class QRConfirmer:
    """Accept a payload only after N consecutive identical sightings."""

    def __init__(self, needed: int = QR_CONFIRM_COUNT):
        self.needed = needed
        self._last: Optional[str] = None
        self._count = 0

    def reset(self):
        self._last, self._count = None, 0

    def update(self, value: Optional[str]) -> Optional[str]:
        """Feed one frame's result (None = nothing seen). Returns the value
        once confirmed, else None."""
        if value is None:
            self.reset()
            return None
        if value == self._last:
            self._count += 1
        else:
            self._last, self._count = value, 1
        return value if self._count >= self.needed else None
