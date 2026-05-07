from pyzbar.pyzbar import decode as pyzbar_decode
import cv2

from qrpipe import preprocess_for_qr


class RobustQRScanner:
    def __init__(self):
        self.cv_detector = cv2.QRCodeDetector()
        self._decode_attempts = 0
        self._pyzbar_hits = 0
        self._opencv_hits = 0

    def decode_frame(self, frame):
        """
        Returns decoded string or None.
        Tracks which library succeeded — useful for tuning.
        """
        self._decode_attempts += 1
        variants = preprocess_for_qr(frame)

        # Pass 1: pyzbar on all variants (fast path)
        for img in variants:
            results = pyzbar_decode(img)
            if results:
                data = results[0].data.decode('utf-8').strip()
                if data:
                    self._pyzbar_hits += 1
                    return data

        # Pass 2: OpenCV on original and equalized (handles distortion)
        for img in [frame, variants[2]]:  # original + CLAHE
            data, _, _ = self.cv_detector.detectAndDecode(img)
            if data and len(data) > 0:
                self._opencv_hits += 1
                return data.strip()

        return None

    def stats(self):
        total = self._decode_attempts
        print(f"Decode attempts: {total}")
        print(f"pyzbar hits: {self._pyzbar_hits} ({100 * self._pyzbar_hits / max(total, 1):.0f}%)")
        print(f"OpenCV hits:  {self._opencv_hits} ({100 * self._opencv_hits / max(total, 1):.0f}%)")