import cv2
import numpy as np


class QRDetector:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        # For OpenCV 4.8+ you can use the better detector:
        # self.detector = cv2.QRCodeDetectorAruco()

    def decode_opencv(self, frame):
        """
        Returns (data_string, points, straight_qr_image) or (None, None, None)
        points = 4 corner points of the QR in the image
        """
        data, points, straight = self.detector.detectAndDecode(frame)
        if data and len(data) > 0:
            return data, points, straight
        return None, None, None

    def get_qr_center_pixels(self, frame):
        """
        Returns pixel coordinates of QR center in the frame, or None.
        Use this for visual servoing positioning.
        """
        data, points, _ = self.decode_opencv(frame)
        if data is None or points is None:
            return None, None
        # points shape is (1, 4, 2) — four corners
        corners = points[0]
        cx = int(np.mean(corners[:, 0]))
        cy = int(np.mean(corners[:, 1]))
        return data, (cx, cy)