import cv2
import numpy as np


def preprocess_for_qr(frame):
    """
    Full preprocessing stack for aerial QR detection.
    Returns a list of variants to try — best first.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Stage 1: CLAHE — normalizes local contrast
    # clipLimit=2.0 prevents over-amplifying noise
    # tileGridSize=(8,8) divides image into 64 tiles
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    equalized = clahe.apply(gray)

    # Stage 2: Adaptive threshold
    # blockSize=11 means each pixel uses a 11x11 neighborhood
    # C=2 subtracts a constant to fine-tune the threshold
    adaptive = cv2.adaptiveThreshold(
        equalized, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 11, 2
    )

    # Stage 3: Sharpening kernel
    # This enhances edges that motion blur softened
    sharpen_kernel = np.array([[0, -1, 0],
                               [-1, 5, -1],
                               [0, -1, 0]])
    sharpened = cv2.filter2D(adaptive, -1, sharpen_kernel)

    # Stage 4: Morphological closing
    # Fills small gaps in QR modules caused by sensor noise
    kernel = np.ones((2, 2), np.uint8)
    closed = cv2.morphologyEx(sharpened, cv2.MORPH_CLOSE, kernel)

    # Return in order of quality — pyzbar tries each in sequence
    return [
        frame,  # original color — try first, sometimes just works
        gray,  # plain grayscale
        equalized,  # CLAHE only
        adaptive,  # threshold only
        sharpened,  # sharpen only
        closed  # full stack
    ]