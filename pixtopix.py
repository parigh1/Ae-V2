import numpy as np


class PixelToMeters:
    """
    Converts pixel offset (from frame center) to real-world meters.

    Pi Camera v3 wide specs:
      Sensor: 12MP IMX708
      Full FOV: 102° diagonal
      At 1280x720: horizontal ~84°, vertical ~64°

    Standard Pi Camera v2 specs (if using this instead):
      Full FOV: 62.2° horizontal, 48.8° vertical
    """

    # Change these to match your actual camera
    CAMERA_FOV_H_DEG = 84.0  # horizontal field of view
    CAMERA_FOV_V_DEG = 64.0  # vertical field of view
    IMG_W = 1280
    IMG_H = 720

    @classmethod
    def offset_at_altitude(cls, dx_px, dy_px, altitude_m):
        """
        dx_px: positive = QR is to the RIGHT of frame center
        dy_px: positive = QR is BELOW frame center (image y-down)
        altitude_m: current rangefinder reading

        Returns (dx_m, dy_m) in drone body frame:
          dx_m positive = drone needs to move EAST
          dy_m positive = drone needs to move NORTH

        Note: camera +x (right) maps to drone +East (NED vy)
              camera +y (down)  maps to drone +North (NED vx) because
              camera faces forward-and-down in typical mounting.
              Adjust signs based on your actual camera mounting.
        """
        fov_h = np.radians(cls.CAMERA_FOV_H_DEG)
        fov_v = np.radians(cls.CAMERA_FOV_V_DEG)

        # Real-world width/height visible at this altitude
        ground_width = 2 * altitude_m * np.tan(fov_h / 2)
        ground_height = 2 * altitude_m * np.tan(fov_v / 2)

        # Meters per pixel
        m_per_px_x = ground_width / cls.IMG_W
        m_per_px_y = ground_height / cls.IMG_H

        dx_m = dx_px * m_per_px_x
        dy_m = dy_px * m_per_px_y

        return dx_m, dy_m

# At 5m altitude with Pi Camera v3 wide:
# ground_width  = 2 * 5 * tan(42°) = 9.00 m visible
# m_per_px_x    = 9.00 / 1280     = 0.0070 m/px
# So a 100px offset → 0.70m real displacement

# At 10m altitude:
# ground_width  = 2 * 10 * tan(42°) = 18.00 m visible
# m_per_px_x    = 18.00 / 1280      = 0.0141 m/px
# So a 100px offset → 1.41m real displacement