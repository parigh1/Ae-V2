import cv2
from config import params
from navigation.corridor import VisionObstacleSensor
from tests.test_corridor import drone_at, PITCH, CorridorWorldCamera

veh = drone_at(e=0.0)
cam = CorridorWorldCamera(veh, PITCH, obstacles=[(2.0, -0.2, 0.4, 0.4)])
frame = cam.capture_array()
print("PITCH used by the test:", PITCH, "| picture:", frame.shape, "| mount in params:", params.CAM_MOUNTS[params.CAMERA_MODE])
hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
mask = cv2.inRange(hsv, (0, 0, 0), (179, 255, params.OBSTACLE_V_MAX))
print("dark pixels:", int(mask.sum() / 255), "| minimum blob area:", params.OBSTACLE_MIN_AREA)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for c in sorted(contours, key=cv2.contourArea, reverse=True)[:5]:
    print("   blob area", int(cv2.contourArea(c)), "box (x, y, w, h)", cv2.boundingRect(c))
print("sensor result:", VisionObstacleSensor().measure(frame, 3.0, PITCH))
cv2.imwrite("box_frame.png", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
cv2.imwrite("box_mask.png", mask)