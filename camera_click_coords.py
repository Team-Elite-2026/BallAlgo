import cv2
import numpy as np
from picamera2 import Picamera2

RESIZE_TO = (655, 600)

click_pos = None

def on_mouse(event, x, y, flags, param):
    global click_pos
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos = (x, y)

picam2 = Picamera2()
cam_config = picam2.create_video_configuration(
    main={"size": RESIZE_TO, "format": "RGB888"},
    controls={"FrameRate": 60}
)
picam2.configure(cam_config)
picam2.start()

cv2.namedWindow("Camera")
cv2.setMouseCallback("Camera", on_mouse)

print("Click anywhere on the feed to see coordinates. Press 'q' to quit.")

while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if click_pos is not None:
        x, y = click_pos
        cv2.drawMarker(frame, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        label = f"({x}, {y})"
        tx = x + 10 if x < RESIZE_TO[0] - 100 else x - 120
        ty = y - 10 if y > 20 else y + 20
        cv2.putText(frame, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
picam2.close()
cv2.destroyAllWindows()
