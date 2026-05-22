import cv2
import json
import time
import numpy as np
from picamera2 import Picamera2

# ========================
# Config
# ========================
JSON_PATH = str(__import__("pathlib").Path(__file__).resolve().parent.parent / "thresholds.json")
CAM_RES = (640, 480)

TARGETS = ["ball", "yellowGoal", "blueGoal"]
CURRENT_TARGET = 0  # index into TARGETS

# Initial HSV margins
H_MARGIN = 10
S_MARGIN = 40
V_MARGIN = 40

clicked_hsv = None

# ========================
# Helpers
# ========================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def load_json():
    try:
        with open(JSON_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

def save_json(data):
    with open(JSON_PATH, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[SAVED] {JSON_PATH}")

def compute_bounds(hsv):
    h, s, v = hsv
    lower = (
        clamp(h - cv2.getTrackbarPos("H_margin", "HSV Picker"), 0, 179),
        clamp(s - cv2.getTrackbarPos("S_margin", "HSV Picker"), 0, 255),
        clamp(v - cv2.getTrackbarPos("V_margin", "HSV Picker"), 0, 255),
    )
    upper = (
        clamp(h + cv2.getTrackbarPos("H_margin", "HSV Picker"), 0, 179),
        clamp(s + cv2.getTrackbarPos("S_margin", "HSV Picker"), 0, 255),
        clamp(v + cv2.getTrackbarPos("V_margin", "HSV Picker"), 0, 255),
    )
    return lower, upper

def mouse_cb(event, x, y, flags, param):
    global clicked_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_img = param
        clicked_hsv = tuple(int(v) for v in hsv_img[y, x])
        print(f"[CLICK] HSV = {clicked_hsv}")

# ========================
# Camera Setup
# ========================
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": CAM_RES, "format": "RGB888"},
    controls={"FrameRate": 120}
)
picam2.configure(config)

picam2.start()
time.sleep(0.4)
picam2.set_controls({
"AeEnable": False,
"AwbEnable": False,
"ExposureTime": 10000,   # microseconds
"AnalogueGain": 0,
"Brightness": 0,
"Contrast": 1,
"Saturation": 1
})


# ========================
# UI Setup
# ========================
cv2.namedWindow("HSV Picker")
cv2.createTrackbar("H_margin", "HSV Picker", H_MARGIN, 50, lambda x: None)
cv2.createTrackbar("S_margin", "HSV Picker", S_MARGIN, 100, lambda x: None)
cv2.createTrackbar("V_margin", "HSV Picker", V_MARGIN, 100, lambda x: None)
cv2.namedWindow('HSV Picker', cv2.WINDOW_NORMAL)
cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)

data = load_json()

print("===================================")
print("HSV Picker Controls")
print("===================================")
print("Left Click : Sample HSV")
print("TAB        : Switch target")
print("S          : Save to JSON")
print("Q          : Quit")
print("===================================")

# ========================
# Main Loop
# ========================
while True:
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.setMouseCallback("HSV Picker", mouse_cb, hsv)

    target_name = TARGETS[CURRENT_TARGET]

    mask = None
    lower = upper = None

    if clicked_hsv is not None:
        lower, upper = compute_bounds(clicked_hsv)
        mask = cv2.inRange(hsv, lower, upper)

        cv2.putText(
            frame,
            f"{target_name.upper()} HSV {clicked_hsv}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            frame,
            f"LOW {lower}  HIGH {upper}",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )

    cv2.imshow("HSV Picker", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    if mask is not None:
        cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

    # Quit
    if key == ord("q"):
        break

    # Switch target
    if key == 9:  # TAB
        CURRENT_TARGET = (CURRENT_TARGET + 1) % len(TARGETS)
        clicked_hsv = None
        print(f"[TARGET] {TARGETS[CURRENT_TARGET]}")

    # Save
    if key == ord("s") and lower is not None:
        data[target_name] = {
            "lower": list(lower),
            "upper": list(upper)
        }
        save_json(data)

# ========================
# Cleanup
# ========================
picam2.stop()
picam2.close()
cv2.destroyAllWindows()
