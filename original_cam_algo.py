import cv2
import numpy as np
import math
import json
import os
import time
import csv

# =========================
# Config
# =========================
VIDEO_PATH       = "output.mp4"
THRESHOLDS_JSON  = "thresholds.json"

# Match C++ processing size
RESIZE_TO        = (655, 600)

# For max-speed benchmarking, turn this OFF
ENABLE_VISUALS   = False   # True: show windows & draw; False: no GUI (faster)

# Optional morphology (C++ didn’t use it). Leave False for strict parity/speed.
USE_MORPH        = False

# Area cutoffs (C++ used 5 for ball, 400 for goals)
BALL_MIN_AREA    = 5
GOAL_MIN_AREA    = 400

# =========================
# Utilities
# =========================
def clamp(v, lo, hi): 
    return max(lo, min(hi, v))

def parse_hsv(obj):
    """Case-insensitive h/s/v dict → clamped OpenCV HSV triple."""
    h = obj.get("H", obj.get("h"))
    s = obj.get("S", obj.get("s"))
    v = obj.get("V", obj.get("v"))
    return (clamp(int(h),0,179), clamp(int(s),0,255), clamp(int(v),0,255))

def load_thresholds(path):
    with open(path, "r") as f:
        T = json.load(f)

    lower_ball   = parse_hsv(T["ball"]["lower"])
    upper_ball   = parse_hsv(T["ball"]["upper"])
    lower_yellow = parse_hsv(T["yellowGoal"]["lower"])
    upper_yellow = parse_hsv(T["yellowGoal"]["upper"])
    lower_blue   = parse_hsv(T["blueGoal"]["lower"])
    upper_blue   = parse_hsv(T["blueGoal"]["upper"])

    xoffset = int(T.get("offsets", {}).get("x", 0))
    yoffset = int(T.get("offsets", {}).get("y", 0))

    # Elliptical mask like C++
    mask_tuple = None
    if "mask1" in T:
        cx = int(T["mask1"]["x"])
        cy = int(T["mask1"]["y"])
        s1 = int(T["mask1"]["size1"])
        s2 = int(T["mask1"]["size2"])
        mask_tuple = (cx, cy, s1, s2)

    return (lower_ball, upper_ball,
            lower_yellow, upper_yellow,
            lower_blue, upper_blue,
            xoffset, yoffset, mask_tuple)

def largest_contour(mask, min_area):
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: 
        return None
    best = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(best) < min_area:
        return None
    return cv2.boundingRect(best)  # (x,y,w,h)

def angle_dist_from_bbox(bbox, xoffset, yoffset):
    """Replicates the C++ mid-point/angle/dist math."""
    x,y,w,h = bbox
    cx = x + w//2
    cy = y + h//2
    midx = cx - xoffset
    midy = cy - yoffset
    angle_deg = math.degrees(math.atan2(midx, midy)) + 180.0
    dist_px   = 1.5 * math.hypot(midx, midy)   # raw pixel distance scaled like C++
    return angle_deg, dist_px, (cx, cy)

def apply_ball_distance_calibration(balldist):
    """Piecewise exponential mapping identical to C++."""
    if balldist == -5:
        return -5
    if balldist < 185:
        return -228.02 * math.exp(-0.00198188 * balldist) + 200.086
    else:
        return 7.01168 * math.exp(0.00594217 * balldist) + 27.48

# =========================
# Main
# =========================
def main():
    if not os.path.exists(THRESHOLDS_JSON):
        print("Failed to read thresholds:", THRESHOLDS_JSON)
        return

    (low_ball, up_ball,
     low_yel,  up_yel,
     low_blue, up_blue,
     xoff, yoff, mask_tuple) = load_thresholds(THRESHOLDS_JSON)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("Error: Could not open video.")
        return

    # Prepare mask & optional morphology kernel when frame size is known
    mask_img = None
    kernel = np.ones((3,3), np.uint8)

    # State carried across frames for C++ derivative calc
    prevBall = -1
    prevDist = -1

    # Timing for Performance Stats + CSV
    frame_times_ms = []
    csv_rows = []

    if ENABLE_VISUALS:
        print("Press 'q' to quit.")

    frame_idx = 0
    while True:
        t0 = time.perf_counter()

        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1

        # Resize like C++
        if RESIZE_TO is not None:
            frame = cv2.resize(frame, RESIZE_TO, interpolation=cv2.INTER_AREA)
        H, W = frame.shape[:2]

        # Build ellipse mask once at working resolution
        if mask_img is None:
            mask_img = np.zeros((H, W), dtype=np.uint8)
            if mask_tuple is not None:
                cx, cy, s1, s2 = mask_tuple
            else:
                # Fallback: full frame
                cx, cy, s1, s2 = W//2, H//2, W//2, H//2
            cv2.ellipse(mask_img, (int(cx), int(cy)), (int(s1), int(s2)), 0, 0, 360, 255, -1)

        # Apply mask like the C++ pipeline (copy into black)
        masked = cv2.bitwise_and(frame, frame, mask=mask_img)

        # Convert to HSV and threshold for each class
        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

        # ---- Ball ----
        ballMask = cv2.inRange(hsv, low_ball, up_ball)
        if USE_MORPH:
            ballMask = cv2.morphologyEx(ballMask, cv2.MORPH_OPEN, kernel, iterations=1)
            ballMask = cv2.morphologyEx(ballMask, cv2.MORPH_CLOSE, kernel, iterations=1)
        ball_bbox = largest_contour(ballMask, BALL_MIN_AREA)

        ballAngle  = -5.0
        balldist_c = -5.0
        roundedD   = -5.0

        if ball_bbox is not None:
            ballAngle, balldist_raw, (bcx,bcy) = angle_dist_from_bbox(ball_bbox, xoff, yoff)
            balldist_c = apply_ball_distance_calibration(balldist_raw)

            # Derivative like C++
            newballAngle = (360 - ballAngle) if (ballAngle > 180) else ballAngle
            dInput = (math.sin(math.radians(int(newballAngle))) * int(balldist_c)) - \
                     (math.sin(math.radians(prevBall)) * int(prevDist))

            if dInput < 0 and (ballAngle < 90 or ballAngle > 270):
                roundedD = round(-dInput * 100) / 100.0

            prevBall = int(newballAngle)
            prevDist = int(balldist_c)

            if ENABLE_VISUALS:
                x,y,w,h = ball_bbox
                cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,255), 2)
                cv2.circle(frame, (bcx,bcy), 4, (0,0,255), -1)
                cv2.putText(frame, f"angle={ballAngle:.1f}  dist~{balldist_c:.1f}  d={roundedD:.2f}",
                            (x, max(20, y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0,255,0), 2, cv2.LINE_AA)

        # ---- Yellow goal ----
        yelMask = cv2.inRange(hsv, low_yel, up_yel)
        if USE_MORPH:
            yelMask = cv2.morphologyEx(yelMask, cv2.MORPH_OPEN, kernel, iterations=1)
            yelMask = cv2.morphologyEx(yelMask, cv2.MORPH_CLOSE, kernel, iterations=1)
        yel_bbox = largest_contour(yelMask, GOAL_MIN_AREA)
        yellowAngle = -5.0
        if yel_bbox is not None:
            yellowAngle, _, _ = angle_dist_from_bbox(yel_bbox, xoff, yoff)

        # ---- Blue goal ----
        bluMask = cv2.inRange(hsv, low_blue, up_blue)
        if USE_MORPH:
            bluMask = cv2.morphologyEx(bluMask, cv2.MORPH_OPEN, kernel, iterations=1)
            bluMask = cv2.morphologyEx(bluMask, cv2.MORPH_CLOSE, kernel, iterations=1)
        blu_bbox = largest_contour(bluMask, GOAL_MIN_AREA)
        blueAngle = -5.0
        if blu_bbox is not None:
            blueAngle, _, _ = angle_dist_from_bbox(blu_bbox, xoff, yoff)

        # Timing
        loop_ms = (time.perf_counter() - t0) * 1000.0
        fps = 1000.0 / loop_ms if loop_ms > 0 else 0.0
        frame_times_ms.append(loop_ms)
        csv_rows.append([frame_idx, loop_ms, fps])

        # Console output like C++
        # Ball Angle: <>, Dist: <>, Blue Angle: <>, Yellow Angle: <>, Derivative: <>
        # Processing FPS = <>
        print(f"Ball Angle: {ballAngle:.1f}, Dist: {balldist_c:.1f}, "
              f"Blue Angle: {blueAngle:.1f}, Yellow Angle: {yellowAngle:.1f}, "
              f"Derivative: {roundedD:.2f}")
        print(f"Processing FPS = {fps:.1f}")

        if ENABLE_VISUALS:
            # Draw ellipse ROI outline
            if mask_tuple is not None:
                cx, cy, s1, s2 = mask_tuple
            else:
                cx, cy, s1, s2 = W//2, H//2, W//2, H//2
            cv2.ellipse(frame, (int(cx), int(cy)), (int(s1), int(s2)),
                        0, 0, 360, (0,255,255), 2)

            cv2.putText(frame, f"Process: {loop_ms:.2f}ms | FPS: {fps:.1f}",
                        (10, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            cv2.imshow("Ball Detection (video)", frame)
            cv2.imshow("Ball mask", ballMask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    if ENABLE_VISUALS:
        cv2.destroyAllWindows()

    # =========================
    # Performance Stats + CSV
    # =========================
    if frame_times_ms:
        avg_ms = sum(frame_times_ms) / len(frame_times_ms)
        min_ms = min(frame_times_ms)
        max_ms = max(frame_times_ms)
        avg_fps = 1000.0 / avg_ms if avg_ms > 0 else 0.0

        # Match your print format exactly
        print("\nPerformance Stats:")
        print(f"Average processing time: {avg_ms:.2f}ms ({avg_fps:.1f} FPS)")
        print(f"Min/Max processing time: {min_ms:.2f}ms / {max_ms:.2f}ms")
        print(f"Total frames processed: {len(frame_times_ms)}")

        csv_name = "performance_data_with_visuals.csv" if ENABLE_VISUALS else "performance_data_no_visuals.csv"
        with open(csv_name, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Frame", "Processing_Time_ms", "FPS"])
            w.writerows(csv_rows)
        print(f"Data saved to: {csv_name}")

if __name__ == "__main__":
    main()
