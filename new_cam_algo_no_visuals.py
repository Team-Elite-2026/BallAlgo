import cv2
import numpy as np
import math
import json
import os
import time
import csv

# ========================
# Config
# ========================
VIDEO_PATH = "output.mp4"
USE_JSON_THRESHOLDS = True
THRESHOLDS_JSON = "thresholds.json"
RESIZE_TO = (655, 600)

CSV_PATH = "sector_performance_data_no_visuals.csv"

# HSV fallback (OpenCV ranges)
FALLBACK_LOWER = (5, 120, 120)
FALLBACK_UPPER = (25, 255, 255)

# Elliptical ROI fallback
ENABLE_MASK = True
MASK_CENTER = (330, 300)
MASK_AXES   = (280, 230)

# Sectors
NUM_SECTORS = 12
SECTOR_ANGLE = 360 // NUM_SECTORS

# Processing knobs
USE_MORPH = True
MORPH_ITERS = 1
MIN_AREA_PIX = 5

# Prediction knobs
VEL_ALPHA = 0.7            # EMA for velocity
VEL_MAX = 40.0             # px/frame clamp
LOOKAHEAD_MIN = 1          # frames
LOOKAHEAD_MAX = 3          # frames
LOOKAHEAD_SPEED_THRESH = 12.0  # px/frame: above this, add more lookahead

# NEW: stop as soon as we find a valid blob in a sector
STOP_ON_FIRST_HIT = True

# ========================
# Helpers
# ========================
def clamp(v, lo, hi): return max(lo, min(hi, v))

def load_thresholds():
    def parse_color(obj):
        if isinstance(obj, dict):
            h = obj.get("H", obj.get("h"))
            s = obj.get("S", obj.get("s"))
            v = obj.get("V", obj.get("v"))
            if h is None or s is None or v is None:
                raise ValueError("HSV dict missing h/s/v keys")
            return (clamp(int(h),0,179), clamp(int(s),0,255), clamp(int(v),0,255))
        elif isinstance(obj, (list, tuple)) and len(obj) == 3:
            h,s,v = map(int, obj)
            return (clamp(h,0,179), clamp(s,0,255), clamp(v,0,255))
        else:
            raise ValueError("Color threshold format not recognized")

    if USE_JSON_THRESHOLDS and os.path.exists(THRESHOLDS_JSON):
        with open(THRESHOLDS_JSON, "r") as f:
            T = json.load(f)
        lower_ball = parse_color(T["ball"]["lower"])
        upper_ball = parse_color(T["ball"]["upper"])
        xoffset = int(T.get("offsets", {}).get("x", 0))
        yoffset = int(T.get("offsets", {}).get("y", 0))
        mask_tuple = None
        if "mask1" in T:
            cx = int(T["mask1"]["x"]); cy = int(T["mask1"]["y"])
            s1 = int(T["mask1"]["size1"]); s2 = int(T["mask1"]["size2"])
            mask_tuple = (cx, cy, s1, s2)
        return lower_ball, upper_ball, (xoffset, yoffset), mask_tuple

    return FALLBACK_LOWER, FALLBACK_UPPER, (0,0), None

def angle_and_distance_cpp(cx, cy, xoffset=0, yoffset=0):
    """Match your C++ math exactly."""
    midx = cx - xoffset
    midy = cy - yoffset
    angle = math.degrees(math.atan2(midx, midy)) + 180.0
    dist_px = 1.5 * math.hypot(midx, midy)
    return angle, dist_px

def sector_index_from_angle(angle_deg):
    return int((angle_deg % 360) // SECTOR_ANGLE) % NUM_SECTORS

def precompute_sector_masks(frame_shape, sector_center, num_sectors, ellipse_mask=None):
    H, W = frame_shape[:2]
    R = int(math.hypot(W, H))
    masks = []
    for s in range(num_sectors):
        start_deg = s * SECTOR_ANGLE
        end_deg   = (s + 1) * SECTOR_ANGLE
        pts = [sector_center]
        for ang in range(start_deg, end_deg + 1, 2):
            rad = math.radians(ang)
            x = int(sector_center[0] + R * math.cos(rad))
            y = int(sector_center[1] + R * math.sin(rad))
            pts.append((x, y))
        pts.append(sector_center)
        wedge = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(wedge, [np.array(pts, dtype=np.int32)], 255)
        if ellipse_mask is not None:
            wedge = cv2.bitwise_and(wedge, ellipse_mask)
        masks.append(wedge)
    return masks

def ring_sectors(seed, radius):
    if radius == 0:
        return [seed]
    return [(seed - radius) % NUM_SECTORS, (seed + radius) % NUM_SECTORS]

def find_largest_contour(bin_img, min_area=MIN_AREA_PIX):
    cnts, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    max_cnt, max_area = None, 0.0
    for c in cnts:
        a = cv2.contourArea(c)
        if a > max_area and a >= min_area:
            max_area = a; max_cnt = c
    if max_cnt is None: return None
    x,y,w,h = cv2.boundingRect(max_cnt)
    cx, cy = x + w // 2, y + h // 2
    return (x,y,w,h), (cx,cy), max_area

def clamp_vec(vx, vy, vmax):
    mag = math.hypot(vx, vy)
    if mag <= vmax or mag == 0: return vx, vy
    s = vmax / mag
    return vx * s, vy * s

# ========================
# Main
# ========================
def process_video(video_path):
    cv2.setUseOptimized(True)

    lower, upper, (xoff, yoff), json_mask = load_thresholds()

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Error: Could not open video."); return

    ret, first = cap.read()
    if not ret:
        print("Error: empty video."); cap.release(); return
    if RESIZE_TO: first = cv2.resize(first, RESIZE_TO, interpolation=cv2.INTER_AREA)
    H, W = first.shape[:2]
    fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0

    # Ellipse ROI & unified center
    ellipse_mask = None
    if ENABLE_MASK or json_mask is not None:
        ellipse_mask = np.zeros((H, W), dtype=np.uint8)
        if json_mask is not None:
            mcx, mcy, s1, s2 = json_mask
        else:
            mcx, mcy = MASK_CENTER; s1, s2 = MASK_AXES
        cv2.ellipse(ellipse_mask, (int(mcx), int(mcy)), (int(s1), int(s2)), 0, 0, 360, 255, -1)
        sector_center = (int(mcx), int(mcy))
    else:
        sector_center = (W // 2, H // 2)

    # Precompute wedges around sector_center
    sector_masks = precompute_sector_masks(first.shape, sector_center, NUM_SECTORS, ellipse_mask)

    kernel = np.ones((3,3), np.uint8)

    last_position = None
    last_sector = None
    v_ema = None  # (vx, vy) EMA

    frame_times = []
    frames_processed = 0

    print("Processing video...")

    next_frame = first
    first_frame_done = False

    while True:
        t0 = time.perf_counter()
        frame = next_frame
        if frame is None: break

        # HSV once per frame on the RAW image (no drawn overlays)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin_all = cv2.inRange(hsv, lower, upper)
        if ellipse_mask is not None:
            bin_all = cv2.bitwise_and(bin_all, ellipse_mask)
        if USE_MORPH and MORPH_ITERS > 0:
            bin_all = cv2.morphologyEx(bin_all, cv2.MORPH_OPEN, kernel, iterations=MORPH_ITERS)
            bin_all = cv2.morphologyEx(bin_all, cv2.MORPH_CLOSE, kernel, iterations=MORPH_ITERS)

        searched_sectors = []
        best = None  # (bbox, center, area, sector)
        predicted_sector = None

        # ---------- First frame: global search ----------
        if not first_frame_done:
            gcand = find_largest_contour(bin_all, MIN_AREA_PIX)
            if gcand is not None:
                bbox, center, area = gcand
                ang = (math.degrees(math.atan2(center[1]-sector_center[1], center[0]-sector_center[0])) + 360) % 360
                found_sector = sector_index_from_angle(ang)
                best = (bbox, center, area, found_sector)
            searched_sectors = list(range(NUM_SECTORS))
            first_frame_done = True

        else:
            # ---------- Predict sector (EMA + adaptive lookahead) ----------
            if last_position is not None and v_ema is not None:
                speed = math.hypot(v_ema[0], v_ema[1])
                lookahead = LOOKAHEAD_MIN + (1 if speed > LOOKAHEAD_SPEED_THRESH else 0)
                lookahead = min(LOOKAHEAD_MAX, lookahead)
                px = last_position[0] + v_ema[0] * lookahead
                py = last_position[1] + v_ema[1] * lookahead
                ang = (math.degrees(math.atan2(py - sector_center[1], px - sector_center[0])) + 360) % 360
                predicted_sector = sector_index_from_angle(ang)

            seed = predicted_sector if predicted_sector is not None else (last_sector if last_sector is not None else 0)

            # ---------- Expand from seed; STOP on first hit (if enabled) ----------
            found_any = False
            max_radius = NUM_SECTORS // 2 + 1
            for r in range(max_radius):
                for s in ring_sectors(seed, r):
                    if s in searched_sectors:  # avoid duplicates
                        continue
                    searched_sectors.append(s)

                    bin_sector = cv2.bitwise_and(bin_all, sector_masks[s])
                    cand = find_largest_contour(bin_sector, MIN_AREA_PIX)
                    if cand is None:
                        continue

                    bbox, center, area = cand
                    best = (bbox, center, area, s)
                    found_any = True

                    if STOP_ON_FIRST_HIT:
                        break  # break inner loop
                if found_any and STOP_ON_FIRST_HIT:
                    break  # break outer loop

            # If nothing found in the rings (rare), fall back to scanning all sectors
            if not found_any:
                for s in range(NUM_SECTORS):
                    if s in searched_sectors: continue
                    searched_sectors.append(s)
                    bin_sector = cv2.bitwise_and(bin_all, sector_masks[s])
                    cand = find_largest_contour(bin_sector, MIN_AREA_PIX)
                    if cand is None: continue
                    bbox, center, area = cand
                    best = (bbox, center, area, s)
                    break

        # ---------- Update state ----------
        ball_found = best is not None
        if ball_found:
            (x,y,w,h), (cx,cy), area, found_sector = best
            if last_position is not None:
                dvx = cx - last_position[0]
                dvy = cy - last_position[1]
                if v_ema is None:
                    v_ema = (dvx, dvy)
                else:
                    v_ema = (VEL_ALPHA * v_ema[0] + (1 - VEL_ALPHA) * dvx,
                             VEL_ALPHA * v_ema[1] + (1 - VEL_ALPHA) * dvy)
                v_ema = clamp_vec(v_ema[0], v_ema[1], VEL_MAX)
            else:
                v_ema = (0.0, 0.0)
            last_position = (cx, cy)
            last_sector = found_sector

            angle_deg, dist_px = angle_and_distance_cpp(cx, cy, xoff, yoff)
        else:
            if v_ema is not None:
                v_ema = (0.9 * v_ema[0], 0.9 * v_ema[1])

        # Timing & console print (EVERY frame)
        dt_ms = (time.perf_counter() - t0) * 1000.0
        frame_times.append(dt_ms)
        fps_now = 1000.0 / dt_ms if dt_ms > 0 else 0.0

        frames_processed += 1
        pred_str = f"S{predicted_sector}" if predicted_sector is not None else "SNone"
        ball_str = f"S{best[3]}" if ball_found else "SNone"
        print(f"Frame {frames_processed}: Searched sectors {searched_sectors}, Ball in {ball_str}, Predicted: {pred_str}")

        # read next frame
        ret, nf = cap.read()
        if not ret: next_frame = None
        else: next_frame = cv2.resize(nf, RESIZE_TO, interpolation=cv2.INTER_AREA) if RESIZE_TO else nf

    cap.release()

    # Stats + CSV
    if frame_times:
        avg_ms = sum(frame_times) / len(frame_times)
        min_ms = min(frame_times); max_ms = max(frame_times)
        avg_fps = 1000.0 / avg_ms if avg_ms > 0 else 0.0
        with open(CSV_PATH, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['Frame','Processing_Time_ms','FPS'])
            for i, ms in enumerate(frame_times, 1):
                w.writerow([i, ms, (1000.0/ms if ms > 0 else 0.0)])
        print("\nPerformance Stats:")
        print(f"Average processing time: {avg_ms:.2f}ms ({avg_fps:.1f} FPS)")
        print(f"Min/Max processing time: {min_ms:.2f}ms / {max_ms:.2f}ms")
        print(f"Total frames processed: {len(frame_times)}")
        print(f"Data saved to: {CSV_PATH}")

if __name__ == "__main__":
    process_video(VIDEO_PATH)