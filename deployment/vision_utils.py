import json
import math
import os

import cv2
import numpy as np

import config as cfg


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def load_thresholds():
    def parse_color(obj):
        if isinstance(obj, dict):
            h = obj.get("H", obj.get("h"))
            s = obj.get("S", obj.get("s"))
            v = obj.get("V", obj.get("v"))
            if h is None or s is None or v is None:
                raise ValueError("HSV dict missing h/s/v keys")
            return (clamp(int(h), 0, 179), clamp(int(s), 0, 255), clamp(int(v), 0, 255))
        if isinstance(obj, (list, tuple)) and len(obj) == 3:
            h, s, v = map(int, obj)
            return (clamp(h, 0, 179), clamp(s, 0, 255), clamp(v, 0, 255))
        raise ValueError("Color threshold format not recognized")

    if cfg.USE_JSON_THRESHOLDS and os.path.exists(cfg.THRESHOLDS_JSON):
        with open(cfg.THRESHOLDS_JSON, "r", encoding="utf-8") as threshold_file:
            threshold_data = json.load(threshold_file)

        lower_ball = parse_color(threshold_data["ball"]["lower"])
        upper_ball = parse_color(threshold_data["ball"]["upper"])
        lower_yellow = parse_color(threshold_data["yellowGoal"]["lower"])
        upper_yellow = parse_color(threshold_data["yellowGoal"]["upper"])
        lower_blue = parse_color(threshold_data["blueGoal"]["lower"])
        upper_blue = parse_color(threshold_data["blueGoal"]["upper"])
        xoffset = int(threshold_data.get("offsets", {}).get("x", 0))
        yoffset = int(threshold_data.get("offsets", {}).get("y", 0))

        mask_tuple = None
        if "mask1" in threshold_data:
            cx = int(threshold_data["mask1"]["x"])
            cy = int(threshold_data["mask1"]["y"])
            s1 = int(threshold_data["mask1"]["size1"])
            s2 = int(threshold_data["mask1"]["size2"])
            mask_tuple = (cx, cy, s1, s2)

        return {
            "ball": (lower_ball, upper_ball),
            "yellow": (lower_yellow, upper_yellow),
            "blue": (lower_blue, upper_blue),
            "offsets": (xoffset, yoffset),
            "mask": mask_tuple,
        }

    return {
        "ball": (cfg.FALLBACK_LOWER, cfg.FALLBACK_UPPER),
        "yellow": (cfg.FALLBACK_LOWER, cfg.FALLBACK_UPPER),
        "blue": (cfg.FALLBACK_LOWER, cfg.FALLBACK_UPPER),
        "offsets": (0, 0),
        "mask": None,
    }


def angle_and_distance_cpp(cx, cy, xoffset=0, yoffset=0):
    midx = cx - xoffset
    midy = cy - yoffset
    angle = math.degrees(math.atan2(midx, midy)) + 180
    dist_px = math.hypot(midx, midy)
    return angle, dist_px


def sector_index_from_angle(angle_deg):
    return int((angle_deg % 360) // cfg.SECTOR_ANGLE) % cfg.NUM_SECTORS


def precompute_sector_masks(frame_shape, sector_center, ellipse_mask=None):
    height, width = frame_shape[:2]
    radius = int(math.hypot(width, height))
    masks = []
    for sector in range(cfg.NUM_SECTORS):
        start_deg = sector * cfg.SECTOR_ANGLE
        end_deg = (sector + 1) * cfg.SECTOR_ANGLE
        points = [sector_center]
        for angle in range(start_deg, end_deg + 1, 2):
            rad = math.radians(angle)
            x = int(sector_center[0] + radius * math.cos(rad))
            y = int(sector_center[1] + radius * math.sin(rad))
            points.append((x, y))
        points.append(sector_center)
        wedge = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(wedge, [np.array(points, dtype=np.int32)], 255)
        if ellipse_mask is not None:
            wedge = cv2.bitwise_and(wedge, ellipse_mask)
        masks.append(wedge)
    return masks


def ring_sectors(seed, radius):
    if radius == 0:
        return [seed]
    return [(seed - radius) % cfg.NUM_SECTORS, (seed + radius) % cfg.NUM_SECTORS]


def apply_ball_distance_calibration(ball_dist):
    if ball_dist == -5:
        return -5
    return 0.00255386 * math.pow(ball_dist, 2.09612)


def find_largest_contour(bin_img, min_area=cfg.MIN_AREA_PIX):
    contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    max_cnt, max_area = None, 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area and area >= min_area:
            max_area = area
            max_cnt = contour
    if max_cnt is None:
        return None
    x, y, w, h = cv2.boundingRect(max_cnt)
    cx, cy = x + w // 2, y + h // 2
    return (x, y, w, h), (cx, cy), max_area


def clamp_vec(vx, vy, vmax):
    mag = math.hypot(vx, vy)
    if mag <= vmax or mag == 0:
        return vx, vy
    scale = vmax / mag
    return vx * scale, vy * scale
