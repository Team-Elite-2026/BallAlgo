"""
Standalone LD19 visualization for bench testing.

Does not start the camera or send perception packets.
Opens the Teensy UART to get live heading; pass --no-teensy to skip.
Run from this directory on the Pi:

    python lidar_visual.py

Keys: q quit, +/- or [/] manually nudge heading.
"""

import argparse
import math
from collections import deque

import cv2
import numpy as np

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from py_lidar_bench import config as cfg
from py_lidar_bench.gpio_utils import cleanup_gpio, setup_lidar_pwm_ground
from py_lidar_bench.lidar_bench import LD19Reader, LidarLocalizer
from py_lidar_bench.teensy_reader import TeensyReader


POLAR_SIZE = 520
POLAR_RANGE_MM = 3500
FIELD_PANEL_W = 760
FIELD_PANEL_H = 520


def parse_args():
    parser = argparse.ArgumentParser(description="LD19 lidar-only visualization (no camera / Teensy).")
    parser.add_argument(
        "--heading",
        type=float,
        default=cfg.ROBOT_HEADING_DEG,
        help="Robot heading in degrees for field-frame view and localizer (default: config).",
    )
    parser.add_argument(
        "--no-localizer",
        action="store_true",
        help="Show raw scan only; skip wall-based pose estimate.",
    )
    parser.add_argument(
        "--no-gpio",
        action="store_true",
        help="Skip holding LIDAR PWM GPIO LOW (only if your setup does not need it).",
    )
    parser.add_argument(
        "--no-teensy",
        action="store_true",
        help="Skip opening the Teensy UART; heading stays at --heading value.",
    )
    return parser.parse_args()


def filter_points(points):
    return [
        p
        for p in points
        if p.distance_mm >= 80
        and p.distance_mm <= 6000
        and p.intensity >= 20
    ]


def robot_xy_mm(point):
    angle_deg = point.angle_cd * 0.01
    radians = math.radians(angle_deg)
    distance = float(point.distance_mm)
    return distance * math.cos(radians), distance * math.sin(radians)


def field_ray_endpoint(robot_x, robot_y, point, heading_deg):
    field_angle_deg = heading_deg + cfg.LIDAR_YAW_OFFSET_DEG + point.angle_cd * 0.01
    radians = math.radians(field_angle_deg)
    distance = float(point.distance_mm)
    return (
        robot_x + distance * math.cos(radians),
        robot_y + distance * math.sin(radians),
    )


def mm_to_polar_pixel(x_mm, y_mm, size, range_mm):
    scale = (size * 0.45) / range_mm
    cx = cy = size // 2
    px = int(cx + x_mm * scale)
    py = int(cy - y_mm * scale)
    return px, py


def mm_to_field_pixel(x_mm, y_mm, width_px, height_px, margin=30):
    fw = cfg.FIELD_WIDTH_MM
    fh = cfg.FIELD_HEIGHT_MM
    draw_w = width_px - 2 * margin
    draw_h = height_px - 2 * margin
    scale = min(draw_w / fw, draw_h / fh)
    px = int(margin + x_mm * scale)
    py = int(height_px - margin - y_mm * scale)
    return px, py


def draw_polar_panel(points):
    panel = np.zeros((POLAR_SIZE, POLAR_SIZE, 3), dtype=np.uint8)
    cv2.circle(panel, (POLAR_SIZE // 2, POLAR_SIZE // 2), int(POLAR_SIZE * 0.45), (40, 40, 40), 1)
    cv2.circle(panel, (POLAR_SIZE // 2, POLAR_SIZE // 2), 3, (80, 80, 255), -1)

    for point in points:
        x_mm, y_mm = robot_xy_mm(point)
        px, py = mm_to_polar_pixel(x_mm, y_mm, POLAR_SIZE, POLAR_RANGE_MM)
        if 0 <= px < POLAR_SIZE and 0 <= py < POLAR_SIZE:
            intensity = min(255, point.intensity * 8)
            cv2.circle(panel, (px, py), 2, (0, intensity, 255 - intensity // 2), -1)

    cv2.putText(panel, "Polar (robot frame)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
    cv2.putText(panel, f"{len(points)} pts", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    return panel


def draw_field_panel(points, heading_deg, pose, heading_live=False):
    panel = np.full((FIELD_PANEL_H, FIELD_PANEL_W, 3), 28, dtype=np.uint8)
    fw = cfg.FIELD_WIDTH_MM
    fh = cfg.FIELD_HEIGHT_MM

    corners = [(0, 0), (fw, 0), (fw, fh), (0, fh), (0, 0)]
    for i in range(len(corners) - 1):
        p0 = mm_to_field_pixel(corners[i][0], corners[i][1], FIELD_PANEL_W, FIELD_PANEL_H)
        p1 = mm_to_field_pixel(corners[i + 1][0], corners[i + 1][1], FIELD_PANEL_W, FIELD_PANEL_H)
        cv2.line(panel, p0, p1, (70, 70, 70), 2)

    if pose["valid"]:
        robot_x, robot_y = pose["x_mm"], pose["y_mm"]
    else:
        robot_x, robot_y = fw * 0.5, fh * 0.5

    robot_px = mm_to_field_pixel(robot_x, robot_y, FIELD_PANEL_W, FIELD_PANEL_H)
    cv2.circle(panel, robot_px, 6, (0, 255, 255), -1)

    heading_rad = math.radians(heading_deg)
    arrow_len_mm = 250.0
    tip_x = robot_x + arrow_len_mm * math.cos(heading_rad)
    tip_y = robot_y + arrow_len_mm * math.sin(heading_rad)
    tip_px = mm_to_field_pixel(tip_x, tip_y, FIELD_PANEL_W, FIELD_PANEL_H)
    cv2.arrowedLine(panel, robot_px, tip_px, (0, 200, 255), 2, tipLength=0.25)

    for point in points:
        end_x, end_y = field_ray_endpoint(robot_x, robot_y, point, heading_deg)
        hit_px = mm_to_field_pixel(end_x, end_y, FIELD_PANEL_W, FIELD_PANEL_H)
        cv2.line(panel, robot_px, hit_px, (60, 140, 60), 1)
        cv2.circle(panel, hit_px, 2, (80, 200, 80), -1)

    status = "pose OK" if pose["valid"] else "pose invalid"
    if pose["valid"]:
        cx = (pose["x_mm"] - cfg.FIELD_WIDTH_MM / 2) / 10
        cy = (pose["y_mm"] - cfg.FIELD_HEIGHT_MM / 2) / 10
        status += f"  ({cx:.1f}, {cy:.1f}) cm"
    heading_src = "teensy" if heading_live else "manual"
    heading_color = (80, 220, 80) if heading_live else (180, 180, 180)
    cv2.putText(panel, "Field frame", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
    cv2.putText(panel, f"heading {heading_deg:.1f} deg  [{heading_src}]", (10, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, heading_color, 1)
    cv2.putText(panel, status, (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 180), 1)
    return panel


def run():
    args = parse_args()
    heading_deg = args.heading
    heading_live = False
    gpio_handle = None if args.no_gpio else setup_lidar_pwm_ground()

    reader = LD19Reader(cfg.LIDAR_PORT, cfg.LIDAR_BAUD, cfg.LIDAR_TIMEOUT)
    if not reader.is_connected:
        print("[LIDAR] Could not open port; exiting.")
        if gpio_handle is not None:
            cleanup_gpio(gpio_handle)
        return

    teensy = None if args.no_teensy else TeensyReader(cfg.TEENSY_PORT, cfg.TEENSY_BAUD)

    localizer = None
    if not args.no_localizer:
        localizer = LidarLocalizer(
            field_width_mm=cfg.FIELD_WIDTH_MM,
            field_height_mm=cfg.FIELD_HEIGHT_MM,
            lidar_yaw_offset_deg=cfg.LIDAR_YAW_OFFSET_DEG,
        )

    points_window = deque(maxlen=cfg.LIDAR_POINTS_WINDOW)
    last_pose = {"valid": False, "x_mm": None, "y_mm": None}

    print("LiDAR visual running. q=quit  +/- or [/]=heading (manual override)")
    if teensy is None:
        print("[TEENSY] skipped — heading is manual-only")

    try:
        while True:
            if teensy is not None:
                live = teensy.poll()
                if live is not None:
                    heading_deg = live
                    heading_live = True

            new_points = reader.poll_points()
            if new_points:
                points_window.extend(new_points)

            points = filter_points(list(points_window))
            pose = last_pose
            if localizer is not None and points:
                result = localizer.update(points, heading_deg)
                if result["valid"]:
                    last_pose = {"valid": True, "x_mm": result["x_mm"], "y_mm": result["y_mm"]}
                else:
                    last_pose = {"valid": False, "x_mm": None, "y_mm": None}
                pose = last_pose

            polar = draw_polar_panel(points)
            field = draw_field_panel(points, heading_deg, pose, heading_live)
            combined = np.hstack([polar, field])
            cv2.imshow("LiDAR test (no pipeline)", combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (ord("+"), ord("="), ord("]")):
                heading_deg += 5.0
            if key in (ord("-"), ord("_"), ord("[")):
                heading_deg -= 5.0

    finally:
        reader.close()
        if teensy is not None:
            teensy.close()
        if gpio_handle is not None:
            cleanup_gpio(gpio_handle)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
