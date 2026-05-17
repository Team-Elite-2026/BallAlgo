from collections import deque
import math
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import config as cfg
from gpio_utils import cleanup_gpio, setup_lidar_pwm_ground
from lidar_processing import LD19Reader, LidarLocalizer
from serial_utils import RobotHeadingReader, open_robot_serial, safe_write
from vision_utils import (
    angle_and_distance_cpp,
    apply_ball_distance_calibration,
    clamp_vec,
    find_largest_contour,
    load_thresholds,
    precompute_sector_masks,
    ring_sectors,
    sector_index_from_angle,
)


def setup_camera():
    picam2 = Picamera2()
    cam_config = picam2.create_video_configuration(
        main={"size": cfg.RESIZE_TO if cfg.RESIZE_TO else (1536, 864), "format": "RGB888"},
        controls={"FrameRate": 120},
    )
    picam2.configure(cam_config)
    picam2.start()

    time.sleep(0.5)  # allow auto-exposure to settle
    picam2.set_controls(
        {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": 10000,  # microseconds
            "AnalogueGain": 0,
            "Brightness": 0,
            "Contrast": 1,
            "Saturation": 1,
        }
    )
    return picam2


def build_roi(first_frame, json_mask):
    height, width = first_frame.shape[:2]

    ellipse_mask = None
    if cfg.ENABLE_MASK or json_mask is not None:
        ellipse_mask = np.zeros((height, width), dtype=np.uint8)
        if json_mask is not None:
            mcx, mcy, s1, s2 = json_mask
        else:
            mcx, mcy = cfg.MASK_CENTER
            s1, s2 = cfg.MASK_AXES
        cv2.ellipse(ellipse_mask, (int(mcx), int(mcy)), (int(s1), int(s2)), 0, 0, 360, 255, -1)
        sector_center = (int(mcx), int(mcy))
    else:
        sector_center = (width // 2, height // 2)

    sector_masks = precompute_sector_masks(first_frame.shape, sector_center, ellipse_mask)
    return ellipse_mask, sector_center, sector_masks


def track_goals(frame, bin_yellow, bin_blue, xoff, yoff):
    yellow_goal = find_largest_contour(bin_yellow, cfg.MIN_AREA_PIX)
    blue_goal = find_largest_contour(bin_blue, cfg.MIN_AREA_PIX)

    blue_angle = -5
    yellow_angle = -5

    if yellow_goal is not None:
        (x, y, w, h), (cx, cy), _ = yellow_goal
        yellow_angle, _ = angle_and_distance_cpp(cx, cy, xoff, yoff)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(frame, "YELLOW GOAL", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if blue_goal is not None:
        (x, y, w, h), (cx, cy), _ = blue_goal
        blue_angle, _ = angle_and_distance_cpp(cx, cy, xoff, yoff)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, "BLUE GOAL", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    return blue_angle, yellow_angle


def track_ball(
    frame,
    bin_ball,
    sector_center,
    sector_masks,
    first_frame_done,
    last_position,
    last_sector,
    v_ema,
    xoff,
    yoff,
):
    searched_sectors = []
    best = None
    predicted_sector = None

    if not first_frame_done:
        candidate = find_largest_contour(bin_ball, cfg.MIN_AREA_PIX)
        if candidate is not None:
            bbox, center, area = candidate
            angle = (math.degrees(math.atan2(center[1] - sector_center[1], center[0] - sector_center[0])) + 360) % 360
            found_sector = sector_index_from_angle(angle)
            best = (bbox, center, area, found_sector)
        searched_sectors = list(range(cfg.NUM_SECTORS))
        first_frame_done = True
    else:
        if last_position is not None and v_ema is not None:
            speed = math.hypot(v_ema[0], v_ema[1])
            lookahead = cfg.LOOKAHEAD_MIN + (1 if speed > cfg.LOOKAHEAD_SPEED_THRESH else 0)
            lookahead = min(cfg.LOOKAHEAD_MAX, lookahead)
            px = last_position[0] + v_ema[0] * lookahead
            py = last_position[1] + v_ema[1] * lookahead
            angle = (math.degrees(math.atan2(py - sector_center[1], px - sector_center[0])) + 360) % 360
            predicted_sector = sector_index_from_angle(angle)

        seed = predicted_sector if predicted_sector is not None else (last_sector if last_sector is not None else 0)
        found_any = False
        max_radius = cfg.NUM_SECTORS // 2 + 1

        for radius in range(max_radius):
            for sector in ring_sectors(seed, radius):
                if sector in searched_sectors:
                    continue
                searched_sectors.append(sector)
                bin_sector = cv2.bitwise_and(bin_ball, sector_masks[sector])
                candidate = find_largest_contour(bin_sector, cfg.MIN_AREA_PIX)
                if candidate is None:
                    continue
                bbox, center, area = candidate
                best = (bbox, center, area, sector)
                found_any = True
                if cfg.STOP_ON_FIRST_HIT:
                    break
            if found_any and cfg.STOP_ON_FIRST_HIT:
                break

        if not found_any:
            for sector in range(cfg.NUM_SECTORS):
                if sector in searched_sectors:
                    continue
                searched_sectors.append(sector)
                bin_sector = cv2.bitwise_and(bin_ball, sector_masks[sector])
                candidate = find_largest_contour(bin_sector, cfg.MIN_AREA_PIX)
                if candidate is None:
                    continue
                bbox, center, area = candidate
                best = (bbox, center, area, sector)
                break

    ball_found = best is not None
    angle_deg = -5
    dist_px = -5

    if ball_found:
        (x, y, w, h), (cx, cy), _area, found_sector = best
        if last_position is not None:
            dvx = cx - last_position[0]
            dvy = cy - last_position[1]
            if v_ema is None:
                v_ema = (dvx, dvy)
            else:
                v_ema = (
                    cfg.VEL_ALPHA * v_ema[0] + (1 - cfg.VEL_ALPHA) * dvx,
                    cfg.VEL_ALPHA * v_ema[1] + (1 - cfg.VEL_ALPHA) * dvy,
                )
            v_ema = clamp_vec(v_ema[0], v_ema[1], cfg.VEL_MAX)
        else:
            v_ema = (0.0, 0.0)

        last_position = (cx, cy)
        last_sector = found_sector
        angle_deg, dist_px = angle_and_distance_cpp(cx, cy, xoff, yoff)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.arrowedLine(
            frame,
            (cx, cy),
            (cx + int(5 * (v_ema[0] if v_ema else 0)), cy + int(5 * (v_ema[1] if v_ema else 0))),
            (255, 0, 0),
            2,
            tipLength=0.3,
        )
        cv2.putText(
            frame,
            f"S{found_sector} | {angle_deg:.1f}\xb0 | {dist_px:.1f}px",
            (x, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    elif v_ema is not None:
        v_ema = (0.9 * v_ema[0], 0.9 * v_ema[1])

    return {
        "first_frame_done": first_frame_done,
        "last_position": last_position,
        "last_sector": last_sector,
        "v_ema": v_ema,
        "searched_sectors": searched_sectors,
        "best": best,
        "predicted_sector": predicted_sector,
        "angle_deg": angle_deg,
        "dist_px": dist_px,
    }


def draw_sector_overlay(frame, sector_center, searched_sectors, predicted_sector, best):
    height, width = frame.shape[:2]
    max_radius = int(math.hypot(width, height))

    for index in range(cfg.NUM_SECTORS):
        angle = math.radians(index * cfg.SECTOR_ANGLE)
        ex = int(sector_center[0] + max_radius * math.cos(angle))
        ey = int(sector_center[1] + max_radius * math.sin(angle))

        color = cfg.COLOR_UNSEARCHED
        if predicted_sector is not None and index == predicted_sector:
            color = cfg.COLOR_PREDICTED
        if index in searched_sectors:
            color = cfg.COLOR_SEARCHED
        if best is not None and index == best[3]:
            color = cfg.COLOR_FOUND

        cv2.line(frame, sector_center, (ex, ey), color, 3)
        lx = int(sector_center[0] + 90 * math.cos(angle))
        ly = int(sector_center[1] + 90 * math.sin(angle))
        cv2.putText(frame, str(index), (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def process_video(show_video):
    robot_serial = open_robot_serial() if cfg.ENABLE_SERIAL else None
    heading_reader = RobotHeadingReader(cfg.ROBOT_HEADING_DEG)
    gpio_handle = setup_lidar_pwm_ground()

    lidar_reader = None
    lidar_localizer = None
    lidar_points_window = deque(maxlen=60)
    last_pose = {"valid": False, "x_mm": -5, "y_mm": -5}

    if cfg.ENABLE_LIDAR:
        lidar_reader = LD19Reader(cfg.LIDAR_PORT, cfg.LIDAR_BAUD, cfg.LIDAR_TIMEOUT)
        lidar_localizer = LidarLocalizer(
            field_width_mm=cfg.FIELD_WIDTH_MM,
            field_height_mm=cfg.FIELD_HEIGHT_MM,
            lidar_yaw_offset_deg=cfg.LIDAR_YAW_OFFSET_DEG,
        )

    cv2.setUseOptimized(True)
    threshold_data = load_thresholds()
    (lower_ball, upper_ball) = threshold_data["ball"]
    (lower_yellow, upper_yellow) = threshold_data["yellow"]
    (lower_blue, upper_blue) = threshold_data["blue"]
    (xoff, yoff) = threshold_data["offsets"]
    json_mask = threshold_data["mask"]

    picam2 = setup_camera()
    first = picam2.capture_array()
    first = cv2.cvtColor(first, cv2.COLOR_RGB2BGR)

    ellipse_mask, sector_center, sector_masks = build_roi(first, json_mask)
    kernel = np.ones((3, 3), np.uint8)

    first_frame_done = False
    last_position = None
    last_sector = None
    v_ema = None
    next_frame = first

    print("Press 'q' to quit.")

    while True:
        start_time = time.perf_counter()
        frame = next_frame
        if frame is None:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin_ball = cv2.inRange(hsv, lower_ball, upper_ball)
        bin_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        bin_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        for bin_img in (bin_ball, bin_yellow, bin_blue):
            if ellipse_mask is not None:
                bin_img[:] = cv2.bitwise_and(bin_img, ellipse_mask)
            if cfg.USE_MORPH and cfg.MORPH_ITERS > 0:
                bin_img[:] = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, kernel, iterations=cfg.MORPH_ITERS)
                bin_img[:] = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=cfg.MORPH_ITERS)

        blue_angle, yellow_angle = track_goals(frame, bin_yellow, bin_blue, xoff, yoff)
        ball_data = track_ball(
            frame,
            bin_ball,
            sector_center,
            sector_masks,
            first_frame_done,
            last_position,
            last_sector,
            v_ema,
            xoff,
            yoff,
        )

        first_frame_done = ball_data["first_frame_done"]
        last_position = ball_data["last_position"]
        last_sector = ball_data["last_sector"]
        v_ema = ball_data["v_ema"]
        searched_sectors = ball_data["searched_sectors"]
        best = ball_data["best"]
        predicted_sector = ball_data["predicted_sector"]
        angle_deg = ball_data["angle_deg"]
        dist_px = ball_data["dist_px"]

        draw_sector_overlay(frame, sector_center, searched_sectors, predicted_sector, best)

        if ellipse_mask is not None:
            if json_mask is not None:
                _, _, s1, s2 = json_mask
            else:
                s1, s2 = cfg.MASK_AXES
            cv2.ellipse(frame, sector_center, (int(s1), int(s2)), 0, 0, 360, (0, 255, 255), 2)

        binmask_display = np.zeros_like(bin_ball)
        for sector in searched_sectors:
            binmask_display = cv2.bitwise_or(binmask_display, cv2.bitwise_and(bin_ball, sector_masks[sector]))

        dt_ms = (time.perf_counter() - start_time) * 1000.0
        fps_now = 1000.0 / dt_ms if dt_ms > 0 else 0.0
        cv2.putText(
            frame,
            f"Process: {dt_ms:.1f}ms | FPS: {fps_now:.1f}",
            (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ball_dist = apply_ball_distance_calibration(dist_px)

        robot_heading_deg = heading_reader.poll(robot_serial)

        if lidar_reader is not None and lidar_localizer is not None and lidar_reader.is_connected:
            new_points = lidar_reader.poll_points()
            if new_points:
                lidar_points_window.extend(new_points)
                pose = lidar_localizer.update(list(lidar_points_window), robot_heading_deg)
                if pose["valid"]:
                    last_pose = {"valid": True, "x_mm": pose["x_mm"], "y_mm": pose["y_mm"]}
                else:
                    last_pose = {"valid": False, "x_mm": -5, "y_mm": -5}

        lidar_x = int(last_pose["x_mm"]) if last_pose["valid"] else -5
        lidar_y = int(last_pose["y_mm"]) if last_pose["valid"] else -5

        send_string = (
            f"{int(angle_deg)}b{int(ball_dist)}a{int(blue_angle)}c{int(yellow_angle)}d{lidar_x}e{lidar_y}f"
        )
        safe_write(robot_serial, send_string)

        next_frame = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)

        if show_video:
            cv2.imshow("Ball Detection", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            cv2.imshow("Mask (binary)", binmask_display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    picam2.stop()
    picam2.close()
    if robot_serial is not None:
        robot_serial.close()
        print("[SERIAL] Closed")
    if lidar_reader is not None:
        lidar_reader.close()
    cleanup_gpio(gpio_handle)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    process_video(False)
