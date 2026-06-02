#!/usr/bin/env python3
"""Record LD19-derived robot position over time to a CSV file.

This uses the same point history and wall-based pose localizer as
``lidar_visual.py``. A row is saved each time fresh lidar points arrive, so
the resulting CSV can be used to graph position versus time.

Run from this directory on the Pi:

    python lidar_position_logger.py

Press Ctrl-C to stop recording.
"""

from __future__ import annotations

import argparse
import csv
from collections import deque
from datetime import datetime, timezone
import glob
from pathlib import Path
import sys
import time


DEPLOYMENT_DIR = Path(__file__).resolve().parents[1] / "deployment"
if str(DEPLOYMENT_DIR) not in sys.path:
    sys.path.insert(0, str(DEPLOYMENT_DIR))

import config as cfg
from gpio_utils import cleanup_gpio, setup_lidar_pwm_ground


CSV_FIELDS = [
    "tick",
    "time_s",
    "timestamp_utc",
    "valid",
    "x_cm",
    "y_cm",
    "abs_x_mm",
    "abs_y_mm",
    "raw_x_cm",
    "raw_y_cm",
    "raw_x_mm",
    "raw_y_mm",
    "heading_deg",
    "new_point_count",
    "filtered_point_count",
    "pose_source",
    "pose_held",
    "pose_alpha",
    "pose_inliers",
    "pose_x_inliers",
    "pose_y_inliers",
    "pose_residual_mm",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record lidar pose values to CSV for position-versus-time graphs.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="CSV output path (default: timestamped file beside this script).",
    )
    parser.add_argument(
        "--heading",
        type=float,
        default=cfg.ROBOT_HEADING_DEG,
        help="Robot heading in degrees supplied to the localizer (default: config).",
    )
    parser.add_argument(
        "--port",
        default=None,
        help="LiDAR serial device. If omitted, try config plus common Pi/USB serial ports.",
    )
    parser.add_argument(
        "--history-points",
        type=int,
        default=cfg.LIDAR_HISTORY_POINTS,
        help="Number of recent lidar points used for localization (default: config).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds; otherwise run until Ctrl-C.",
    )
    parser.add_argument(
        "--no-gpio",
        action="store_true",
        help="Skip holding LIDAR PWM GPIO LOW (only if your setup does not need it).",
    )
    parser.add_argument(
        "--print-pose",
        action="store_true",
        help="Print each recorded position row to the terminal.",
    )
    return parser.parse_args()


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / f"lidar_position_data_{stamp}.csv"


def lidar_candidate_ports(requested_port: str | None) -> list[str]:
    candidates = []
    if requested_port:
        candidates.append(requested_port)
    candidates.append(cfg.LIDAR_PORT)

    for pattern in (
        "/dev/serial/by-id/*",
        "/dev/serial0",
        "/dev/ttyAMA*",
        "/dev/ttyS*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    ):
        candidates.extend(sorted(glob.glob(pattern)))

    unique_candidates = []
    for port in candidates:
        if port not in unique_candidates:
            unique_candidates.append(port)
    return unique_candidates


def open_lidar_reader(requested_port: str | None, reader_class):
    for port in lidar_candidate_ports(requested_port):
        reader = reader_class(port, cfg.LIDAR_BAUD, cfg.LIDAR_TIMEOUT)
        if reader.is_connected:
            return reader
    return None


def filter_points(points: list) -> list:
    return [
        point
        for point in points
        if point.distance_mm >= 80
        and point.distance_mm <= 6000
        and point.intensity >= 20
    ]


def pose_row(
    tick: int,
    start_time: float,
    heading_deg: float,
    new_point_count: int,
    filtered_point_count: int,
    pose: dict,
) -> dict:
    return {
        "tick": tick,
        "time_s": f"{time.monotonic() - start_time:.6f}",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "valid": pose["valid"],
        "x_cm": pose.get("x_cm"),
        "y_cm": pose.get("y_cm"),
        "abs_x_mm": pose.get("abs_x_mm"),
        "abs_y_mm": pose.get("abs_y_mm"),
        "raw_x_cm": pose.get("raw_x_cm"),
        "raw_y_cm": pose.get("raw_y_cm"),
        "raw_x_mm": pose.get("raw_x_mm"),
        "raw_y_mm": pose.get("raw_y_mm"),
        "heading_deg": heading_deg,
        "new_point_count": new_point_count,
        "filtered_point_count": filtered_point_count,
        "pose_source": pose.get("pose_source"),
        "pose_held": pose.get("pose_held"),
        "pose_alpha": pose.get("pose_alpha"),
        "pose_inliers": pose.get("pose_inliers"),
        "pose_x_inliers": pose.get("pose_x_inliers"),
        "pose_y_inliers": pose.get("pose_y_inliers"),
        "pose_residual_mm": pose.get("pose_residual_mm"),
    }


def record_positions(args: argparse.Namespace) -> None:
    try:
        from lidar_processing import LD19Reader, LidarLocalizer
    except ModuleNotFoundError as exc:
        if exc.name == "serial":
            print("[LIDAR] PySerial is required to record data. Install it with: python -m pip install pyserial")
            return
        raise

    output_path = args.output if args.output is not None else default_output_path()
    gpio_handle = None if args.no_gpio else setup_lidar_pwm_ground()
    reader = open_lidar_reader(args.port, LD19Reader)
    if reader is None:
        print("[LIDAR] Could not open port; exiting.")
        if gpio_handle is not None:
            cleanup_gpio(gpio_handle)
        return

    localizer = LidarLocalizer(
        field_width_mm=cfg.FIELD_WIDTH_MM,
        field_height_mm=cfg.FIELD_HEIGHT_MM,
        lidar_yaw_offset_deg=cfg.LIDAR_YAW_OFFSET_DEG,
    )
    points_window = deque(maxlen=max(12, args.history_points))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    start_time = time.monotonic()
    tick = 0

    print(f"Recording lidar positions to: {output_path}")
    print("Coordinates are field-centered centimeters: x=right, y=forward.")
    print("Press Ctrl-C to stop.")

    try:
        with output_path.open("w", newline="") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            output_file.flush()

            while args.duration is None or time.monotonic() - start_time < args.duration:
                new_points = reader.poll_points()
                if not new_points:
                    continue

                points_window.extend(new_points)
                filtered_points = filter_points(list(points_window))
                pose = localizer.update(filtered_points, args.heading)
                tick += 1
                row = pose_row(
                    tick=tick,
                    start_time=start_time,
                    heading_deg=args.heading,
                    new_point_count=len(new_points),
                    filtered_point_count=len(filtered_points),
                    pose=pose,
                )
                writer.writerow(row)
                output_file.flush()

                if args.print_pose:
                    print(
                        f"[POSE] tick={tick} time={row['time_s']}s valid={row['valid']} "
                        f"x={row['x_cm']}cm y={row['y_cm']}cm"
                    )
    except KeyboardInterrupt:
        print("\nRecording stopped.")
    finally:
        reader.close()
        if gpio_handle is not None:
            cleanup_gpio(gpio_handle)

    print(f"Saved {tick} lidar position samples to: {output_path}")


def main() -> None:
    record_positions(parse_args())


if __name__ == "__main__":
    main()
