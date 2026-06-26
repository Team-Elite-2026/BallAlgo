#!/usr/bin/env python3
"""Collect x,y pixel coordinates and real ball distance measurements.

Run this script, place the ball at a known distance, then press Enter while
the ball is detected. Type the real distance in inches, and the script converts
it to cm before immediately appending the row to OUTPUT_CSV as:

    x,y,measured_cm
"""

from __future__ import annotations

import csv
from pathlib import Path
import sys

import cv2
import numpy as np

BALLALGO_DIR = Path(__file__).resolve().parents[1]
if str(BALLALGO_DIR) not in sys.path:
    sys.path.insert(0, str(BALLALGO_DIR))

from tools.legacy_main import (
    LegacyBallTracker,
    LegacyVisionResult,
    RESIZE_TO,
    load_thresholds,
    open_legacy_capture,
)
from training_data_config import DISTANCE_ADJUSTMENT_CM


# ========================
# Configure these values
# ========================
OUTPUT_CSV = Path(__file__).resolve().parent / "distance_training_data.csv"
THRESHOLDS_JSON = Path(__file__).resolve().parents[1] / "thresholds.json"

# Use a video path for repeatable collection, or leave as None for a camera.
VIDEO_PATH: str | None = None
CAMERA_INDEX = 0

# Camera backend selection:
# - True: force Picamera2
# - False: force OpenCV VideoCapture
# - None: prefer Picamera2 when available, otherwise fall back to OpenCV
USE_PICAMERA2: bool | None = None

WINDOW_NAME = "Ball distance data collection"
CM_PER_INCH = 2.54


def open_capture():
    return open_legacy_capture(
        video_path=VIDEO_PATH,
        camera_index=CAMERA_INDEX,
        use_picamera2=USE_PICAMERA2,
        resize_to=RESIZE_TO,
        frame_rate=30,
    )


def append_measurement(output_csv: Path, x: int, y: int, measured_cm: float) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not output_csv.exists() or output_csv.stat().st_size == 0

    with output_csv.open("a", newline="") as handle:
        writer = csv.writer(handle)
        if needs_header:
            writer.writerow(["x", "y", "measured_cm"])
        writer.writerow([x, y, measured_cm])


def inches_to_cm(inches: float) -> float:
    return inches * CM_PER_INCH


def prompt_for_distance(center: tuple[int, int]) -> float | None:
    x, y = center
    raw_value = input(f"\nDetected center ({x}, {y}). Real distance in inches, or blank to cancel: ").strip()
    if not raw_value:
        print("Canceled this sample.")
        return None

    try:
        measured_inches = float(raw_value)
    except ValueError:
        print(f"Invalid distance: {raw_value!r}. Sample was not saved.")
        return None

    return inches_to_cm(measured_inches) + float(DISTANCE_ADJUSTMENT_CM)


def draw_overlay(frame: np.ndarray, result: LegacyVisionResult, saved_count: int) -> np.ndarray:
    display = frame.copy()
    center = result.ball.center if result.ball.found else None

    if center is not None:
        x, y = center
        cv2.circle(display, (x, y), 7, (0, 255, 0), 2)
        cv2.drawMarker(display, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            display,
            f"x={x} y={y} a={result.ball.angle_deg:.1f} dpx={result.ball.dist_px:.1f}",
            (x + 12, max(24, y - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(display, "No ball detected", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    cv2.putText(display, "Enter: save sample | q: quit", (18, display.shape[0] - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    cv2.putText(display, f"Saved: {saved_count} -> {OUTPUT_CSV}", (18, display.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
    return display


def main() -> None:
    thresholds = load_thresholds(THRESHOLDS_JSON)
    tracker = LegacyBallTracker(thresholds)
    read_frame, close_capture = open_capture()
    saved_count = 0

    print(f"Writing samples to: {OUTPUT_CSV}")
    print("Press Enter in the OpenCV window to save the current detected center.")
    print("Press q to quit.")

    try:
        while True:
            ok, frame = read_frame()
            if not ok or frame is None:
                print("No more frames available.")
                break

            result = tracker.track_frame(frame)
            center = result.ball.center if result.ball.found else None
            display = draw_overlay(frame, result, saved_count)
            cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (10, 13):
                if center is None:
                    print("\nNo ball detected right now, so no sample was saved.")
                    continue
                measured_cm = prompt_for_distance(center)
                if measured_cm is None:
                    continue
                training_center = tracker.to_training_coordinates(center)
                append_measurement(OUTPUT_CSV, training_center[0], training_center[1], measured_cm)
                saved_count += 1
                print(f"Saved row: {training_center[0]},{training_center[1]},{measured_cm} cm")
    finally:
        close_capture()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
