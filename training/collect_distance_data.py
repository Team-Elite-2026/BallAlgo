#!/usr/bin/env python3
"""Collect centered (dx, dy) ball coordinates and real ball distance.

Run this script, place the ball at a known distance, then press Enter while the
ball is detected. Type the real distance in inches; the script converts it to cm
(plus DISTANCE_ADJUSTMENT_CM) and appends a row to OUTPUT_CSV as:

    x,y,measured_cm

where ``x``/``y`` are the *centered* pixel coordinates ``cx - offset_x`` /
``cy - offset_y`` (same convention used by ``main.py`` at inference time).

This uses the production detector from ``main.py`` (``CameraDetector`` +
``PiCameraSource``) so the pixel centers match exactly what the robot sees at
runtime.
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

from main import (  # noqa: E402
    RESIZE_TO,
    CameraDetector,
    PiCameraSource,
    load_thresholds,
)
from training_data_config import DISTANCE_ADJUSTMENT_CM  # noqa: E402


# ========================
# Configure these values
# ========================
OUTPUT_CSV = Path(__file__).resolve().parent / "distance_training_data.csv"
THRESHOLDS_JSON = BALLALGO_DIR / "thresholds.json"

# Use a video path for repeatable collection, or leave as None for the camera.
VIDEO_PATH: str | None = None
CAMERA_INDEX = 0

WINDOW_NAME = "Ball distance data collection"
CM_PER_INCH = 2.54


def open_capture(thresholds):
    """Return (read_frame, close) for either a video file or the Pi camera."""
    if VIDEO_PATH is not None:
        cap = cv2.VideoCapture(VIDEO_PATH)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open video: {VIDEO_PATH}")

        def read_frame():
            ok, frame = cap.read()
            if not ok or frame is None:
                return False, None
            if RESIZE_TO is not None:
                frame = cv2.resize(frame, RESIZE_TO)
            return True, frame

        return read_frame, cap.release

    source = PiCameraSource(thresholds, size=RESIZE_TO)
    source.start()

    def read_frame():
        frame = source.capture_bgr()
        return (frame is not None), frame

    return read_frame, source.close


def append_measurement(output_csv: Path, x: float, y: float, measured_cm: float) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not output_csv.exists() or output_csv.stat().st_size == 0
    with output_csv.open("a", newline="") as handle:
        writer = csv.writer(handle)
        if needs_header:
            writer.writerow(["x", "y", "measured_cm"])
        writer.writerow([round(x, 2), round(y, 2), measured_cm])


def inches_to_cm(inches: float) -> float:
    return inches * CM_PER_INCH


def prompt_for_distance(centered: tuple[float, float]) -> float | None:
    dx, dy = centered
    raw_value = input(
        f"\nDetected centered ({dx:.1f}, {dy:.1f}). Real distance in inches, or blank to cancel: "
    ).strip()
    if not raw_value:
        print("Canceled this sample.")
        return None
    try:
        measured_inches = float(raw_value)
    except ValueError:
        print(f"Invalid distance: {raw_value!r}. Sample was not saved.")
        return None
    return inches_to_cm(measured_inches) + float(DISTANCE_ADJUSTMENT_CM)


def draw_overlay(result, offsets, saved_count: int) -> np.ndarray:
    # ``detect(annotate=True)`` already drew the ball/sectors; add status text.
    display = result.annotated_frame
    if result.ball.found and result.ball.center is not None:
        cx, cy = result.ball.center
        dx, dy = cx - offsets[0], cy - offsets[1]
        cv2.putText(
            display,
            f"centered=({dx:.1f},{dy:.1f}) dpx={result.ball.dist_px:.1f}",
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(display, "No ball detected", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    cv2.putText(display, "Enter: save sample | q: quit", (18, display.shape[0] - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    cv2.putText(display, f"Saved: {saved_count} -> {OUTPUT_CSV.name}", (18, display.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
    return display


def main() -> None:
    thresholds = load_thresholds(THRESHOLDS_JSON)
    offsets = thresholds.offsets
    detector = CameraDetector(thresholds)
    read_frame, close_capture = open_capture(thresholds)
    saved_count = 0

    print(f"Writing samples to: {OUTPUT_CSV}")
    print(f"Pixel origin (offsets): {offsets}")
    print("Press Enter in the OpenCV window to save the current detected center; q to quit.")

    try:
        while True:
            ok, frame = read_frame()
            if not ok or frame is None:
                print("No more frames available.")
                break

            result = detector.detect(frame, annotate=True)
            display = draw_overlay(result, offsets, saved_count)
            cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key in (10, 13):
                if not result.ball.found or result.ball.center is None:
                    print("\nNo ball detected right now, so no sample was saved.")
                    continue
                cx, cy = result.ball.center
                centered = (cx - offsets[0], cy - offsets[1])
                measured_cm = prompt_for_distance(centered)
                if measured_cm is None:
                    continue
                append_measurement(OUTPUT_CSV, centered[0], centered[1], measured_cm)
                saved_count += 1
                print(f"Saved row: {centered[0]},{centered[1]},{measured_cm} cm")
    finally:
        close_capture()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
