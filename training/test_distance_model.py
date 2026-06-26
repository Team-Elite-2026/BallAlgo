#!/usr/bin/env python3
"""Interactive test for the exported ONNX ball distance model.

Usage:
    python3 test_distance_model.py
    python3 test_distance_model.py --raw-x 450 --raw-y 380   # single shot from raw pixel coords
    python3 test_distance_model.py --centered-x 100 --centered-y -30  # already centered

Camera center (PIXEL_ORIGIN) is read from thresholds.json offsets.
Enter raw pixel coords (as seen on screen) and the script subtracts the
camera center before querying the model.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parent
BALLALGO_DIR = TRAINING_DIR.parent
THRESHOLDS_JSON = BALLALGO_DIR / "thresholds.json"
ONNX_MODEL = TRAINING_DIR / "exported" / "ball_distance_latest.onnx"

if str(TRAINING_DIR) not in sys.path:
    sys.path.insert(0, str(TRAINING_DIR))


def load_camera_center() -> tuple[int, int]:
    data = json.loads(THRESHOLDS_JSON.read_text())
    off = data["offsets"]
    return off["x"], off["y"]


def predict_onnx(cx: float, cy: float) -> float:
    try:
        import onnxruntime as ort
    except ModuleNotFoundError:
        sys.exit("onnxruntime not installed. Run: pip install onnxruntime")

    radius = float(np.hypot(cx, cy))
    features = np.array([[cx, cy, radius]], dtype=np.float32)
    sess = ort.InferenceSession(str(ONNX_MODEL))
    input_name = sess.get_inputs()[0].name
    result = sess.run(None, {input_name: features})
    return float(result[0].flat[0])


def run_interactive(cam_x: int, cam_y: int) -> None:
    print(f"Camera center from thresholds.json: ({cam_x}, {cam_y})")
    print(f"Model: {ONNX_MODEL}")
    print("Enter raw pixel coords (x y) as seen in the frame, or 'q' to quit.")
    print()

    while True:
        try:
            line = input("raw x y > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if line.lower() in ("q", "quit", "exit"):
            break
        if not line:
            continue

        parts = line.split()
        if len(parts) != 2:
            print("  Enter two numbers: raw_x raw_y")
            continue
        try:
            raw_x, raw_y = float(parts[0]), float(parts[1])
        except ValueError:
            print("  Invalid numbers.")
            continue

        cx = raw_x - cam_x
        cy = raw_y - cam_y
        dist = predict_onnx(cx, cy)
        print(f"  centered=({cx:.1f}, {cy:.1f})  →  distance = {dist:.1f} cm  ({dist/2.54:.1f} in)")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--raw-x", type=float, help="Raw pixel x (will subtract camera center)")
    p.add_argument("--raw-y", type=float, help="Raw pixel y (will subtract camera center)")
    p.add_argument("--centered-x", type=float, help="Already-centered x (skip camera center subtraction)")
    p.add_argument("--centered-y", type=float, help="Already-centered y (skip camera center subtraction)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cam_x, cam_y = load_camera_center()

    if args.centered_x is not None and args.centered_y is not None:
        dist = predict_onnx(args.centered_x, args.centered_y)
        print(f"centered=({args.centered_x}, {args.centered_y})  →  {dist:.1f} cm  ({dist/2.54:.1f} in)")
    elif args.raw_x is not None and args.raw_y is not None:
        cx = args.raw_x - cam_x
        cy = args.raw_y - cam_y
        dist = predict_onnx(cx, cy)
        print(f"raw=({args.raw_x}, {args.raw_y})  centered=({cx:.1f}, {cy:.1f})  →  {dist:.1f} cm  ({dist/2.54:.1f} in)")
    else:
        run_interactive(cam_x, cam_y)


if __name__ == "__main__":
    main()
