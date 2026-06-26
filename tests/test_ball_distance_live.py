#!/usr/bin/env python3
"""Live robot ball distance test.

Runs the same camera detector and ONNX distance estimator used by ``main.py``.
Use this on the Raspberry Pi/robot to verify ball detection and distance output.

Examples:
    python3 tests/test_ball_distance_live.py
    python3 tests/test_ball_distance_live.py --dry-run --centered-x 0 --centered-y 100
    python3 tests/test_ball_distance_live.py --model training/run_cleaned_center_330_318_polar_2000/exported/ball_distance_polar_epoch_1800.onnx
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path


BALLALGO_DIR = Path(__file__).resolve().parents[1]
if str(BALLALGO_DIR) not in sys.path:
    sys.path.insert(0, str(BALLALGO_DIR))

from main import (  # noqa: E402
    LOST_SENTINEL,
    RESIZE_TO,
    CameraDetector,
    DistanceEstimator,
    PiCameraSource,
    ThresholdSet,
    angle_and_distance_cpp,
    load_params,
    load_thresholds,
)


DEFAULT_MODEL = Path("training/run_cleaned_center_330_318_polar_2000/exported/ball_distance_polar_epoch_1800.onnx")


def resolve_model_path(model_path: str | Path | None) -> Path:
    if model_path is None:
        params_path = load_params().get("ball_distance_model_path")
        candidate = Path(params_path) if params_path else DEFAULT_MODEL
    else:
        candidate = Path(model_path)
    if not candidate.is_absolute():
        candidate = BALLALGO_DIR / candidate
    return candidate


def model_feature_mode(model_path: Path) -> str:
    try:
        import onnxruntime as ort
    except ModuleNotFoundError:
        sys.exit("onnxruntime is not installed. On the robot, install it in the active env.")

    session = ort.InferenceSession(str(model_path))
    return session.get_modelmeta().custom_metadata_map.get("feature_mode", "xy_radius")


def build_estimator(args: argparse.Namespace, params: dict) -> tuple[DistanceEstimator, str]:
    """Build a DistanceEstimator for the requested (or params.json) mode and a
    short human-readable label for the on-screen overlay."""
    mode = args.mode or params.get("ball_distance_mode")
    if mode is None:
        mode = "onnx" if params.get("use_ball_distance_onnx_model") else "exponential"
    params = dict(params)
    params["ball_distance_mode"] = mode
    label = mode
    if mode == "onnx":
        model_path = resolve_model_path(args.model)
        if not model_path.exists():
            sys.exit(f"ONNX model not found: {model_path}")
        feature_mode = model_feature_mode(model_path)
        params["ball_distance_model_path"] = str(model_path)
        label = f"onnx:{model_path.name} ({feature_mode})"
    estimator = DistanceEstimator(params=params, repo_root=BALLALGO_DIR)
    return estimator, label


def dry_run(args: argparse.Namespace, thresholds: ThresholdSet, estimator: DistanceEstimator) -> None:
    if args.centered_x is not None and args.centered_y is not None:
        centered_x = float(args.centered_x)
        centered_y = float(args.centered_y)
        raw_x = int(round(centered_x + thresholds.offsets[0]))
        raw_y = int(round(centered_y + thresholds.offsets[1]))
    elif args.raw_x is not None and args.raw_y is not None:
        raw_x = int(round(args.raw_x))
        raw_y = int(round(args.raw_y))
        centered_x = float(raw_x - thresholds.offsets[0])
        centered_y = float(raw_y - thresholds.offsets[1])
    else:
        sys.exit("--dry-run needs either --centered-x/--centered-y or --raw-x/--raw-y")

    _angle, dist_px = angle_and_distance_cpp(raw_x, raw_y, *thresholds.offsets)
    distance_cm = estimator.estimate_cm(raw_x, raw_y, dist_px, thresholds)
    print(f"raw=({raw_x}, {raw_y})")
    print(f"centered=({centered_x:.1f}, {centered_y:.1f}) radius={math.hypot(centered_x, centered_y):.1f}px")
    print(f"distance={distance_cm:.2f} cm ({distance_cm / 2.54:.2f} in)")


def draw_status_overlay(
    display,
    result,
    thresholds: ThresholdSet,
    mode_label: str,
) -> None:
    import cv2

    cv2.drawMarker(display, thresholds.offsets, (255, 0, 0), cv2.MARKER_CROSS, 18, 2)
    if result.ball.found:
        cx, cy = result.ball.center or (0, 0)
        dx = cx - thresholds.offsets[0]
        dy = cy - thresholds.offsets[1]
        lines = [
            f"BALL FOUND  raw=({cx},{cy}) centered=({dx},{dy})",
            f"distance={result.ball.distance_cm:.1f} cm  {result.ball.distance_cm / 2.54:.1f} in  radius={result.ball.dist_px:.1f}px",
            f"angle={result.ball.angle_deg:.1f} deg  sector={result.ball.sector}",
        ]
        color = (0, 255, 0)
    else:
        lines = [
            "NO BALL DETECTED",
            "Check ball color/lighting, HSV thresholds, and that the ball is inside the yellow ROI.",
        ]
        color = (0, 0, 255)

    lines.extend([
        f"center={thresholds.offsets}",
        f"distance_mode={mode_label}",
    ])
    y = 28
    for line in lines:
        cv2.putText(display, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
        y += 24


def draw_raw_feed_overlay(display, thresholds: ThresholdSet) -> None:
    import cv2

    frame_min = int(display.min())
    frame_max = int(display.max())
    frame_mean = float(display.mean())
    cv2.drawMarker(display, thresholds.offsets, (255, 0, 0), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(
        display,
        "RAW CAMERA FEED",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        f"center={thresholds.offsets}  min/mean/max={frame_min}/{frame_mean:.1f}/{frame_max}",
        (10, 54),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def run_live(
    args: argparse.Namespace,
    thresholds: ThresholdSet,
    estimator: DistanceEstimator,
    mode_label: str,
) -> None:
    try:
        import cv2
    except ModuleNotFoundError:
        sys.exit("OpenCV is not installed. Install opencv-python or python3-opencv on the robot.")

    detector = CameraDetector(thresholds=thresholds, distance_estimator=estimator)
    camera = PiCameraSource(thresholds=thresholds, size=tuple(args.size))
    camera.start()
    print("Camera started. Press q in the OpenCV window to quit.")
    print("raw_x,raw_y centered_x,centered_y radius_px distance_cm distance_in angle_deg process_ms")

    frame_count = 0
    last_print_s = 0.0
    try:
        while True:
            frame = camera.capture_bgr()
            result = detector.detect(frame, annotate=not args.no_display)
            frame_count += 1

            now = time.monotonic()
            if result.ball.found and (now - last_print_s) >= args.print_interval:
                cx, cy = result.ball.center or (0, 0)
                dx = cx - thresholds.offsets[0]
                dy = cy - thresholds.offsets[1]
                print(
                    f"{cx:4d},{cy:4d} "
                    f"{dx:7.1f},{dy:7.1f} "
                    f"{result.ball.dist_px:8.1f} "
                    f"{result.ball.distance_cm:8.2f} "
                    f"{result.ball.distance_cm / 2.54:8.2f} "
                    f"{result.ball.angle_deg:8.1f} "
                    f"{result.process_ms:8.2f}"
                )
                last_print_s = now
            elif not result.ball.found and args.print_missing and (now - last_print_s) >= args.print_interval:
                print(
                    f"No ball detected ({result.process_ms:.2f} ms) "
                    f"frame min/mean/max={int(frame.min())}/{float(frame.mean()):.1f}/{int(frame.max())}"
                )
                last_print_s = now

            if not args.no_display:
                raw_display = frame.copy()
                draw_raw_feed_overlay(raw_display, thresholds)
                display = result.annotated_frame if result.annotated_frame is not None else frame
                draw_status_overlay(display, result, thresholds, mode_label)
                cv2.imshow("Ball distance raw feed", raw_display)
                cv2.imshow("Ball distance annotated", display)
                if result.mask_display is not None:
                    cv2.imshow("Ball distance mask", result.mask_display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            elif args.frames is not None and frame_count >= args.frames:
                break
    finally:
        camera.close()
        if not args.no_display:
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("parametric", "onnx", "exponential"),
        default=None,
        help="Distance estimation mode. Defaults to params.json 'ball_distance_mode'.",
    )
    parser.add_argument("--model", type=Path, default=None, help="ONNX model path (onnx mode). Defaults to params.json model path.")
    parser.add_argument("--dry-run", action="store_true", help="Do not open camera; run one coordinate through the model.")
    parser.add_argument("--raw-x", type=float, help="Raw frame x for --dry-run.")
    parser.add_argument("--raw-y", type=float, help="Raw frame y for --dry-run.")
    parser.add_argument("--centered-x", type=float, help="Centered x for --dry-run.")
    parser.add_argument("--centered-y", type=float, help="Centered y for --dry-run.")
    parser.add_argument("--size", type=int, nargs=2, default=RESIZE_TO, metavar=("W", "H"), help="Camera/detection size.")
    parser.add_argument("--no-display", action="store_true", help="Run without OpenCV window; useful over SSH.")
    parser.add_argument("--frames", type=int, default=None, help="Stop after this many frames in --no-display mode.")
    parser.add_argument("--print-interval", type=float, default=0.15, help="Minimum seconds between console prints.")
    parser.add_argument("--print-missing", action="store_true", help="Print periodic lines when no ball is detected.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    thresholds = load_thresholds(BALLALGO_DIR / "thresholds.json")
    params = dict(load_params())
    print(f"Thresholds: {BALLALGO_DIR / 'thresholds.json'}")
    print(f"Camera center offsets: {thresholds.offsets}")

    estimator, mode_label = build_estimator(args, params)
    print(f"Distance mode: {mode_label}")

    if args.dry_run:
        dry_run(args, thresholds, estimator)
    else:
        run_live(args, thresholds, estimator, mode_label)


if __name__ == "__main__":
    main()
