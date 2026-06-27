from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2
except ModuleNotFoundError:  # pragma: no cover - Pi/OpenCV runtime dependency
    cv2 = None

try:
    from picamera2 import Picamera2
except ModuleNotFoundError:  # pragma: no cover - Pi runtime dependency
    Picamera2 = None

from communication import SerialLink, TeensyTelemetry, format_detection_packet
from estimation import BallKalman, PoseEstimate, PoseKalman
from estimation.kalman import polar_to_body_xy
from foxglove_runtime import FoxgloveRuntimePublisher


# ========================
# Branch-correct config
# ========================
SERIAL_PORT = "/dev/ttyAMA2"
SERIAL_BAUD = 2000000
SERIAL_TIMEOUT = 0.01
ENABLE_SERIAL = True

THRESHOLDS_JSON = "thresholds.json"
PARAMS_JSON = "params.json"
RESIZE_TO = (655, 600)
CAMERA_FPS = 120

FALLBACK_LOWER = (5, 120, 120)
FALLBACK_UPPER = (25, 255, 255)
ENABLE_MASK = True
MASK_CENTER = (330, 300)
MASK_AXES = (280, 230)

NUM_SECTORS = 12
SECTOR_ANGLE = 360 // NUM_SECTORS
USE_MORPH = True
MORPH_ITERS = 1
MIN_AREA_PIX = 5

VEL_ALPHA = 0.7
VEL_MAX = 40.0
LOOKAHEAD_MIN = 1
LOOKAHEAD_MAX = 3
LOOKAHEAD_SPEED_THRESH = 12.0
STOP_ON_FIRST_HIT = True

COLOR_UNSEARCHED = (0, 0, 255)
COLOR_SEARCHED = (0, 165, 255)
COLOR_PREDICTED = (128, 0, 128)
COLOR_FOUND = (0, 255, 0)

LOST_SENTINEL = -5
GOAL_CERT_REF_AREA = 1200.0
FIELD_WIDTH_MM = 1820.0
FIELD_HEIGHT_MM = 2430.0
BLUE_GOAL_X_MM = FIELD_WIDTH_MM * 0.5
BLUE_GOAL_Y_MM = 0.0
YELLOW_GOAL_X_MM = FIELD_WIDTH_MM * 0.5
YELLOW_GOAL_Y_MM = FIELD_HEIGHT_MM


@dataclass(slots=True)
class ThresholdSet:
    ball_lower: tuple[int, int, int]
    ball_upper: tuple[int, int, int]
    yellow_lower: tuple[int, int, int]
    yellow_upper: tuple[int, int, int]
    blue_lower: tuple[int, int, int]
    blue_upper: tuple[int, int, int]
    offsets: tuple[int, int]
    mask: tuple[int, int, int, int] | None
    raw: dict[str, Any]


@dataclass(slots=True)
class BlobDetection:
    found: bool = False
    bbox: tuple[int, int, int, int] | None = None
    center: tuple[int, int] | None = None
    area: float = 0.0
    angle_deg: float = LOST_SENTINEL
    dist_px: float = LOST_SENTINEL
    distance_cm: float = LOST_SENTINEL
    certainty: float = 0.0
    sector: int | None = None


@dataclass(slots=True)
class FrameDetections:
    ball: BlobDetection
    blue_goal: BlobDetection
    yellow_goal: BlobDetection
    searched_sectors: list[int]
    predicted_sector: int | None
    process_ms: float
    annotated_frame: Any = None
    mask_display: Any = None

    @property
    def ball_angle_deg(self) -> float:
        return self.ball.angle_deg if self.ball.found else LOST_SENTINEL

    @property
    def ball_distance_cm(self) -> float:
        return self.ball.distance_cm if self.ball.found else LOST_SENTINEL

    @property
    def blue_goal_angle_deg(self) -> float:
        return self.blue_goal.angle_deg if self.blue_goal.found else LOST_SENTINEL

    @property
    def yellow_goal_angle_deg(self) -> float:
        return self.yellow_goal.angle_deg if self.yellow_goal.found else LOST_SENTINEL

    def teensy_packet(self) -> str:
        return format_detection_packet(
            self.ball_angle_deg,
            self.ball_distance_cm,
            self.blue_goal_angle_deg,
            self.yellow_goal_angle_deg,
        )

    def snapshot_dict(self) -> dict[str, Any]:
        ball_px = {
            "found": self.ball.found,
            "cx": self.ball.center[0] if self.ball.center else 0,
            "cy": self.ball.center[1] if self.ball.center else 0,
        }
        return {
            "ball_found": self.ball.found,
            "ball_angle_deg": self.ball_angle_deg,
            "ball_distance_cm": self.ball_distance_cm,
            "ball_dist_px": self.ball.dist_px,
            "blue_goal_found": self.blue_goal.found,
            "blue_goal_angle_deg": self.blue_goal_angle_deg,
            "blue_goal_certainty": self.blue_goal.certainty,
            "yellow_goal_found": self.yellow_goal.found,
            "yellow_goal_angle_deg": self.yellow_goal_angle_deg,
            "yellow_goal_certainty": self.yellow_goal.certainty,
            "searched_sectors": self.searched_sectors,
            "predicted_sector": self.predicted_sector,
            "process_ms": self.process_ms,
            "ball_px": ball_px,
        }


@dataclass(slots=True)
class OrbitDerivativeTracker:
    prev_ball_angle_deg: float = LOST_SENTINEL
    prev_ball_distance_cm: float = LOST_SENTINEL

    def update(self, ball_angle_deg: float, ball_distance_cm: float) -> float:
        derivative = compute_orbit_derivative(
            ball_angle_deg,
            ball_distance_cm,
            self.prev_ball_angle_deg,
            self.prev_ball_distance_cm,
        )
        if ball_angle_deg != LOST_SENTINEL and ball_distance_cm != LOST_SENTINEL:
            self.prev_ball_angle_deg = ball_angle_deg
            self.prev_ball_distance_cm = ball_distance_cm
        return derivative


def require_cv2() -> None:
    if cv2 is None:
        raise RuntimeError("OpenCV is required for camera detection. Install python3-opencv or opencv-python.")


def compute_orbit_derivative(
    ball_angle_deg: float,
    ball_distance_cm: float,
    prev_ball_angle_deg: float,
    prev_ball_distance_cm: float,
) -> float:
    if (
        ball_angle_deg == LOST_SENTINEL
        or ball_distance_cm == LOST_SENTINEL
        or prev_ball_angle_deg == LOST_SENTINEL
        or prev_ball_distance_cm == LOST_SENTINEL
    ):
        return LOST_SENTINEL

    new_ball_angle = 360 - ball_angle_deg if ball_angle_deg > 180 else ball_angle_deg
    prev_new_ball_angle = 360 - prev_ball_angle_deg if prev_ball_angle_deg > 180 else prev_ball_angle_deg
    d_input = (
        math.sin(math.radians(int(new_ball_angle))) * int(ball_distance_cm)
        - math.sin(math.radians(int(prev_new_ball_angle))) * int(prev_ball_distance_cm)
    )
    if d_input < 0 and (ball_angle_deg < 90 or ball_angle_deg > 270):
        return round((-d_input) * 100) / 100
    return LOST_SENTINEL


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def _parse_color(obj: Any) -> tuple[int, int, int]:
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


def load_thresholds(path: str | Path = THRESHOLDS_JSON) -> ThresholdSet:
    threshold_path = Path(path)
    if threshold_path.exists():
        raw = json.loads(threshold_path.read_text())
        mask = None
        if "mask1" in raw:
            mask = (
                int(raw["mask1"]["x"]),
                int(raw["mask1"]["y"]),
                int(raw["mask1"]["size1"]),
                int(raw["mask1"]["size2"]),
            )
        return ThresholdSet(
            ball_lower=_parse_color(raw["ball"]["lower"]),
            ball_upper=_parse_color(raw["ball"]["upper"]),
            yellow_lower=_parse_color(raw["yellowGoal"]["lower"]),
            yellow_upper=_parse_color(raw["yellowGoal"]["upper"]),
            blue_lower=_parse_color(raw["blueGoal"]["lower"]),
            blue_upper=_parse_color(raw["blueGoal"]["upper"]),
            offsets=(
                int(raw.get("offsets", {}).get("x", 0)),
                int(raw.get("offsets", {}).get("y", 0)),
            ),
            mask=mask,
            raw=raw,
        )
    return ThresholdSet(
        ball_lower=FALLBACK_LOWER,
        ball_upper=FALLBACK_UPPER,
        yellow_lower=FALLBACK_LOWER,
        yellow_upper=FALLBACK_UPPER,
        blue_lower=FALLBACK_LOWER,
        blue_upper=FALLBACK_UPPER,
        offsets=(0, 0),
        mask=None,
        raw={},
    )


def load_params(path: str | Path = PARAMS_JSON) -> dict[str, Any]:
    params_path = Path(path)
    if not params_path.exists():
        return {}
    return json.loads(params_path.read_text())


def angle_and_distance_cpp(cx: int, cy: int, xoffset: int = 0, yoffset: int = 0) -> tuple[float, float]:
    midx = cx - xoffset
    midy = cy - yoffset
    angle = math.degrees(math.atan2(midx, midy)) + 180.0
    dist_px = math.hypot(midx, midy)
    return angle, dist_px


def apply_ball_distance_calibration(dist_px: float) -> float:
    if dist_px == LOST_SENTINEL:
        return LOST_SENTINEL
    return 0.00255386 * math.pow(dist_px, 2.09612)


DISTANCE_MODES = ("parametric", "onnx", "exponential")


class DistanceEstimator:
    """Camera pixel distance API with three switchable modes (params.json
    ``ball_distance_mode``):

    * ``parametric``  -- closed-form log-poly(r) x Fourier(theta) surface loaded
      from ``ball_distance_parametric_path`` (no torch/onnxruntime needed).
    * ``onnx``        -- the trained MLP exported to ONNX.
    * ``exponential`` -- the legacy ``apply_ball_distance_calibration`` power law.

    For backward compatibility, if ``ball_distance_mode`` is absent the old
    ``use_ball_distance_onnx_model`` boolean selects onnx vs exponential.
    """

    def __init__(self, params: dict[str, Any] | None = None, repo_root: str | Path = "."):
        self.params = params or load_params()
        self.repo_root = Path(repo_root)
        self.min_cm = float(self.params.get("ball_distance_min_cm", 0.0))
        self.max_cm = float(self.params.get("ball_distance_max_cm", 350.0))
        self.mode = self._resolve_mode()
        # onnx state
        self._session = None
        self._input_name = ""
        self._feature_mode = "xy_radius"
        # parametric state
        self._param_weights: np.ndarray | None = None
        self._param_radial_degree = 0
        self._param_angular_order = 0
        if self.mode == "onnx":
            self._load_onnx()
        elif self.mode == "parametric":
            self._load_parametric()
        print(f"[DISTANCE] mode = {self.mode}")

    def _resolve_mode(self) -> str:
        mode = self.params.get("ball_distance_mode")
        if mode is not None:
            mode = str(mode).lower()
            if mode not in DISTANCE_MODES:
                raise RuntimeError(f"ball_distance_mode must be one of {DISTANCE_MODES}, got {mode!r}")
            return mode
        return "onnx" if bool(self.params.get("use_ball_distance_onnx_model", False)) else "exponential"

    def _load_parametric(self) -> None:
        model_path = Path(self.params.get("ball_distance_parametric_path", "training/parametric_model.json"))
        if not model_path.is_absolute():
            model_path = self.repo_root / model_path
        if not model_path.exists():
            raise RuntimeError(f"ball_distance_mode=parametric but model file not found: {model_path}")
        spec = json.loads(model_path.read_text())
        self._param_radial_degree = int(spec["radial_degree"])
        self._param_angular_order = int(spec["angular_order"])
        self._param_weights = np.asarray(spec["weights"], dtype=np.float64)
        print(f"[DISTANCE] parametric model: {model_path.name} "
              f"(radial_degree={self._param_radial_degree}, angular_order={self._param_angular_order})")

    def _predict_parametric(self, dx: float, dy: float) -> float:
        # Must match training/fit_parametric_model.py design_matrix column order:
        # base = [1, cos1, sin1, ..., cosM, sinM]; row = concat_p (logr**p * base).
        radius = max(math.hypot(dx, dy), 1e-6)
        log_r = math.log(radius)
        theta = math.atan2(dy, dx)
        base = [1.0]
        for k in range(1, self._param_angular_order + 1):
            base.append(math.cos(k * theta))
            base.append(math.sin(k * theta))
        row: list[float] = []
        scale = 1.0
        for _ in range(self._param_radial_degree + 1):
            row.extend(scale * value for value in base)
            scale *= log_r
        log_cm = float(np.dot(self._param_weights, np.asarray(row, dtype=np.float64)))
        return math.exp(log_cm)

    def _load_onnx(self) -> None:
        try:
            import onnxruntime as ort
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "use_ball_distance_onnx_model=true but onnxruntime is not installed"
            ) from exc
        model_path = Path(self.params.get("ball_distance_model_path", "training/exported/ball_distance_latest.onnx"))
        if not model_path.is_absolute():
            model_path = self.repo_root / model_path
        self._session = ort.InferenceSession(str(model_path))
        self._input_name = self._session.get_inputs()[0].name
        metadata = self._session.get_modelmeta().custom_metadata_map
        self._feature_mode = metadata.get("feature_mode", "xy_radius")
        if self._feature_mode not in ("xy_radius", "polar"):
            raise RuntimeError(f"Unsupported ball distance ONNX feature_mode: {self._feature_mode}")
        print(f"[DISTANCE] ONNX model enabled: {model_path} ({self._feature_mode})")

    def estimate_cm(self, cx: int, cy: int, dist_px: float, thresholds: ThresholdSet) -> float:
        if dist_px == LOST_SENTINEL:
            return LOST_SENTINEL
        if self.mode == "exponential":
            distance = apply_ball_distance_calibration(dist_px)
        elif self.mode == "parametric":
            xoff, yoff = thresholds.offsets
            distance = self._predict_parametric(float(cx - xoff), float(cy - yoff))
        else:  # onnx
            xoff, yoff = thresholds.offsets
            dx = float(cx - xoff)
            dy = float(cy - yoff)
            radius = math.hypot(dx, dy)
            if self._feature_mode == "polar":
                theta = math.atan2(dy, dx)
                features = np.array([[radius, math.sin(theta), math.cos(theta)]], dtype=np.float32)
            else:
                features = np.array([[dx, dy, radius]], dtype=np.float32)
            result = self._session.run(None, {self._input_name: features})
            distance = float(result[0].flat[0])
        return max(self.min_cm, min(self.max_cm, distance))


def sector_index_from_angle(angle_deg: float) -> int:
    return int((angle_deg % 360.0) // SECTOR_ANGLE) % NUM_SECTORS


def ring_sectors(seed: int, radius: int) -> list[int]:
    if radius == 0:
        return [seed]
    return [(seed - radius) % NUM_SECTORS, (seed + radius) % NUM_SECTORS]


def clamp_vec(vx: float, vy: float, vmax: float) -> tuple[float, float]:
    mag = math.hypot(vx, vy)
    if mag <= vmax or mag == 0:
        return vx, vy
    scale = vmax / mag
    return vx * scale, vy * scale


def find_largest_contour(bin_img: Any, min_area: int = MIN_AREA_PIX) -> tuple[tuple[int, int, int, int], tuple[int, int], float] | None:
    require_cv2()
    cnts, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    max_cnt = None
    max_area = 0.0
    for contour in cnts:
        area = cv2.contourArea(contour)
        if area > max_area and area >= min_area:
            max_area = area
            max_cnt = contour
    if max_cnt is None:
        return None
    x, y, w, h = cv2.boundingRect(max_cnt)
    return (x, y, w, h), (x + w // 2, y + h // 2), max_area


class CameraDetector:
    """Reusable ball/goal camera detection API."""

    def __init__(
        self,
        thresholds: ThresholdSet | None = None,
        distance_estimator: DistanceEstimator | None = None,
    ):
        require_cv2()
        self.thresholds = thresholds or load_thresholds()
        self.distance_estimator = distance_estimator or DistanceEstimator()
        self.ellipse_mask = None
        self.sector_center = None
        self.sector_masks: list[Any] = []
        self.last_position: tuple[int, int] | None = None
        self.last_sector: int | None = None
        self.v_ema: tuple[float, float] | None = None
        self.first_frame_done = False
        self.kernel = np.ones((3, 3), np.uint8)

    def configure_for_frame(self, frame: Any) -> None:
        height, width = frame.shape[:2]
        if ENABLE_MASK or self.thresholds.mask is not None:
            self.ellipse_mask = np.zeros((height, width), dtype=np.uint8)
            if self.thresholds.mask is not None:
                mcx, mcy, s1, s2 = self.thresholds.mask
            else:
                mcx, mcy = MASK_CENTER
                s1, s2 = MASK_AXES
            cv2.ellipse(self.ellipse_mask, (int(mcx), int(mcy)), (int(s1), int(s2)), 0, 0, 360, 255, -1)
            self.sector_center = (int(mcx), int(mcy))
        else:
            self.ellipse_mask = None
            self.sector_center = (width // 2, height // 2)
        self.sector_masks = self._precompute_sector_masks(frame.shape)

    def detect(self, frame: Any, annotate: bool = False) -> FrameDetections:
        t0 = time.perf_counter()
        if not self.sector_masks:
            self.configure_for_frame(frame)

        draw_frame = frame.copy() if annotate else None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin_ball = cv2.inRange(hsv, self.thresholds.ball_lower, self.thresholds.ball_upper)
        bin_yellow = cv2.inRange(hsv, self.thresholds.yellow_lower, self.thresholds.yellow_upper)
        bin_blue = cv2.inRange(hsv, self.thresholds.blue_lower, self.thresholds.blue_upper)
        for bin_img in (bin_ball, bin_yellow, bin_blue):
            if self.ellipse_mask is not None:
                bin_img[:] = cv2.bitwise_and(bin_img, self.ellipse_mask)
            if USE_MORPH and MORPH_ITERS > 0:
                bin_img[:] = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, self.kernel, iterations=MORPH_ITERS)
                bin_img[:] = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, self.kernel, iterations=MORPH_ITERS)

        yellow_goal = self._goal_detection(bin_yellow, draw_frame, "YELLOW GOAL", (0, 255, 255))
        blue_goal = self._goal_detection(bin_blue, draw_frame, "BLUE GOAL", (255, 0, 0))
        ball, searched, predicted = self._ball_detection(bin_ball, draw_frame)

        mask_display = None
        if annotate:
            self._draw_sector_overlay(draw_frame, frame.shape, searched, predicted, ball)
            mask_display = np.zeros_like(bin_ball)
            for sector in searched:
                mask_display = cv2.bitwise_or(mask_display, cv2.bitwise_and(bin_ball, self.sector_masks[sector]))
            dt_ms = (time.perf_counter() - t0) * 1000.0
            cv2.putText(
                draw_frame,
                f"Process: {dt_ms:.1f}ms | FPS: {1000.0 / dt_ms if dt_ms > 0 else 0.0:.1f}",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return FrameDetections(
            ball=ball,
            blue_goal=blue_goal,
            yellow_goal=yellow_goal,
            searched_sectors=searched,
            predicted_sector=predicted,
            process_ms=(time.perf_counter() - t0) * 1000.0,
            annotated_frame=draw_frame,
            mask_display=mask_display,
        )

    def _goal_detection(self, bin_img: Any, draw_frame: Any, label: str, color: tuple[int, int, int]) -> BlobDetection:
        candidate = find_largest_contour(bin_img, MIN_AREA_PIX)
        if candidate is None:
            return BlobDetection()
        bbox, center, area = candidate
        angle, dist_px = angle_and_distance_cpp(center[0], center[1], *self.thresholds.offsets)
        if draw_frame is not None:
            x, y, w, h = bbox
            cv2.rectangle(draw_frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(draw_frame, label, (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return BlobDetection(
            found=True,
            bbox=bbox,
            center=center,
            area=area,
            angle_deg=angle,
            dist_px=dist_px,
            certainty=min(1.0, area / GOAL_CERT_REF_AREA),
        )

    def _ball_detection(self, bin_ball: Any, draw_frame: Any) -> tuple[BlobDetection, list[int], int | None]:
        searched: list[int] = []
        best = None
        predicted_sector = None

        if not self.first_frame_done:
            candidate = find_largest_contour(bin_ball, MIN_AREA_PIX)
            if candidate is not None:
                bbox, center, area = candidate
                ang = (math.degrees(math.atan2(center[1] - self.sector_center[1], center[0] - self.sector_center[0])) + 360.0) % 360.0
                best = (bbox, center, area, sector_index_from_angle(ang))
            searched = list(range(NUM_SECTORS))
            self.first_frame_done = True
        else:
            if self.last_position is not None and self.v_ema is not None:
                speed = math.hypot(self.v_ema[0], self.v_ema[1])
                lookahead = LOOKAHEAD_MIN + (1 if speed > LOOKAHEAD_SPEED_THRESH else 0)
                lookahead = min(LOOKAHEAD_MAX, lookahead)
                px = self.last_position[0] + self.v_ema[0] * lookahead
                py = self.last_position[1] + self.v_ema[1] * lookahead
                ang = (math.degrees(math.atan2(py - self.sector_center[1], px - self.sector_center[0])) + 360.0) % 360.0
                predicted_sector = sector_index_from_angle(ang)

            seed = predicted_sector if predicted_sector is not None else (self.last_sector if self.last_sector is not None else 0)
            found_any = False
            for radius in range(NUM_SECTORS // 2 + 1):
                for sector in ring_sectors(seed, radius):
                    if sector in searched:
                        continue
                    searched.append(sector)
                    candidate = find_largest_contour(cv2.bitwise_and(bin_ball, self.sector_masks[sector]), MIN_AREA_PIX)
                    if candidate is None:
                        continue
                    bbox, center, area = candidate
                    best = (bbox, center, area, sector)
                    found_any = True
                    if STOP_ON_FIRST_HIT:
                        break
                if found_any and STOP_ON_FIRST_HIT:
                    break

            if not found_any:
                for sector in range(NUM_SECTORS):
                    if sector in searched:
                        continue
                    searched.append(sector)
                    candidate = find_largest_contour(cv2.bitwise_and(bin_ball, self.sector_masks[sector]), MIN_AREA_PIX)
                    if candidate is None:
                        continue
                    bbox, center, area = candidate
                    best = (bbox, center, area, sector)
                    break

        if best is None:
            if self.v_ema is not None:
                self.v_ema = (0.9 * self.v_ema[0], 0.9 * self.v_ema[1])
            return BlobDetection(), searched, predicted_sector

        bbox, center, area, sector = best
        if self.last_position is not None:
            dvx = center[0] - self.last_position[0]
            dvy = center[1] - self.last_position[1]
            self.v_ema = (
                dvx if self.v_ema is None else VEL_ALPHA * self.v_ema[0] + (1.0 - VEL_ALPHA) * dvx,
                dvy if self.v_ema is None else VEL_ALPHA * self.v_ema[1] + (1.0 - VEL_ALPHA) * dvy,
            )
            self.v_ema = clamp_vec(self.v_ema[0], self.v_ema[1], VEL_MAX)
        else:
            self.v_ema = (0.0, 0.0)
        self.last_position = center
        self.last_sector = sector

        angle, dist_px = angle_and_distance_cpp(center[0], center[1], *self.thresholds.offsets)
        distance_cm = self.distance_estimator.estimate_cm(center[0], center[1], dist_px, self.thresholds)
        if draw_frame is not None:
            x, y, w, h = bbox
            cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.circle(draw_frame, center, 4, (0, 0, 255), -1)
            cv2.arrowedLine(
                draw_frame,
                center,
                (center[0] + int(5 * self.v_ema[0]), center[1] + int(5 * self.v_ema[1])),
                (255, 0, 0),
                2,
                tipLength=0.3,
            )
            cv2.putText(
                draw_frame,
                f"S{sector} | {angle:.1f} deg | {dist_px:.1f}px",
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return BlobDetection(
            found=True,
            bbox=bbox,
            center=center,
            area=area,
            angle_deg=angle,
            dist_px=dist_px,
            distance_cm=distance_cm,
            certainty=1.0,
            sector=sector,
        ), searched, predicted_sector

    def _draw_sector_overlay(
        self,
        frame: Any,
        frame_shape: tuple[int, int, int],
        searched: list[int],
        predicted: int | None,
        ball: BlobDetection,
    ) -> None:
        height, width = frame_shape[:2]
        max_r = int(math.hypot(width, height))
        for sector in range(NUM_SECTORS):
            ang = math.radians(sector * SECTOR_ANGLE)
            end = (
                int(self.sector_center[0] + max_r * math.cos(ang)),
                int(self.sector_center[1] + max_r * math.sin(ang)),
            )
            color = COLOR_UNSEARCHED
            if predicted is not None and sector == predicted:
                color = COLOR_PREDICTED
            if sector in searched:
                color = COLOR_SEARCHED
            if ball.found and sector == ball.sector:
                color = COLOR_FOUND
            cv2.line(frame, self.sector_center, end, color, 3)
            label_pos = (
                int(self.sector_center[0] + 90 * math.cos(ang)),
                int(self.sector_center[1] + 90 * math.sin(ang)),
            )
            cv2.putText(frame, str(sector), label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if self.ellipse_mask is not None:
            if self.thresholds.mask is not None:
                _, _, s1, s2 = self.thresholds.mask
            else:
                s1, s2 = MASK_AXES
            cv2.ellipse(frame, self.sector_center, (int(s1), int(s2)), 0, 0, 360, (0, 255, 255), 2)

    def _precompute_sector_masks(self, frame_shape: tuple[int, int, int]) -> list[Any]:
        height, width = frame_shape[:2]
        radius = int(math.hypot(width, height))
        masks = []
        for sector in range(NUM_SECTORS):
            start_deg = sector * SECTOR_ANGLE
            end_deg = (sector + 1) * SECTOR_ANGLE
            pts = [self.sector_center]
            for angle in range(start_deg, end_deg + 1, 2):
                radians = math.radians(angle)
                pts.append((
                    int(self.sector_center[0] + radius * math.cos(radians)),
                    int(self.sector_center[1] + radius * math.sin(radians)),
                ))
            pts.append(self.sector_center)
            wedge = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(wedge, [np.array(pts, dtype=np.int32)], 255)
            if self.ellipse_mask is not None:
                wedge = cv2.bitwise_and(wedge, self.ellipse_mask)
            masks.append(wedge)
        return masks


class PiCameraSource:
    def __init__(self, thresholds: ThresholdSet, size: tuple[int, int] = RESIZE_TO):
        if Picamera2 is None:
            raise RuntimeError("picamera2 is not installed; this must run on the Raspberry Pi")
        self.thresholds = thresholds
        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            controls={"FrameRate": CAMERA_FPS},
        )
        self.picam2.configure(cam_config)

    def start(self) -> None:
        self.picam2.start()
        time.sleep(0.5)
        exposure = self.thresholds.raw.get("exposure", {})
        contrast = float(self.thresholds.raw.get("contrast", 1.0))
        saturation = float(self.thresholds.raw.get("saturation", 1.0))
        # Match legacy_camera.py exactly: only Ae/Awb, exposure, brightness,
        # contrast, saturation (+ gain below). Legacy never sets white balance
        # (ColourTemperature) or focus (AfMode/LensPosition), so leave them at
        # the camera defaults to reproduce the legacy image.
        controls = {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": int(exposure.get("time", 10000)),
            "Brightness": float(self.thresholds.raw.get("brightness", 0.0)),
            "Contrast": contrast if contrast > 0.0 else 1.0,
            "Saturation": saturation if saturation > 0.0 else 1.0,
        }
        if "sens" in exposure:
            controls["AnalogueGain"] = max(0.0, float(exposure["sens"]) / 100.0)
        self.picam2.set_controls(controls)

    def capture_bgr(self) -> Any:
        require_cv2()
        rgb = self.picam2.capture_array()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        self.picam2.stop()
        self.picam2.close()


def open_serial() -> SerialLink | None:
    try:
        return SerialLink(SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT)
    except Exception as exc:
        print(f"[SERIAL] Failed to open serial: {exc}")
        return None


def _camera_jpeg(frame: Any) -> bytes | None:
    if cv2 is None or frame is None:
        return None
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if not ok:
        return None
    return encoded.tobytes()


def _init_lidar(enable_lidar: bool):
    if not enable_lidar:
        return None, None, None
    from tools.py_lidar_bench import config as lidar_cfg
    from tools.py_lidar_bench.lidar_bench import LD19Reader, LidarLocalizer

    reader = LD19Reader(lidar_cfg.LIDAR_PORT, lidar_cfg.LIDAR_BAUD, lidar_cfg.LIDAR_TIMEOUT)
    if not reader.is_connected:
        return None, None, None
    localizer = LidarLocalizer(
        field_width_mm=lidar_cfg.FIELD_WIDTH_MM,
        field_height_mm=lidar_cfg.FIELD_HEIGHT_MM,
        lidar_yaw_offset_deg=lidar_cfg.LIDAR_YAW_OFFSET_DEG,
    )
    return reader, localizer, deque(maxlen=lidar_cfg.LIDAR_POINTS_WINDOW)


def _telemetry_recent(telemetry: TeensyTelemetry, max_age_s: float) -> bool:
    if telemetry.timestamp_s <= 0.0:
        return False
    return (time.monotonic() - telemetry.timestamp_s) <= max_age_s


def wrap_control_angle(angle_deg: float | int | None) -> float:
    """Convert valid camera angles to -180..180 while preserving the lost sentinel."""
    if angle_deg is None:
        return LOST_SENTINEL
    angle = float(angle_deg)
    if angle == LOST_SENTINEL:
        return LOST_SENTINEL
    wrapped = angle % 360.0
    if wrapped > 180.0:
        wrapped -= 360.0
    return wrapped


def _control_intent(
    seq: int,
    pi_time_us: int,
    detections: FrameDetections,
    *,
    telemetry_fresh: bool,
) -> dict[str, Any]:
    blue_goal_angle = wrap_control_angle(detections.blue_goal_angle_deg)
    yellow_goal_angle = wrap_control_angle(detections.yellow_goal_angle_deg)
    return {
        "seq": seq,
        "pi_time_us": pi_time_us,
        "role_command": "offense",
        "mode": "heading_only_telemetry",
        "auto_role_mode": False,
        "ball_found": detections.ball.found,
        "ball_angle_deg": wrap_control_angle(detections.ball_angle_deg),
        "ball_distance_cm": detections.ball_distance_cm if detections.ball.found else LOST_SENTINEL,
        "blue_goal_found": detections.blue_goal.found,
        "blue_goal_angle_deg": blue_goal_angle,
        "yellow_goal_found": detections.yellow_goal.found,
        "yellow_goal_angle_deg": yellow_goal_angle,
        "telemetry_fresh": telemetry_fresh,
    }


def run_runtime(
    *,
    show_video: bool = False,
    enable_serial: bool = ENABLE_SERIAL,
    enable_lidar: bool = False,
    enable_foxglove: bool = False,
) -> None:
    require_cv2()
    params = load_params()
    thresholds = load_thresholds()
    distance_estimator = DistanceEstimator(params)
    detector = CameraDetector(thresholds, distance_estimator)
    camera = PiCameraSource(thresholds)
    serial_link = open_serial() if enable_serial else None
    orbit_derivative = OrbitDerivativeTracker()
    lidar_reader, lidar_localizer, lidar_window = _init_lidar(enable_lidar)
    foxglove = FoxgloveRuntimePublisher() if enable_foxglove else None
    ball_filter = BallKalman(params, camera_fps=CAMERA_FPS)
    pose_filter = PoseKalman(params)
    telemetry_stale_s = float(params.get("teensy_telemetry_stale_s", 0.25))
    telemetry = TeensyTelemetry()

    loop_count = 0
    last_t = time.perf_counter()
    last_lidar_points: list[Any] = []

    camera.start()
    print("BallAlgo Python runtime running. Press q in the debug window to quit.")
    try:
        while True:
            loop_count += 1
            now = time.perf_counter()
            dt_s = max(1e-4, now - last_t)
            last_t = now

            if serial_link is not None:
                telemetry = serial_link.poll_telemetry()

            frame = camera.capture_bgr()
            detections = detector.detect(frame, annotate=show_video or bool(foxglove and foxglove.stream_camera))
            pi_time_us = time.monotonic_ns() // 1000
            telemetry_fresh = _telemetry_recent(telemetry, telemetry_stale_s)

            heading = telemetry.heading_deg
            if lidar_reader is not None and lidar_window is not None:
                new_points = lidar_reader.poll_points()
                if new_points:
                    lidar_window.extend(new_points)
                last_lidar_points = list(lidar_window)
                if lidar_localizer is not None and last_lidar_points:
                    result = lidar_localizer.update(last_lidar_points, heading)
                    pose_filter.update(
                        PoseEstimate(
                            valid=bool(result.get("valid")),
                            x_mm=float(result.get("x_mm") or 0.0),
                            y_mm=float(result.get("y_mm") or 0.0),
                        ),
                        heading,
                    )

            pose_filter.predict(dt_s)
            pose_state = pose_filter.state(heading)

            if serial_link is not None:
                derivative = orbit_derivative.update(
                    detections.ball_angle_deg,
                    detections.ball_distance_cm,
                )
                serial_link.send_detection(
                    detections.ball_angle_deg,
                    detections.ball_distance_cm,
                    detections.blue_goal_angle_deg,
                    detections.yellow_goal_angle_deg,
                    derivative,
                    pose_state.x_mm if pose_state.valid else LOST_SENTINEL,
                    pose_state.y_mm if pose_state.valid else LOST_SENTINEL,
                )

            ball_filter.predict(dt_s)
            if detections.ball.found:
                x_m, y_m = polar_to_body_xy(detections.ball_angle_deg, detections.ball_distance_cm * 0.01)
                ball_filter.update(x_m, y_m, True)
            ball_state = ball_filter.state()

            if detections.blue_goal.found and detections.blue_goal.certainty >= float(params.get("goal_certainty_threshold", 0.25)):
                pose_filter.update_goal_bearing(
                    math.radians(detections.blue_goal.angle_deg - 180.0),
                    BLUE_GOAL_X_MM,
                    BLUE_GOAL_Y_MM,
                    heading,
                    detections.blue_goal.certainty,
                )
            if detections.yellow_goal.found and detections.yellow_goal.certainty >= float(params.get("goal_certainty_threshold", 0.25)):
                pose_filter.update_goal_bearing(
                    math.radians(detections.yellow_goal.angle_deg - 180.0),
                    YELLOW_GOAL_X_MM,
                    YELLOW_GOAL_Y_MM,
                    heading,
                    detections.yellow_goal.certainty,
                )
            pose_state = pose_filter.state(heading)

            if foxglove is not None:
                publish_frame = detections.annotated_frame if detections.annotated_frame is not None else frame
                foxglove.publish(
                    detections=detections.snapshot_dict(),
                    telemetry=telemetry,
                    pose_state=pose_state,
                    ball_state=ball_state,
                    control_intent=_control_intent(
                        loop_count,
                        pi_time_us,
                        detections,
                        telemetry_fresh=telemetry_fresh,
                    ),
                    lidar_points=last_lidar_points,
                    camera_jpeg_bytes=_camera_jpeg(publish_frame) if foxglove.stream_camera else None,
                    loop_count=loop_count,
                    dt_s=dt_s,
                )

            if show_video:
                raw_view = frame.copy()
                cv2.putText(raw_view, "Raw camera feed", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("Raw Camera", raw_view)
                view = detections.annotated_frame if detections.annotated_frame is not None else frame
                cv2.imshow("Ball Detection", cv2.cvtColor(view, COLOR_RGB2BGR))
                if detections.mask_display is not None:
                    cv2.imshow("Mask (binary)", detections.mask_display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        camera.close()
        if serial_link is not None:
            serial_link.close()
        if lidar_reader is not None:
            lidar_reader.close()
        if foxglove is not None:
            foxglove.close()
        cv2.destroyAllWindows()


def process_video(showVideo: bool) -> None:
    run_runtime(show_video=showVideo, enable_serial=ENABLE_SERIAL)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BallAlgo Python Pi runtime")
    parser.add_argument("--show-video", action="store_true", help="Show OpenCV debug windows")
    parser.add_argument("--no-serial", action="store_true", help="Do not open Pi/Teensy serial")
    parser.add_argument("--enable-lidar", action="store_true", help="Enable LD19 lidar localization")
    parser.add_argument("--enable-foxglove", action="store_true", help="Publish runtime snapshots to foxglove_sim sidecar")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_runtime(
        show_video=args.show_video,
        enable_serial=not args.no_serial,
        enable_lidar=args.enable_lidar,
        enable_foxglove=args.enable_foxglove,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
