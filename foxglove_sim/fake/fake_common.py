"""Shared utilities for fake runtime scenarios."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any

# Make the parent foxglove_sim/ directory importable (field_geometry, config, etc.)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from field_geometry import DEFAULT_FIELD_HEIGHT_MM, DEFAULT_FIELD_WIDTH_MM

FIELD_WIDTH_MM = DEFAULT_FIELD_WIDTH_MM
FIELD_HEIGHT_MM = DEFAULT_FIELD_HEIGHT_MM
FRAME_ID = "field"


def now_timestamp_ns() -> int:
    import time
    return time.time_ns()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def point_heading_deg(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def centered_cm_to_mm(x_cm: float, y_cm: float) -> tuple[float, float]:
    """Convert center-relative cm to bottom-left-origin mm (matching LidarLocalizer output)."""
    return x_cm * 10.0 + FIELD_WIDTH_MM * 0.5, y_cm * 10.0 + FIELD_HEIGHT_MM * 0.5


def path_length_mm(points: list[tuple[float, float]]) -> list[float]:
    distances = [0.0]
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
        distances.append(total)
    return distances


def encode_snapshot(snapshot: dict[str, Any]) -> bytes:
    return json.dumps(snapshot, separators=(",", ":")).encode("utf-8")


def make_snapshot(
    *,
    timestamp_ns: int,
    pose: dict[str, Any],
    ball: dict[str, Any],
    planner_path: dict[str, Any],
    robot_twist: dict[str, Any],
    robot_accel: dict[str, Any],
    robot_angular: dict[str, Any],
    ball_twist: dict[str, Any],
    log_message: str,
    log_level: int = 2,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "timestamp_ns": timestamp_ns,
        "field": {
            "frame_id": FRAME_ID,
            "width_mm": FIELD_WIDTH_MM,
            "height_mm": FIELD_HEIGHT_MM,
        },
        "pose": pose,
        "ball": ball,
        "planner_path": planner_path,
        "robot_twist": robot_twist,
        "robot_accel": robot_accel,
        "robot_angular": robot_angular,
        "ball_twist": ball_twist,
        "log": {
            "level": log_level,
            "name": "ballalgo.fake_runtime",
            "message": log_message,
        },
    }
