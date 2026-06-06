from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


DEFAULT_FRAME_ID = "field"
DEFAULT_FIELD_WIDTH_MM = 1820.0
DEFAULT_FIELD_HEIGHT_MM = 2430.0

GOAL_INNER_WIDTH_MM = 600.0
GOAL_DEPTH_MM = 74.0
PENALTY_BOX_WIDTH_MM = 750.0
PENALTY_BOX_DEPTH_MM = 450.0
CENTER_CIRCLE_RADIUS_MM = 300.0


@dataclass(frozen=True, slots=True)
class FieldGeometry:
    frame_id: str
    width_mm: float
    height_mm: float

    @property
    def half_width_mm(self) -> float:
        return self.width_mm * 0.5

    @property
    def half_height_mm(self) -> float:
        return self.height_mm * 0.5

    @classmethod
    def from_snapshot(cls, field: Mapping[str, Any]) -> "FieldGeometry":
        return cls(
            frame_id=str(field.get("frame_id", DEFAULT_FRAME_ID)),
            width_mm=float(field.get("width_mm", DEFAULT_FIELD_WIDTH_MM)),
            height_mm=float(field.get("height_mm", DEFAULT_FIELD_HEIGHT_MM)),
        )


def rectangle_loop_mm(
    *, min_x_mm: float, max_x_mm: float, min_y_mm: float, max_y_mm: float
) -> list[tuple[float, float]]:
    return [
        (min_x_mm, min_y_mm),
        (max_x_mm, min_y_mm),
        (max_x_mm, max_y_mm),
        (min_x_mm, max_y_mm),
        (min_x_mm, min_y_mm),
    ]


def border_loop_mm(field: FieldGeometry) -> list[tuple[float, float]]:
    return rectangle_loop_mm(
        min_x_mm=-field.half_width_mm,
        max_x_mm=field.half_width_mm,
        min_y_mm=-field.half_height_mm,
        max_y_mm=field.half_height_mm,
    )


def center_line_mm(field: FieldGeometry) -> list[tuple[float, float]]:
    return [
        (0.0, -field.half_height_mm),
        (0.0, field.half_height_mm),
    ]


def circle_polyline_mm(radius_mm: float, samples: int = 48) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(samples + 1):
        theta = 2.0 * math.pi * index / max(samples, 1)
        points.append((radius_mm * math.cos(theta), radius_mm * math.sin(theta)))
    return points


def penalty_box_loops_mm(field: FieldGeometry) -> dict[str, list[tuple[float, float]]]:
    half_box_height = PENALTY_BOX_WIDTH_MM * 0.5
    return {
        "field-left-penalty-box": rectangle_loop_mm(
            min_x_mm=-field.half_width_mm,
            max_x_mm=-field.half_width_mm + PENALTY_BOX_DEPTH_MM,
            min_y_mm=-half_box_height,
            max_y_mm=half_box_height,
        ),
        "field-right-penalty-box": rectangle_loop_mm(
            min_x_mm=field.half_width_mm - PENALTY_BOX_DEPTH_MM,
            max_x_mm=field.half_width_mm,
            min_y_mm=-half_box_height,
            max_y_mm=half_box_height,
        ),
    }


def center_field_mm(x_mm: float, y_mm: float, field: FieldGeometry) -> tuple[float, float]:
    """Convert bottom-left-origin field mm (as output by LidarLocalizer) to center-origin mm
    for Foxglove rendering, where (0, 0) is the middle of the field."""
    return x_mm - field.half_width_mm, y_mm - field.half_height_mm


def goal_loops_mm(field: FieldGeometry) -> dict[str, list[tuple[float, float]]]:
    half_goal_height = GOAL_INNER_WIDTH_MM * 0.5
    return {
        "goal-left": rectangle_loop_mm(
            min_x_mm=-field.half_width_mm - GOAL_DEPTH_MM,
            max_x_mm=-field.half_width_mm,
            min_y_mm=-half_goal_height,
            max_y_mm=half_goal_height,
        ),
        "goal-right": rectangle_loop_mm(
            min_x_mm=field.half_width_mm,
            max_x_mm=field.half_width_mm + GOAL_DEPTH_MM,
            min_y_mm=-half_goal_height,
            max_y_mm=half_goal_height,
        ),
    }
