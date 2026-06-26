from __future__ import annotations

import math
from typing import Any

from field_geometry import (
    CENTER_CIRCLE_RADIUS_MM,
    FieldGeometry,
    border_loop_mm,
    center_field_mm,
    center_line_mm,
    circle_polyline_mm,
    goal_loops_mm,
    penalty_box_loops_mm,
)
from foxglove.messages import (
    ArrowPrimitive,
    Color,
    CubePrimitive,
    LinePrimitive,
    Point3,
    Pose,
    Quaternion,
    SceneEntity,
    SpherePrimitive,
    TextPrimitive,
    Timestamp,
    Vector3,
)


FIELD_LINE_COLOR = Color(r=1.0, g=1.0, b=1.0, a=1.0)
FIELD_SECONDARY_COLOR = Color(r=0.78, g=0.82, b=0.84, a=1.0)
LEFT_GOAL_COLOR = Color(r=0.25, g=0.45, b=1.0, a=1.0)
RIGHT_GOAL_COLOR = Color(r=1.0, g=0.9, b=0.2, a=1.0)
FIELD_SURFACE_COLOR = Color(r=0.1, g=0.42, b=0.2, a=0.96)

ROBOT_BODY_COLOR = Color(r=0.1, g=0.65, b=1.0, a=0.95)
ROBOT_NOSE_COLOR = Color(r=1.0, g=1.0, b=1.0, a=0.95)
# Robot arrows — kept visually distinct (and matched in the legend).
ACTUAL_HEADING_COLOR = Color(r=0.20, g=0.85, b=1.0, a=1.0)       # cyan
BALL_BEARING_COLOR = Color(r=1.0, g=0.62, b=0.10, a=1.0)         # orange
BALL_COLOR = Color(r=1.0, g=0.55, b=0.1, a=1.0)
HUD_TEXT_COLOR = Color(r=0.95, g=0.98, b=1.0, a=1.0)

# Fixed lengths (m) for direction-only heading arrows.
ACTUAL_HEADING_LEN_M = 0.16
BALL_BEARING_LEN_M = 0.32


def yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    half = math.radians(yaw_deg) * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def build_static_scene_entities(field: dict[str, Any], timestamp: Timestamp) -> list[SceneEntity]:
    geometry = FieldGeometry.from_snapshot(field)
    entities = [
        SceneEntity(
            timestamp=timestamp,
            frame_id=geometry.frame_id,
            id="field-surface",
            cubes=[
                CubePrimitive(
                    pose=_pose_from_mm(0.0, 0.0, 0.0, z_m=-0.01),
                    size=Vector3(
                        x=geometry.width_mm / 1000.0,
                        y=geometry.height_mm / 1000.0,
                        z=0.01,
                    ),
                    color=FIELD_SURFACE_COLOR,
                )
            ],
        ),
        _line_entity("field-border", geometry.frame_id, timestamp, border_loop_mm(geometry), 0.02, FIELD_LINE_COLOR),
        _line_entity(
            "field-center-line", geometry.frame_id, timestamp, center_line_mm(geometry), 0.012, FIELD_SECONDARY_COLOR
        ),
        _line_entity(
            "field-center-circle",
            geometry.frame_id,
            timestamp,
            circle_polyline_mm(CENTER_CIRCLE_RADIUS_MM, samples=56),
            0.012,
            FIELD_SECONDARY_COLOR,
        ),
    ]

    for entity_id, points_mm in penalty_box_loops_mm(geometry).items():
        entities.append(_line_entity(entity_id, geometry.frame_id, timestamp, points_mm, 0.014, FIELD_SECONDARY_COLOR))

    goal_colors = {
        "goal-blue":   LEFT_GOAL_COLOR,
        "goal-yellow": RIGHT_GOAL_COLOR,
    }
    for entity_id, points_mm in goal_loops_mm(geometry).items():
        entities.append(_line_entity(entity_id, geometry.frame_id, timestamp, points_mm, 0.04, goal_colors[entity_id]))

    return entities


def build_live_scene_entities(snapshot: dict[str, Any], timestamp: Timestamp) -> list[SceneEntity]:
    field = FieldGeometry.from_snapshot(snapshot["field"])
    pose = snapshot.get("pose")
    ball = snapshot.get("ball")
    entities: list[SceneEntity] = [_legend_entity(field, timestamp), _hud_entity(field, timestamp, snapshot)]

    if pose and pose.get("valid"):
        entities.append(_robot_entity(field, timestamp, snapshot))

    if ball and ball.get("field_visible"):
        cx_mm, cy_mm = center_field_mm(ball["field_x_mm"], ball["field_y_mm"], field)
        entities.append(
            SceneEntity(
                timestamp=timestamp,
                frame_id=field.frame_id,
                id="ball-state",
                spheres=[
                    SpherePrimitive(
                        pose=_pose_from_mm(cx_mm, cy_mm, 0.0, z_m=0.015),
                        size=Vector3(x=0.05, y=0.05, z=0.03),
                        color=BALL_COLOR,
                    )
                ],
            )
        )

    return entities


def _robot_entity(field: FieldGeometry, timestamp: Timestamp, snapshot: dict[str, Any]) -> SceneEntity:
    pose = snapshot["pose"]
    robot_x_mm, robot_y_mm = center_field_mm(pose["x_mm"], pose["y_mm"], field)
    robot_heading_deg = pose["heading_deg"]

    arrows: list[ArrowPrimitive] = []

    # 1. Actual heading — robot's measured heading (compass). Cyan.
    actual_heading = _heading_arrow(
        robot_x_mm, robot_y_mm, robot_heading_deg,
        ACTUAL_HEADING_LEN_M, ACTUAL_HEADING_COLOR, z_m=0.012,
    )
    if actual_heading is not None:
        arrows.append(actual_heading)

    # 2. Ball angle relative to the robot — the current Pi bearing input.
    control = snapshot.get("control_intent")
    if control is not None and control.get("ball_found"):
        ball_heading = robot_heading_deg + float(control.get("ball_angle_deg", 180.0)) - 180.0
        ball_bearing = _heading_arrow(
            robot_x_mm, robot_y_mm, ball_heading,
            BALL_BEARING_LEN_M, BALL_BEARING_COLOR, z_m=0.014,
        )
        if ball_bearing is not None:
            arrows.append(ball_bearing)

    # Nose dot is 55 mm in front of the robot body center.
    nose_x_mm = robot_x_mm + 55.0 * math.cos(math.radians(robot_heading_deg))
    nose_y_mm = robot_y_mm + 55.0 * math.sin(math.radians(robot_heading_deg))
    return SceneEntity(
        timestamp=timestamp,
        frame_id=field.frame_id,
        id="robot-state",
        spheres=[
            SpherePrimitive(
                pose=_pose_from_mm(robot_x_mm, robot_y_mm, 0.0, z_m=0.01),
                size=Vector3(x=0.12, y=0.12, z=0.02),
                color=ROBOT_BODY_COLOR,
            ),
            SpherePrimitive(
                pose=_pose_from_mm(nose_x_mm, nose_y_mm, 0.0, z_m=0.015),
                size=Vector3(x=0.03, y=0.03, z=0.02),
                color=ROBOT_NOSE_COLOR,
            ),
        ],
        arrows=arrows,
    )


def _legend_entity(field: FieldGeometry, timestamp: Timestamp) -> SceneEntity:
    """In-scene color key for the live vectors, anchored at the field's top-left."""
    half_w_m = field.half_width_mm / 1000.0
    half_h_m = field.half_height_mm / 1000.0
    anchor_x = -half_w_m + 0.42
    top_y = half_h_m - 0.12
    spacing = 0.16
    rows = [
        ("Actual heading", ACTUAL_HEADING_COLOR),
        ("Ball angle rel robot", BALL_BEARING_COLOR),
    ]
    texts = [
        TextPrimitive(
            pose=Pose(
                position=Vector3(x=anchor_x, y=top_y - index * spacing, z=0.02),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            billboard=True,
            font_size=14.0,
            scale_invariant=True,
            color=color,
            text=label,
        )
        for index, (label, color) in enumerate(rows)
    ]
    return SceneEntity(
        timestamp=timestamp,
        frame_id=field.frame_id,
        id="robot-arrow-legend",
        texts=texts,
    )


def _hud_entity(field: FieldGeometry, timestamp: Timestamp, snapshot: dict[str, Any]) -> SceneEntity:
    """Compact pose/value HUD inside the field scene.

    Foxglove's 3D panel does not provide a built-in legend or numeric inspector
    for custom scene entities, so we render a small in-scene status block for
    the latest robot/ball state.
    """
    half_w_m = field.half_width_mm / 1000.0
    half_h_m = field.half_height_mm / 1000.0
    anchor_x = half_w_m - 0.90
    top_y = half_h_m - 0.12
    spacing = 0.14

    pose = snapshot.get("pose", {})
    ball = snapshot.get("ball", {})
    control = snapshot.get("control_intent", {})

    robot_line = "Robot: lost"
    if pose.get("valid"):
        rx_mm, ry_mm = center_field_mm(pose.get("x_mm", 0.0), pose.get("y_mm", 0.0), field)
        robot_line = (
            f"Robot: x={rx_mm/1000.0:+.2f} m  y={ry_mm/1000.0:+.2f} m  "
            f"hdg={pose.get('heading_deg', 0.0):.1f} deg"
        )

    ball_line = "Ball: lost"
    if ball.get("field_visible"):
        bx_mm, by_mm = center_field_mm(ball.get("field_x_mm", 0.0), ball.get("field_y_mm", 0.0), field)
        ball_line = f"Ball:  x={bx_mm/1000.0:+.2f} m  y={by_mm/1000.0:+.2f} m"

    bearing_line = "Ball rel: unavailable"
    if control.get("ball_found"):
        bearing_line = (
            f"Ball rel: ang={control.get('ball_angle_deg', -5.0):.1f} deg  "
            f"dist={control.get('ball_distance_cm', -5.0):.1f} cm"
        )

    mode_line = (
        f"Mode: {control.get('mode', 'unknown')}  "
        f"Role: {control.get('role_command', 'unknown')}"
    )

    lines = [robot_line, ball_line, bearing_line, mode_line]
    texts = [
        TextPrimitive(
            pose=Pose(
                position=Vector3(x=anchor_x, y=top_y - index * spacing, z=0.02),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            billboard=True,
            font_size=13.0,
            scale_invariant=True,
            color=HUD_TEXT_COLOR,
            text=line,
        )
        for index, line in enumerate(lines)
    ]
    return SceneEntity(
        timestamp=timestamp,
        frame_id=field.frame_id,
        id="field-hud",
        texts=texts,
    )


def _line_entity(
    entity_id: str,
    frame_id: str,
    timestamp: Timestamp,
    points_mm: list[tuple[float, float]],
    thickness_m: float,
    color: Color,
) -> SceneEntity:
    return SceneEntity(
        timestamp=timestamp,
        frame_id=frame_id,
        id=entity_id,
        lines=[
            LinePrimitive(
                thickness=thickness_m,
                scale_invariant=True,
                points=[_point_from_mm(x_mm, y_mm, z_m=0.0) for x_mm, y_mm in points_mm],
                color=color,
            )
        ],
    )


def _pose_from_mm(x_mm: float, y_mm: float, yaw_deg: float, *, z_m: float = 0.0) -> Pose:
    return Pose(
        position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_m),
        orientation=yaw_to_quaternion(yaw_deg),
    )


def _point_from_mm(x_mm: float, y_mm: float, *, z_m: float = 0.0) -> Point3:
    return Point3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_m)


def _heading_arrow(
    x_mm: float,
    y_mm: float,
    heading_deg: float,
    length_m: float,
    color: Color,
    *,
    z_m: float = 0.0,
) -> ArrowPrimitive | None:
    return _arrow_from_components(
        x_mm,
        y_mm,
        length_m * math.cos(math.radians(heading_deg)),
        length_m * math.sin(math.radians(heading_deg)),
        color,
        z_m=z_m,
    )


def _arrow_from_components(
    x_mm: float,
    y_mm: float,
    vx_m_s: float,
    vy_m_s: float,
    color: Color,
    *,
    z_m: float = 0.0,
) -> ArrowPrimitive | None:
    magnitude = math.hypot(vx_m_s, vy_m_s)
    if magnitude < 1e-6:
        return None
    yaw_deg = math.degrees(math.atan2(vy_m_s, vx_m_s))
    return ArrowPrimitive(
        pose=Pose(
            position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_m),
            orientation=yaw_to_quaternion(yaw_deg),
        ),
        shaft_length=magnitude,
        shaft_diameter=0.012,
        head_length=0.04,
        head_diameter=0.025,
        color=color,
    )
