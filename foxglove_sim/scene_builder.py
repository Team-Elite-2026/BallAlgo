from __future__ import annotations

import math
from typing import Any

from field_geometry import (
    CENTER_CIRCLE_RADIUS_MM,
    FieldGeometry,
    border_loop_mm,
    center_line_mm,
    circle_polyline_mm,
    goal_loops_mm,
    penalty_box_loops_mm,
)
from foxglove.messages import (
    ArrowPrimitive,
    Color,
    LinePrimitive,
    Point3,
    Pose,
    Quaternion,
    SceneEntity,
    SpherePrimitive,
    Timestamp,
    Vector3,
)


FIELD_LINE_COLOR = Color(r=1.0, g=1.0, b=1.0, a=1.0)
FIELD_SECONDARY_COLOR = Color(r=0.78, g=0.82, b=0.84, a=1.0)
LEFT_GOAL_COLOR = Color(r=0.25, g=0.45, b=1.0, a=1.0)
RIGHT_GOAL_COLOR = Color(r=1.0, g=0.9, b=0.2, a=1.0)

ROBOT_BODY_COLOR = Color(r=0.1, g=0.65, b=1.0, a=0.95)
ROBOT_NOSE_COLOR = Color(r=1.0, g=1.0, b=1.0, a=0.95)
ROBOT_HEADING_COLOR = Color(r=0.1, g=0.65, b=1.0, a=1.0)
ROBOT_VELOCITY_COLOR = Color(r=0.2, g=1.0, b=0.45, a=0.95)
BALL_COLOR = Color(r=1.0, g=0.55, b=0.1, a=1.0)
PATH_COLOR = Color(r=0.1, g=0.8, b=0.3, a=1.0)
TARGET_COLOR = Color(r=1.0, g=0.2, b=0.1, a=0.95)


def build_static_scene_entities(field: dict[str, Any], timestamp: Timestamp) -> list[SceneEntity]:
    geometry = FieldGeometry.from_snapshot(field)
    entities = [
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
        "goal-left": LEFT_GOAL_COLOR,
        "goal-right": RIGHT_GOAL_COLOR,
    }
    for entity_id, points_mm in goal_loops_mm(geometry).items():
        entities.append(_line_entity(entity_id, geometry.frame_id, timestamp, points_mm, 0.02, goal_colors[entity_id]))

    return entities


def build_live_scene_entities(snapshot: dict[str, Any], timestamp: Timestamp) -> list[SceneEntity]:
    field = snapshot["field"]
    pose = snapshot.get("pose")
    ball = snapshot.get("ball")
    entities: list[SceneEntity] = []

    if pose and pose.get("valid"):
        entities.append(_robot_entity(field["frame_id"], timestamp, snapshot))

    if ball and ball.get("field_visible"):
        entities.append(
            SceneEntity(
                timestamp=timestamp,
                frame_id=field["frame_id"],
                id="ball-state",
                spheres=[
                    SpherePrimitive(
                        pose=_pose_from_mm(ball["field_x_mm"], ball["field_y_mm"], 0.0, z_m=0.015),
                        size=Vector3(x=0.05, y=0.05, z=0.03),
                        color=BALL_COLOR,
                    )
                ],
            )
        )

    return entities


def build_path_scene_entities(snapshot: dict[str, Any], timestamp: Timestamp) -> list[SceneEntity]:
    planner = snapshot.get("planner_path")
    if not planner:
        return []

    frame_id = snapshot["field"]["frame_id"]
    points = [_point_from_mm(point["x_mm"], point["y_mm"], z_m=0.01) for point in planner.get("points", [])]
    entities: list[SceneEntity] = []

    if points:
        entities.append(
            SceneEntity(
                timestamp=timestamp,
                frame_id=frame_id,
                id="planner-path",
                lines=[
                    LinePrimitive(
                        thickness=0.03,
        scale_invariant=True,
                        points=points,
                        color=PATH_COLOR,
                    )
                ],
            )
        )

    target_heading = planner["target_heading_deg"]
    target_x_mm = planner["target_x_mm"]
    target_y_mm = planner["target_y_mm"]
    target_arrow = _arrow_from_components(
        target_x_mm,
        target_y_mm,
        0.16 * math.cos(math.radians(target_heading)),
        0.16 * math.sin(math.radians(target_heading)),
        TARGET_COLOR,
        z_m=0.01,
    )
    entities.append(
        SceneEntity(
            timestamp=timestamp,
            frame_id=frame_id,
            id="planner-target",
            spheres=[
                SpherePrimitive(
                    pose=_pose_from_mm(target_x_mm, target_y_mm, target_heading, z_m=0.012),
                    size=Vector3(x=0.045, y=0.045, z=0.025),
                    color=TARGET_COLOR,
                )
            ],
            arrows=[target_arrow] if target_arrow is not None else [],
        )
    )
    return entities


def _robot_entity(frame_id: str, timestamp: Timestamp, snapshot: dict[str, Any]) -> SceneEntity:
    pose = snapshot["pose"]
    robot_x_mm = pose["x_mm"]
    robot_y_mm = pose["y_mm"]
    robot_heading_deg = pose["heading_deg"]

    heading_arrow = _heading_arrow(
        robot_x_mm,
        robot_y_mm,
        robot_heading_deg,
        0.15,
        ROBOT_HEADING_COLOR,
        z_m=0.01,
    )
    velocity_arrow = None
    robot_twist = snapshot.get("robot_twist")
    if robot_twist is not None:
        velocity_arrow = _arrow_from_components(
            robot_x_mm,
            robot_y_mm,
            robot_twist.get("vx_field_m_s", 0.0),
            robot_twist.get("vy_field_m_s", 0.0),
            ROBOT_VELOCITY_COLOR,
            z_m=0.012,
        )

    nose_x_mm = robot_x_mm + 55.0 * math.cos(math.radians(robot_heading_deg))
    nose_y_mm = robot_y_mm + 55.0 * math.sin(math.radians(robot_heading_deg))
    return SceneEntity(
        timestamp=timestamp,
        frame_id=frame_id,
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
        arrows=[arrow for arrow in (heading_arrow, velocity_arrow) if arrow is not None],
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
                scale_invariant=False,
                points=[_point_from_mm(x_mm, y_mm, z_m=0.0) for x_mm, y_mm in points_mm],
                color=color,
            )
        ],
    )


def _timestamped_pose(x_mm: float, y_mm: float, yaw_deg: float, z_m: float) -> Pose:
    return Pose(
        position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_m),
        orientation=_yaw_to_quaternion(yaw_deg),
    )


def _pose_from_mm(x_mm: float, y_mm: float, yaw_deg: float, *, z_m: float = 0.0) -> Pose:
    return _timestamped_pose(x_mm, y_mm, yaw_deg, z_m)


def _point_from_mm(x_mm: float, y_mm: float, *, z_m: float = 0.0) -> Point3:
    return Point3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_m)


def _yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    half = math.radians(yaw_deg) * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


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
            orientation=_yaw_to_quaternion(yaw_deg),
        ),
        shaft_length=magnitude,
        shaft_diameter=0.012,
        head_length=0.04,
        head_diameter=0.025,
        color=color,
    )
