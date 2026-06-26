from __future__ import annotations

import json


def _json_schema(title: str, properties: dict[str, object], required: list[str] | None = None) -> bytes:
    schema = {
        "type": "object",
        "title": title,
        "properties": properties,
        "additionalProperties": False,
    }
    if required:
        schema["required"] = required
    return json.dumps(schema, separators=(",", ":")).encode("utf-8")


SCHEMAS: dict[str, bytes] = {
    "ballalgo.RobotLinearVelocity": _json_schema(
        "RobotLinearVelocity",
        {
            "vx_body_m_s": {"type": "number"},
            "vy_body_m_s": {"type": "number"},
            "speed_body_m_s": {"type": "number"},
            "vx_field_m_s": {"type": "number"},
            "vy_field_m_s": {"type": "number"},
            "speed_field_m_s": {"type": "number"},
        },
    ),
    "ballalgo.RobotLinearAcceleration": _json_schema(
        "RobotLinearAcceleration",
        {
            "ax_body_m_s2": {"type": "number"},
            "ay_body_m_s2": {"type": "number"},
            "magnitude_body_m_s2": {"type": "number"},
            "ax_field_m_s2": {"type": "number"},
            "ay_field_m_s2": {"type": "number"},
            "magnitude_field_m_s2": {"type": "number"},
        },
    ),
    "ballalgo.RobotAngularKinematics": _json_schema(
        "RobotAngularKinematics",
        {
            "heading_deg": {"type": "number"},
            "omega_deg_s": {"type": "number"},
            "alpha_deg_s2": {"type": "number"},
        },
    ),
    "ballalgo.BallVelocity": _json_schema(
        "BallVelocity",
        {
            "visible": {"type": "boolean"},
            "vx_body_m_s": {"type": "number"},
            "vy_body_m_s": {"type": "number"},
            "speed_body_m_s": {"type": "number"},
            "vx_field_m_s": {"type": "number"},
            "vy_field_m_s": {"type": "number"},
            "speed_field_m_s": {"type": "number"},
        },
    ),
    "ballalgo.BallRange": _json_schema(
        "BallRange",
        {
            "visible": {"type": "boolean"},
            "angle_deg": {"type": "number"},
            "dist_cal_cm": {"type": "number"},
            "body_x_m": {"type": "number"},
            "body_y_m": {"type": "number"},
        },
        required=["visible"],
    ),
    "ballalgo.RobotPoseState": _json_schema(
        "RobotPoseState",
        {
            "valid": {"type": "boolean"},
            "x_mm": {"type": "number"},
            "y_mm": {"type": "number"},
            "heading_deg": {"type": "number"},
            "vx_body_m_s": {"type": "number"},
            "vy_body_m_s": {"type": "number"},
        },
        required=["valid"],
    ),
    "ballalgo.BallPoseState": _json_schema(
        "BallPoseState",
        {
            "visible": {"type": "boolean"},
            "field_visible": {"type": "boolean"},
            "field_x_mm": {"type": "number"},
            "field_y_mm": {"type": "number"},
            "body_x_m": {"type": "number"},
            "body_y_m": {"type": "number"},
            "vision_angle_deg": {"type": "number"},
            "vision_dist_cal_cm": {"type": "number"},
        },
        required=["visible"],
    ),
    "ballalgo.ControlIntent": _json_schema(
        "ControlIntent",
        {
            "seq": {"type": "integer"},
            "pi_time_us": {"type": "integer"},
            "role_command": {"type": "string"},
            "mode": {"type": "string"},
            "auto_role_mode": {"type": "boolean"},
            "ball_found": {"type": "boolean"},
            "ball_angle_deg": {"type": "number"},
            "ball_distance_cm": {"type": "number"},
            "blue_goal_found": {"type": "boolean"},
            "blue_goal_angle_deg": {"type": "number"},
            "yellow_goal_found": {"type": "boolean"},
            "yellow_goal_angle_deg": {"type": "number"},
            "scoring_goal_found": {"type": "boolean"},
            "scoring_goal_angle_deg": {"type": "number"},
            "own_goal_found": {"type": "boolean"},
            "own_goal_angle_deg": {"type": "number"},
            "offense_active": {"type": "boolean"},
            "state_machine_enabled": {"type": "boolean"},
            "control_packet_enabled": {"type": "boolean"},
            "command_link_fresh": {"type": "boolean"},
            "telemetry_fresh": {"type": "boolean"},
            "offense_command": {"type": "integer"},
            "offense_command_name": {"type": "string"},
            "dribbler_power": {"type": "integer"},
            "kick_request": {"type": "boolean"},
            "preferred_orbit_dir": {"type": "integer"},
            "kick_cooldown_active": {"type": "boolean"},
        },
        required=["seq", "role_command", "mode"],
    ),
    "ballalgo.TeensyRawTelemetry": _json_schema(
        "TeensyRawTelemetry",
        {
            "heading_deg": {"type": "number"},
            "has_ball": {"type": "boolean"},
            "start_enabled": {"type": "boolean"},
            "goal_is_blue": {"type": "boolean"},
            "mode_override": {"type": "string"},
            "telemetry_fresh": {"type": "boolean"},
            "line_angle_deg": {"type": "number"},
            "avoidance_angle_deg": {"type": "number"},
            "chord_length": {"type": "number"},
            "cross_line": {"type": "boolean"},
            "camera_ball_angle_deg": {"type": "number"},
            "camera_ball_distance_cm": {"type": "number"},
            "camera_blue_goal_angle_deg": {"type": "number"},
            "camera_yellow_goal_angle_deg": {"type": "number"},
            "camera_fresh": {"type": "boolean"},
        },
    ),
    "ballalgo.SessionInfo": _json_schema(
        "SessionInfo",
        {
            "session_label": {"type": "string"},
            "started_at": {"type": "string"},
            "git_sha": {"type": "string"},
            "config_path": {"type": "string"},
            "recording_path": {"type": "string"},
            "note": {"type": "string"},
            "config": {"type": "object"},
        },
        required=["session_label", "started_at", "config"],
    ),
}
