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
            "dist_cal_m": {"type": "number"},
            "body_x_m": {"type": "number"},
            "body_y_m": {"type": "number"},
        },
        required=["visible"],
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
