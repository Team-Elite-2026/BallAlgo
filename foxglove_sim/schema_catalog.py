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
    "ballalgo.TrajectorySpeedProfile": _json_schema(
        "TrajectorySpeedProfile",
        {
            "trajectory_id": {"type": "integer"},
            "samples": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "progress_01": {"type": "number"},
                        "speed_m_s": {"type": "number"},
                    },
                    "required": ["progress_01", "speed_m_s"],
                    "additionalProperties": False,
                },
            },
        },
        required=["trajectory_id", "samples"],
    ),
    "ballalgo.TrajectoryTarget": _json_schema(
        "TrajectoryTarget",
        {
            "valid": {"type": "boolean"},
            "active": {"type": "boolean"},
            "grace_hold": {"type": "boolean"},
            "stop_chunk": {"type": "boolean"},
            "scheduled": {"type": "boolean"},
            "trajectory_id": {"type": "integer"},
            "start_time_pi_us": {"type": "integer"},
            "dt_ms": {"type": "integer"},
            "num_actions": {"type": "integer"},
            "action_index": {"type": "integer"},
            "progress_01": {"type": "number"},
            "kick": {"type": "integer"},
            "dribbler_power": {"type": "integer"},
            "vx_global_m_s": {"type": "number"},
            "vy_global_m_s": {"type": "number"},
            "omega_rad_s": {"type": "number"},
            "ax_global_m_s2": {"type": "number"},
            "ay_global_m_s2": {"type": "number"},
            "alpha_rad_s2": {"type": "number"},
            "vx_body_target_m_s": {"type": "number"},
            "vy_body_target_m_s": {"type": "number"},
            "ax_body_target_m_s2": {"type": "number"},
            "ay_body_target_m_s2": {"type": "number"},
        },
        required=["valid", "trajectory_id"],
    ),
    "ballalgo.TeensyRawTelemetry": _json_schema(
        "TeensyRawTelemetry",
        {
            "heading_deg": {"type": "number"},
            "mouse_vx_body_m_s": {"type": "number"},
            "mouse_vy_body_m_s": {"type": "number"},
            "omega_rad_s": {"type": "number"},
            "has_ball": {"type": "boolean"},
            "start_enabled": {"type": "boolean"},
            "goal_is_blue": {"type": "boolean"},
            "mode_override": {"type": "string"},
            "serial_latency_us": {"type": "integer"},
            "telemetry_fresh": {"type": "boolean"},
            "mouse_fresh": {"type": "boolean"},
        },
    ),
    "ballalgo.TrajectoryTrackingError": _json_schema(
        "TrajectoryTrackingError",
        {
            "valid": {"type": "boolean"},
            "trajectory_id": {"type": "integer"},
            "action_index": {"type": "integer"},
            "vx_body_error_m_s": {"type": "number"},
            "vy_body_error_m_s": {"type": "number"},
            "speed_body_error_m_s": {"type": "number"},
            "vx_field_error_m_s": {"type": "number"},
            "vy_field_error_m_s": {"type": "number"},
            "speed_field_error_m_s": {"type": "number"},
            "ax_body_error_m_s2": {"type": "number"},
            "ay_body_error_m_s2": {"type": "number"},
            "ax_field_error_m_s2": {"type": "number"},
            "ay_field_error_m_s2": {"type": "number"},
            "omega_error_rad_s": {"type": "number"},
        },
        required=["valid", "trajectory_id"],
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
