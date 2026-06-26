from __future__ import annotations

import base64
import json
import math
import socket
import struct
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Iterable

from foxglove_sim.config import FoxgloveConfig, load_config


FIELD_WIDTH_MM = 1820.0
FIELD_HEIGHT_MM = 2430.0
FRAME_ID = "field"


def _asdict(value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, dict):
        return value
    return {
        key: getattr(value, key)
        for key in dir(value)
        if not key.startswith("_") and not callable(getattr(value, key))
    }


def _now_ns() -> int:
    return time.time_ns()


def _pack_lidar_points(points: Iterable[Any]) -> str:
    raw = bytearray()
    for point in points:
        angle_cd = getattr(point, "angle_cd", getattr(point, "angleCd", 0))
        distance_mm = getattr(point, "distance_mm", getattr(point, "distanceMm", 0))
        intensity = getattr(point, "intensity", 0)
        angle_rad = float(angle_cd) * math.pi / 18000.0
        distance_m = float(distance_mm) * 0.001
        # Foxglove robot frame: +x forward, +y left, +z up.
        x_m = distance_m * math.cos(angle_rad)
        y_m = -distance_m * math.sin(angle_rad)
        raw.extend(struct.pack("<ffff", x_m, y_m, 0.0, float(intensity) / 255.0))
    return base64.b64encode(raw).decode("ascii")


class FoxgloveRuntimePublisher:
    """Newline-delimited JSON producer for foxglove_sim/sidecar.py."""

    def __init__(self, config_path: str | Path = "foxglove_sim/foxglove.conf"):
        self.config_path = Path(config_path)
        self.cfg: FoxgloveConfig = load_config(self.config_path)
        self._sock: socket.socket | None = None
        self._last_error = ""
        self._frames_sent = 0
        self._last_report_s = 0.0

    @property
    def enabled(self) -> bool:
        return self.cfg.enabled

    @property
    def stream_camera(self) -> bool:
        return self.cfg.stream_camera

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def publish(
        self,
        *,
        detections: Any,
        telemetry: Any,
        pose_state: Any,
        ball_state: Any,
        control_intent: dict[str, Any],
        lidar_points: list[Any] | None = None,
        camera_jpeg_bytes: bytes | None = None,
        loop_count: int = 0,
        dt_s: float = 0.0,
        log_message: str = "",
    ) -> None:
        if not self.cfg.enabled:
            return

        timestamp_ns = _now_ns()
        det = _asdict(detections)
        telemetry_dict = _asdict(telemetry)
        pose = _asdict(pose_state)
        ball = _asdict(ball_state)

        snapshot: dict[str, Any] = {
            "schema_version": 1,
            "timestamp_ns": timestamp_ns,
            "field": {
                "frame_id": FRAME_ID,
                "width_mm": FIELD_WIDTH_MM,
                "height_mm": FIELD_HEIGHT_MM,
            },
            "control_intent": control_intent,
            "teensy_raw": {
                "heading_deg": telemetry_dict.get("heading_deg", 0.0),
                "mouse_vx_body_m_s": telemetry_dict.get("mouse_vx_body_mm_s", 0.0) / 1000.0,
                "mouse_vy_body_m_s": telemetry_dict.get("mouse_vy_body_mm_s", 0.0) / 1000.0,
                "omega_rad_s": telemetry_dict.get("omega_rad_s", 0.0),
                "has_ball": bool(telemetry_dict.get("has_ball", False)),
                "start_enabled": bool(telemetry_dict.get("start_enabled", False)),
                "goal_is_blue": bool(telemetry_dict.get("goal_is_blue", True)),
                "mode_override": telemetry_dict.get("robot_mode", "unknown"),
                "telemetry_fresh": bool(getattr(telemetry, "telemetry_fresh", False)),
                "mouse_fresh": True,
                "mouse_used_by_pi": True,
                "line_angle_deg": telemetry_dict.get("line_angle_deg", -5.0),
                "avoidance_angle_deg": telemetry_dict.get("avoidance_angle_deg", -5.0),
                "chord_length": telemetry_dict.get("chord_length", -5.0),
                "cross_line": bool(telemetry_dict.get("cross_line", False)),
                "camera_ball_angle_deg": telemetry_dict.get("ball_angle_deg", -5.0),
                "camera_ball_distance_cm": telemetry_dict.get("ball_distance_cm", -5.0),
                "camera_blue_goal_angle_deg": telemetry_dict.get("blue_goal_angle_deg", -5.0),
                "camera_yellow_goal_angle_deg": telemetry_dict.get("yellow_goal_angle_deg", -5.0),
                "camera_fresh": bool(telemetry_dict.get("camera_fresh", False)),
            },
        }

        if self.cfg.stream_pose:
            snapshot["pose"] = {
                "valid": bool(pose.get("valid", False)),
                "heading_deg": telemetry_dict.get("heading_deg", 0.0),
                "x_mm": pose.get("x_mm", 0.0),
                "y_mm": pose.get("y_mm", 0.0),
                "vx_mm_s": pose.get("vx_mm_s", 0.0),
                "vy_mm_s": pose.get("vy_mm_s", 0.0),
                "vx_body_m_s": pose.get("vx_body_m_s", 0.0),
                "vy_body_m_s": pose.get("vy_body_m_s", 0.0),
            }

        if self.cfg.stream_ball:
            snapshot["ball"] = self._ball_snapshot(ball, pose, telemetry_dict, det)

        if self.cfg.stream_velocity:
            snapshot["robot_twist"] = {
                "vx_body_m_s": pose.get("vx_body_m_s", 0.0),
                "vy_body_m_s": pose.get("vy_body_m_s", 0.0),
                "speed_body_m_s": math.hypot(pose.get("vx_body_m_s", 0.0), pose.get("vy_body_m_s", 0.0)),
                "vx_field_m_s": pose.get("vx_mm_s", 0.0) / 1000.0,
                "vy_field_m_s": pose.get("vy_mm_s", 0.0) / 1000.0,
                "speed_field_m_s": math.hypot(pose.get("vx_mm_s", 0.0), pose.get("vy_mm_s", 0.0)) / 1000.0,
            }
            snapshot["robot_accel"] = {
                "ax_body_m_s2": 0.0,
                "ay_body_m_s2": 0.0,
                "magnitude_body_m_s2": 0.0,
                "ax_field_m_s2": 0.0,
                "ay_field_m_s2": 0.0,
                "magnitude_field_m_s2": 0.0,
            }
            snapshot["robot_angular"] = {
                "heading_deg": telemetry_dict.get("heading_deg", 0.0),
                "omega_deg_s": math.degrees(telemetry_dict.get("omega_rad_s", 0.0)),
                "alpha_deg_s2": 0.0,
            }
            snapshot["ball_twist"] = {
                "visible": bool(ball.get("visible", False)),
                "vx_body_m_s": ball.get("vx_m_s", 0.0),
                "vy_body_m_s": ball.get("vy_m_s", 0.0),
                "speed_body_m_s": math.hypot(ball.get("vx_m_s", 0.0), ball.get("vy_m_s", 0.0)),
                "vx_field_m_s": 0.0,
                "vy_field_m_s": 0.0,
                "speed_field_m_s": 0.0,
            }

        if self.cfg.stream_lidar and lidar_points:
            snapshot["lidar"] = {
                "frame_id": "robot",
                "n_points": len(lidar_points),
                "data_b64": _pack_lidar_points(lidar_points),
            }

        if self.cfg.stream_camera and camera_jpeg_bytes:
            ball_px = det.get("ball_px", {})
            snapshot["camera"] = {
                "frame_id": "camera",
                "format": "jpeg",
                "data_b64": base64.b64encode(camera_jpeg_bytes).decode("ascii"),
                "ball_px": ball_px,
            }

        if self.cfg.stream_logs:
            snapshot["log"] = {
                "level": 2,
                "name": "ballalgo-python",
                "message": log_message or f"loop={loop_count} dt={dt_s:.4f}s",
            }

        self._send(snapshot)

    def _ball_snapshot(
        self,
        ball: dict[str, Any],
        pose: dict[str, Any],
        telemetry: dict[str, Any],
        detections: dict[str, Any],
    ) -> dict[str, Any]:
        field_visible = bool(ball.get("visible", False) and pose.get("valid", False))
        field_x_mm = 0.0
        field_y_mm = 0.0
        if field_visible:
            heading = math.radians(telemetry.get("heading_deg", 0.0))
            body_x_mm = ball.get("x_m", 0.0) * 1000.0
            body_y_mm = ball.get("y_m", 0.0) * 1000.0
            dx = math.cos(heading) * body_x_mm - math.sin(heading) * body_y_mm
            dy = math.sin(heading) * body_x_mm + math.cos(heading) * body_y_mm
            field_x_mm = pose.get("x_mm", 0.0) + dx
            field_y_mm = pose.get("y_mm", 0.0) + dy

        return {
            "visible": bool(ball.get("visible", False)),
            "field_visible": field_visible,
            "body_x_m": ball.get("x_m", 0.0),
            "body_y_m": ball.get("y_m", 0.0),
            "body_vx_m_s": ball.get("vx_m_s", 0.0),
            "body_vy_m_s": ball.get("vy_m_s", 0.0),
            "field_x_mm": field_x_mm,
            "field_y_mm": field_y_mm,
            "field_vx_m_s": 0.0,
            "field_vy_m_s": 0.0,
            "vision_angle_deg": detections.get("ball_angle_deg", -5.0),
            "vision_dist_cal": detections.get("ball_distance_cm", -5.0),
        }

    def _send(self, snapshot: dict[str, Any]) -> None:
        payload = json.dumps(snapshot, separators=(",", ":")).encode("utf-8") + b"\n"
        if self._sock is None and not self._connect():
            self._maybe_report()
            return
        try:
            self._sock.sendall(payload)
            self._frames_sent += 1
        except OSError as exc:
            self._last_error = str(exc)
            self.close()
            self._maybe_report()

    def _connect(self) -> bool:
        self.close()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(self.cfg.socket_path)
        except OSError as exc:
            self._last_error = str(exc)
            sock.close()
            return False
        self._sock = sock
        self._last_error = ""
        print(f"[Foxglove] connected socket={self.cfg.socket_path}")
        return True

    def _maybe_report(self) -> None:
        now = time.monotonic()
        if now - self._last_report_s < 2.0:
            return
        self._last_report_s = now
        print(
            "[Foxglove] sidecar socket unavailable "
            f"path={self.cfg.socket_path} error={self._last_error or 'none'} "
            f"frames_sent={self._frames_sent}"
        )
