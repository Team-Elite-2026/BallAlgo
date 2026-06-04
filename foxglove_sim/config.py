from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class FoxgloveConfig:
    enabled: bool = True
    websocket_enabled: bool = True
    record_mcap: bool = True
    stream_paths: bool = True
    stream_pose: bool = True
    stream_ball: bool = True
    stream_velocity: bool = True
    stream_lidar: bool = False
    stream_camera: bool = False
    stream_annotations: bool = False
    stream_logs: bool = True

    paths_hz: float = 15.0
    pose_hz: float = 30.0
    ball_hz: float = 30.0
    velocity_hz: float = 30.0
    logs_hz: float = 1.0
    camera_hz: float = 10.0

    socket_path: str = "/tmp/ballalgo_foxglove.sock"
    websocket_host: str = "0.0.0.0"
    websocket_port: int = 8765
    record_dir: str = "foxglove_sim/recordings"


def _parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def load_config(path: str | Path) -> FoxgloveConfig:
    cfg = FoxgloveConfig()
    path = Path(path)
    if not path.exists():
        return cfg

    parsers = {
        "enabled": _parse_bool,
        "websocket_enabled": _parse_bool,
        "record_mcap": _parse_bool,
        "stream_paths": _parse_bool,
        "stream_pose": _parse_bool,
        "stream_ball": _parse_bool,
        "stream_velocity": _parse_bool,
        "stream_lidar": _parse_bool,
        "stream_camera": _parse_bool,
        "stream_annotations": _parse_bool,
        "stream_logs": _parse_bool,
        "paths_hz": float,
        "pose_hz": float,
        "ball_hz": float,
        "velocity_hz": float,
        "logs_hz": float,
        "camera_hz": float,
        "socket_path": str,
        "websocket_host": str,
        "websocket_port": int,
        "record_dir": str,
    }

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or "=" not in line:
            continue
        key, value = [part.strip() for part in line.split("=", 1)]
        parser = parsers.get(key)
        if parser is None:
            continue
        setattr(cfg, key, parser(value))

    return cfg
