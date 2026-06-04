from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import subprocess
from contextlib import ExitStack, nullcontext
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from config import FoxgloveConfig, load_config
from schema_catalog import SCHEMAS

try:
    import foxglove
    from foxglove import Channel, Schema
    from foxglove.channels import LogChannel, PoseInFrameChannel, SceneUpdateChannel
    from foxglove.messages import (
        ArrowPrimitive,
        Color,
        LinePrimitive,
        Log,
        LogLevel,
        Point3,
        Pose,
        PoseInFrame,
        Quaternion,
        SceneEntity,
        SceneUpdate,
        SpherePrimitive,
        TextPrimitive,
        Timestamp,
        Vector3,
    )
except ImportError:  # pragma: no cover - keeps the file importable before SDK install
    foxglove = None
    Channel = Schema = object
    LogChannel = PoseInFrameChannel = SceneUpdateChannel = object
    ArrowPrimitive = Color = LinePrimitive = Log = LogLevel = object
    Point3 = Pose = PoseInFrame = Quaternion = object
    SceneEntity = SceneUpdate = SpherePrimitive = TextPrimitive = object
    Timestamp = Vector3 = object


LOG_LEVELS = {
    0: getattr(LogLevel, "Unknown", 0),
    1: getattr(LogLevel, "Debug", 1),
    2: getattr(LogLevel, "Info", 2),
    3: getattr(LogLevel, "Warning", 3),
    4: getattr(LogLevel, "Error", 4),
    5: getattr(LogLevel, "Fatal", 5),
}


def _timestamp_from_ns(timestamp_ns: int) -> Any:
    sec = timestamp_ns // 1_000_000_000
    nsec = timestamp_ns % 1_000_000_000
    return Timestamp(sec=sec, nsec=nsec)


def _yaw_to_quaternion(yaw_deg: float) -> Any:
    half = math.radians(yaw_deg) * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _pose_from_mm(x_mm: float, y_mm: float, heading_deg: float) -> Any:
    return Pose(
        position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=0.0),
        orientation=_yaw_to_quaternion(heading_deg),
    )


def _point_from_mm(x_mm: float, y_mm: float) -> Any:
    return Point3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=0.0)


def _arrow_from_components(x_mm: float, y_mm: float, vx_m_s: float, vy_m_s: float, color: Any) -> Any:
    magnitude = math.hypot(vx_m_s, vy_m_s)
    if magnitude < 1e-6:
        return None
    yaw_deg = math.degrees(math.atan2(vy_m_s, vx_m_s))
    return ArrowPrimitive(
        pose=Pose(
            position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=0.0),
            orientation=_yaw_to_quaternion(yaw_deg),
        ),
        shaft_length=magnitude,
        shaft_diameter=0.012,
        head_length=0.04,
        head_diameter=0.025,
        color=color,
    )


def _git_sha(repo_root: Path) -> str:
    try:
        return (
            subprocess.check_output(
                ["git", "-C", str(repo_root), "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL,
                text=True,
            )
            .strip()
        )
    except Exception:
        return ""


class BallAlgoFoxgloveSidecar:
    def __init__(self, repo_root: Path, config_path: Path, session_label: str | None, note: str):
        self.repo_root = repo_root
        self.config_path = config_path
        self.cfg: FoxgloveConfig = load_config(config_path)
        self.session_label = session_label or datetime.now(timezone.utc).strftime("ballalgo-%Y%m%d-%H%M%S")
        self.note = note
        self.running = True
        self.socket: socket.socket | None = None
        self.server: Any | None = None
        self.recording_path: Path | None = None
        self.static_scene_published = False
        self.channels: dict[str, Any] = {}

    def _require_sdk(self) -> None:
        if foxglove is None:
            raise RuntimeError(
                "foxglove-sdk is not installed. Install it with `pip install foxglove-sdk` or "
                "use the repo's foxglove_sim/requirements.txt."
            )

    def _init_channels(self) -> None:
        self.channels = {
            "/field/scene/static": SceneUpdateChannel("/field/scene/static"),
            "/planner/scene/path": SceneUpdateChannel("/planner/scene/path"),
            "/robot/pose": PoseInFrameChannel("/robot/pose"),
            "/ball/pose": PoseInFrameChannel("/ball/pose"),
            "/debug/log": LogChannel("/debug/log"),
            "/session/info": Channel(
                topic="/session/info",
                message_encoding="json",
                schema=Schema(
                    name="ballalgo.SessionInfo",
                    encoding="jsonschema",
                    data=SCHEMAS["ballalgo.SessionInfo"],
                ),
            ),
            "/robot/twist": Channel(
                topic="/robot/twist",
                message_encoding="json",
                schema=Schema(
                    name="ballalgo.RobotLinearVelocity",
                    encoding="jsonschema",
                    data=SCHEMAS["ballalgo.RobotLinearVelocity"],
                ),
            ),
            "/robot/accel": Channel(
                topic="/robot/accel",
                message_encoding="json",
                schema=Schema(
                    name="ballalgo.RobotLinearAcceleration",
                    encoding="jsonschema",
                    data=SCHEMAS["ballalgo.RobotLinearAcceleration"],
                ),
            ),
            "/robot/angular": Channel(
                topic="/robot/angular",
                message_encoding="json",
                schema=Schema(
                    name="ballalgo.RobotAngularKinematics",
                    encoding="jsonschema",
                    data=SCHEMAS["ballalgo.RobotAngularKinematics"],
                ),
            ),
            "/ball/twist": Channel(
                topic="/ball/twist",
                message_encoding="json",
                schema=Schema(
                    name="ballalgo.BallVelocity",
                    encoding="jsonschema",
                    data=SCHEMAS["ballalgo.BallVelocity"],
                ),
            ),
        }

    def _make_recording_path(self) -> Path:
        record_dir = (self.repo_root / self.cfg.record_dir).resolve()
        record_dir.mkdir(parents=True, exist_ok=True)
        return record_dir / f"{self.session_label}.mcap"

    def _bind_socket(self) -> None:
        path = Path(self.cfg.socket_path)
        if path.exists():
            path.unlink()
        path.parent.mkdir(parents=True, exist_ok=True)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.bind(str(path))
        sock.settimeout(0.25)
        self.socket = sock

    def _publish_session_info(self) -> None:
        message = {
            "session_label": self.session_label,
            "started_at": datetime.now(timezone.utc).isoformat(),
            "git_sha": _git_sha(self.repo_root),
            "config_path": str(self.config_path),
            "recording_path": str(self.recording_path) if self.recording_path else "",
            "note": self.note,
            "config": asdict(self.cfg),
        }
        self.channels["/session/info"].log(message)

    def _publish_static_scene(self, field: dict[str, Any], timestamp_ns: int) -> None:
        if self.static_scene_published:
            return

        width_m = float(field["width_mm"]) / 1000.0
        height_m = float(field["height_mm"]) / 1000.0
        half_w = width_m * 0.5
        half_h = height_m * 0.5
        ts = _timestamp_from_ns(timestamp_ns)

        border_points = [
            Point3(x=-half_w, y=-half_h, z=0.0),
            Point3(x=half_w, y=-half_h, z=0.0),
            Point3(x=half_w, y=half_h, z=0.0),
            Point3(x=-half_w, y=half_h, z=0.0),
            Point3(x=-half_w, y=-half_h, z=0.0),
        ]
        center_line = [
            Point3(x=0.0, y=-half_h, z=0.0),
            Point3(x=0.0, y=half_h, z=0.0),
        ]

        scene = SceneUpdate(
            entities=[
                SceneEntity(
                    timestamp=ts,
                    frame_id=field["frame_id"],
                    id="field-border",
                    lines=[
                        LinePrimitive(
                            thickness=0.02,
                            scale_invariant=False,
                            points=border_points,
                            color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        )
                    ],
                ),
                SceneEntity(
                    timestamp=ts,
                    frame_id=field["frame_id"],
                    id="field-center-line",
                    lines=[
                        LinePrimitive(
                            thickness=0.01,
                            scale_invariant=False,
                            points=center_line,
                            color=Color(r=0.8, g=0.8, b=0.8, a=1.0),
                        )
                    ],
                ),
            ]
        )
        self.channels["/field/scene/static"].log(scene)
        self.static_scene_published = True

    def _publish_pose(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        pose = snapshot.get("pose")
        if not pose or not pose.get("valid"):
            return
        self.channels["/robot/pose"].log(
            PoseInFrame(
                timestamp=_timestamp_from_ns(timestamp_ns),
                frame_id=snapshot["field"]["frame_id"],
                pose=_pose_from_mm(pose["x_mm"], pose["y_mm"], pose["heading_deg"]),
            )
        )

    def _publish_ball(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ball = snapshot.get("ball")
        if not ball or not ball.get("field_visible"):
            return
        self.channels["/ball/pose"].log(
            PoseInFrame(
                timestamp=_timestamp_from_ns(timestamp_ns),
                frame_id=snapshot["field"]["frame_id"],
                pose=_pose_from_mm(ball["field_x_mm"], ball["field_y_mm"], 0.0),
            )
        )

    def _publish_path(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        planner = snapshot.get("planner_path")
        if not planner:
            return
        points = [_point_from_mm(point["x_mm"], point["y_mm"]) for point in planner.get("points", [])]
        entities = []
        ts = _timestamp_from_ns(timestamp_ns)
        frame_id = snapshot["field"]["frame_id"]

        if points:
            entities.append(
                SceneEntity(
                    timestamp=ts,
                    frame_id=frame_id,
                    id="planner-path",
                    lines=[
                        LinePrimitive(
                            thickness=0.03,
                            scale_invariant=False,
                            points=points,
                            color=Color(r=0.1, g=0.8, b=0.3, a=1.0),
                        )
                    ],
                )
            )

        target_pose = _pose_from_mm(
            planner["target_x_mm"], planner["target_y_mm"], planner["target_heading_deg"]
        )
        target_arrow = _arrow_from_components(
            planner["target_x_mm"],
            planner["target_y_mm"],
            0.18 * math.cos(math.radians(planner["target_heading_deg"])),
            0.18 * math.sin(math.radians(planner["target_heading_deg"])),
            Color(r=1.0, g=0.2, b=0.1, a=1.0),
        )
        entities.append(
            SceneEntity(
                timestamp=ts,
                frame_id=frame_id,
                id="planner-target",
                spheres=[
                    SpherePrimitive(
                        pose=target_pose,
                        size=Vector3(x=0.05, y=0.05, z=0.05),
                        color=Color(r=1.0, g=0.2, b=0.1, a=0.95),
                    )
                ],
                arrows=[target_arrow] if target_arrow is not None else [],
                texts=[
                    TextPrimitive(
                        pose=Pose(
                            position=Vector3(
                                x=planner["target_x_mm"] / 1000.0,
                                y=planner["target_y_mm"] / 1000.0,
                                z=0.06,
                            ),
                            orientation=_yaw_to_quaternion(0.0),
                        ),
                        text=f"traj {planner['trajectory_id']}",
                        font_size=18.0,
                        scale_invariant=True,
                        billboard=True,
                        color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                    )
                ],
            )
        )

        self.channels["/planner/scene/path"].log(SceneUpdate(entities=entities))

    def _publish_numeric(self, snapshot: dict[str, Any]) -> None:
        if self.cfg.stream_velocity:
            for topic, key in (
                ("/robot/twist", "robot_twist"),
                ("/robot/accel", "robot_accel"),
                ("/robot/angular", "robot_angular"),
                ("/ball/twist", "ball_twist"),
            ):
                message = snapshot.get(key)
                if message:
                    self.channels[topic].log(message)

    def _publish_log(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        payload = snapshot.get("log")
        if not payload:
            return
        self.channels["/debug/log"].log(
            Log(
                timestamp=_timestamp_from_ns(timestamp_ns),
                level=LOG_LEVELS.get(int(payload.get("level", 2)), LOG_LEVELS[2]),
                name=str(payload.get("name", "ballalgo")),
                message=str(payload.get("message", "")),
            )
        )

    def _handle_snapshot(self, snapshot: dict[str, Any]) -> None:
        timestamp_ns = int(snapshot["timestamp_ns"])
        self._publish_static_scene(snapshot["field"], timestamp_ns)
        if self.cfg.stream_pose:
            self._publish_pose(snapshot, timestamp_ns)
        if self.cfg.stream_ball:
            self._publish_ball(snapshot, timestamp_ns)
        if self.cfg.stream_paths:
            self._publish_path(snapshot, timestamp_ns)
        self._publish_numeric(snapshot)
        if self.cfg.stream_logs:
            self._publish_log(snapshot, timestamp_ns)

    def stop(self, *_args: object) -> None:
        self.running = False

    def run(self) -> None:
        self._require_sdk()
        if not self.cfg.enabled:
            print("Foxglove streaming disabled in config.")
            return

        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)

        foxglove.set_log_level("INFO")
        with ExitStack() as stack:
            if self.cfg.record_mcap:
                self.recording_path = self._make_recording_path()
                stack.enter_context(foxglove.open_mcap(str(self.recording_path)))
            else:
                stack.enter_context(nullcontext())

            self.server = foxglove.start_server(host=self.cfg.websocket_host, port=self.cfg.websocket_port)
            self._init_channels()
            self._bind_socket()
            self._publish_session_info()

            print(
                f"Foxglove sidecar listening on ws://{self.cfg.websocket_host}:{self.cfg.websocket_port} "
                f"and unix://{self.cfg.socket_path}"
            )

            try:
                assert self.socket is not None
                while self.running:
                    try:
                        data, _addr = self.socket.recvfrom(1 << 20)
                    except socket.timeout:
                        continue
                    snapshot = json.loads(data.decode("utf-8"))
                    self._handle_snapshot(snapshot)
            finally:
                if self.socket is not None:
                    self.socket.close()
                socket_path = Path(self.cfg.socket_path)
                if socket_path.exists():
                    socket_path.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BallAlgo Foxglove live-debugging sidecar")
    parser.add_argument(
        "--config",
        default="foxglove_sim/foxglove.conf",
        help="Path to the shared Foxglove config file.",
    )
    parser.add_argument("--session-label", default="", help="Optional recording/session label override.")
    parser.add_argument("--note", default="", help="Optional operator note stored in session metadata.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = (repo_root / config_path).resolve()
    sidecar = BallAlgoFoxgloveSidecar(
        repo_root=repo_root,
        config_path=config_path,
        session_label=args.session_label or None,
        note=args.note,
    )
    sidecar.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
