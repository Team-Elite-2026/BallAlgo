from __future__ import annotations

import argparse
import json
import select
import signal
import socket
import subprocess
from contextlib import ExitStack, nullcontext
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from config import FoxgloveConfig, load_config
from field_geometry import FieldGeometry, center_field_mm
from schema_catalog import SCHEMAS

try:
    import foxglove
    from foxglove import Channel, Schema
    from foxglove.channels import LogChannel, PoseInFrameChannel, SceneUpdateChannel
    from foxglove.messages import (
        Log,
        LogLevel,
        Pose,
        PoseInFrame,
        Quaternion,
        SceneUpdate,
        Timestamp,
        Vector3,
    )
    from scene_builder import (
        build_live_scene_entities,
        build_path_scene_entities,
        build_static_scene_entities,
        yaw_to_quaternion,
    )
except ImportError:  # pragma: no cover - keeps the file importable before SDK install
    foxglove = None
    Channel = Schema = object
    LogChannel = PoseInFrameChannel = SceneUpdateChannel = object
    Log = LogLevel = object
    Pose = PoseInFrame = Quaternion = object
    SceneUpdate = object
    Timestamp = Vector3 = object
    build_live_scene_entities = build_path_scene_entities = build_static_scene_entities = None
    yaw_to_quaternion = None


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


def _pose_from_mm(x_mm: float, y_mm: float, heading_deg: float) -> Any:
    return Pose(
        position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=0.0),
        orientation=yaw_to_quaternion(heading_deg),
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
        self.server_socket: socket.socket | None = None
        self.client_socket: socket.socket | None = None
        self.client_buffer = bytearray()
        self.server: Any | None = None
        self.recording_path: Path | None = None
        self.last_static_scene_ns = 0
        self.channels: dict[str, Any] = {}

    def _require_sdk(self) -> None:
        if foxglove is None:
            raise RuntimeError(
                "foxglove-sdk is not installed. Install it with `pip install foxglove-sdk` or "
                "use the repo's foxglove_sim/requirements.txt."
            )

    def _warn_unimplemented_streams(self) -> None:
        unimplemented = [
            name
            for flag, name in (
                (self.cfg.stream_lidar, "stream_lidar"),
                (self.cfg.stream_camera, "stream_camera"),
                (self.cfg.stream_annotations, "stream_annotations"),
            )
            if flag
        ]
        if unimplemented:
            print(
                f"Warning: these stream flags are enabled in config but not yet implemented "
                f"in the sidecar: {', '.join(unimplemented)}"
            )

    def _init_channels(self) -> None:
        self.channels = {
            "/field/scene/static": SceneUpdateChannel("/field/scene/static"),
            "/field/scene/live": SceneUpdateChannel("/field/scene/live"),
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
        candidate = record_dir / f"{self.session_label}.mcap"
        if not candidate.exists():
            return candidate
        suffix = 1
        while True:
            candidate = record_dir / f"{self.session_label}-{suffix}.mcap"
            if not candidate.exists():
                return candidate
            suffix += 1

    def _bind_socket(self) -> None:
        path = Path(self.cfg.socket_path)
        if path.exists():
            path.unlink()
        path.parent.mkdir(parents=True, exist_ok=True)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(str(path))
        sock.listen(1)
        sock.setblocking(False)
        self.server_socket = sock

    def _close_client(self) -> None:
        if self.client_socket is not None:
            self.client_socket.close()
            self.client_socket = None
        self.client_buffer.clear()

    def _accept_client_if_ready(self) -> None:
        if self.server_socket is None:
            return
        readable, _, _ = select.select([self.server_socket], [], [], 0.0)
        if not readable:
            return
        client, _addr = self.server_socket.accept()
        client.setblocking(False)
        self._close_client()
        self.client_socket = client

    def _drain_snapshots(self) -> list[dict[str, Any]]:
        """Read all complete newline-framed JSON snapshots from the socket buffer."""
        if self.client_socket is None:
            return []
        readable, _, _ = select.select([self.client_socket], [], [], 0.25)
        if not readable:
            return []
        chunk = self.client_socket.recv(1 << 20)
        if not chunk:
            self._close_client()
            return []
        self.client_buffer.extend(chunk)

        snapshots: list[dict[str, Any]] = []
        while True:
            newline_index = self.client_buffer.find(b"\n")
            if newline_index < 0:
                break
            payload = bytes(self.client_buffer[:newline_index])
            del self.client_buffer[: newline_index + 1]
            snapshots.append(json.loads(payload.decode("utf-8")))
        return snapshots

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
        if self.last_static_scene_ns and timestamp_ns - self.last_static_scene_ns < 2_000_000_000:
            return

        ts = _timestamp_from_ns(timestamp_ns)
        scene = SceneUpdate(entities=build_static_scene_entities(field, ts))
        self.channels["/field/scene/static"].log(scene)
        self.last_static_scene_ns = timestamp_ns

    def _publish_live_scene(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ts = _timestamp_from_ns(timestamp_ns)
        entities = build_live_scene_entities(snapshot, ts)
        if entities:
            self.channels["/field/scene/live"].log(SceneUpdate(entities=entities))

    def _publish_pose(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        pose = snapshot.get("pose")
        if not pose or not pose.get("valid"):
            return
        field = FieldGeometry.from_snapshot(snapshot["field"])
        cx_mm, cy_mm = center_field_mm(pose["x_mm"], pose["y_mm"], field)
        self.channels["/robot/pose"].log(
            PoseInFrame(
                timestamp=_timestamp_from_ns(timestamp_ns),
                frame_id=snapshot["field"]["frame_id"],
                pose=_pose_from_mm(cx_mm, cy_mm, pose["heading_deg"]),
            )
        )

    def _publish_ball(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ball = snapshot.get("ball")
        if not ball or not ball.get("field_visible"):
            return
        field = FieldGeometry.from_snapshot(snapshot["field"])
        cx_mm, cy_mm = center_field_mm(ball["field_x_mm"], ball["field_y_mm"], field)
        self.channels["/ball/pose"].log(
            PoseInFrame(
                timestamp=_timestamp_from_ns(timestamp_ns),
                frame_id=snapshot["field"]["frame_id"],
                pose=_pose_from_mm(cx_mm, cy_mm, 0.0),
            )
        )

    def _publish_path(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ts = _timestamp_from_ns(timestamp_ns)
        entities = build_path_scene_entities(snapshot, ts)
        if entities:
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
        if self.cfg.stream_pose or self.cfg.stream_ball:
            self._publish_live_scene(snapshot, timestamp_ns)
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

        self._warn_unimplemented_streams()
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)

        foxglove.set_log_level("INFO")
        with ExitStack() as stack:
            if self.cfg.record_mcap:
                self.recording_path = self._make_recording_path()
                stack.enter_context(foxglove.open_mcap(str(self.recording_path)))
            else:
                stack.enter_context(nullcontext())

            if self.cfg.websocket_enabled:
                self.server = foxglove.start_server(host=self.cfg.websocket_host, port=self.cfg.websocket_port)
            else:
                self.server = None
            self._init_channels()
            self._bind_socket()
            self._publish_session_info()

            websocket_desc = (
                f"ws://{self.cfg.websocket_host}:{self.cfg.websocket_port}"
                if self.cfg.websocket_enabled
                else "websocket disabled"
            )
            print(f"Foxglove sidecar listening on {websocket_desc} and unix://{self.cfg.socket_path}")

            try:
                while self.running:
                    self._accept_client_if_ready()
                    for snapshot in self._drain_snapshots():
                        self._handle_snapshot(snapshot)
            finally:
                self._close_client()
                if self.server_socket is not None:
                    self.server_socket.close()
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
