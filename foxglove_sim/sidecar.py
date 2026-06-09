"""BallAlgo Foxglove live-debugging sidecar.

Architecture
------------
The C++ binary (ballalgo) connects to a Unix socket and streams newline-delimited
JSON snapshots at up to 60 Hz.  This process receives those snapshots, routes each
field to the appropriate Foxglove channel, and broadcasts everything over a
WebSocket server that Foxglove Studio connects to (optionally also recording MCAP).

Channel design — two patterns:
  DirectChannel  — snapshot key → channel, zero transformation, table-driven.
                   Adding a new plottable signal is a one-liner in DIRECT_CHANNELS.
  Custom publish — pose/ball/path/camera need coordinate transforms or binary
                   encoding, so they get their own _publish_* method.
"""
from __future__ import annotations

import argparse
import base64
import json
import select
import signal
import socket
import subprocess
from contextlib import ExitStack, nullcontext
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from config import FoxgloveConfig, load_config
from field_geometry import FieldGeometry, center_field_mm
from schema_catalog import SCHEMAS

try:
    import struct

    import foxglove
    from foxglove import Channel, Schema
    from foxglove.channels import (
        CompressedImageChannel,
        LogChannel,
        PointCloudChannel,
        PoseInFrameChannel,
        SceneUpdateChannel,
    )
    from foxglove.messages import (
        Color,
        Log,
        LogLevel,
        PackedElementField,
        PackedElementFieldNumericType,
        PointCloud,
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
    # ImageAnnotations support is optional — gracefully absent in older SDK builds.
    try:
        from foxglove.channels import ImageAnnotationsChannel
        from foxglove.messages import CircleAnnotation, ImageAnnotations, Point2
        _HAS_IMAGE_ANNOTATIONS = True
    except ImportError:
        ImageAnnotationsChannel = None
        CircleAnnotation = ImageAnnotations = Point2 = None
        _HAS_IMAGE_ANNOTATIONS = False

    _F32 = PackedElementFieldNumericType.Float32
    _LIDAR_FIELDS = [
        PackedElementField(name="x",         offset=0,  type=_F32),
        PackedElementField(name="y",         offset=4,  type=_F32),
        PackedElementField(name="z",         offset=8,  type=_F32),
        PackedElementField(name="intensity", offset=12, type=_F32),
    ]

except ImportError:  # pragma: no cover
    struct = None
    foxglove = None
    Channel = Schema = object
    CompressedImageChannel = LogChannel = PoseInFrameChannel = SceneUpdateChannel = object
    PointCloudChannel = object
    ImageAnnotationsChannel = None
    Log = LogLevel = object
    Color = Pose = PoseInFrame = Quaternion = object
    SceneUpdate = Timestamp = Vector3 = object
    PackedElementField = PackedElementFieldNumericType = PointCloud = object
    CircleAnnotation = ImageAnnotations = Point2 = None
    _HAS_IMAGE_ANNOTATIONS = False
    _LIDAR_FIELDS = []
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


# ---------------------------------------------------------------------------
# DirectChannel — table-driven pass-through publishing
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class DirectChannel:
    """Maps a snapshot key directly to a Foxglove channel with no transformation.

    To add a new plottable signal:
      1. Add its schema to schema_catalog.py.
      2. Add a DirectChannel entry to DIRECT_CHANNELS below.
      3. Make sure the C++ publisher (or _enrich_snapshot) emits that key.
    """
    topic: str
    snapshot_key: str
    config_attr: str   # FoxgloveConfig boolean attribute that gates publishing
    schema_name: str   # key in schema_catalog.SCHEMAS


DIRECT_CHANNELS: tuple[DirectChannel, ...] = (
    DirectChannel("/robot/twist",   "robot_twist",   "stream_velocity", "ballalgo.RobotLinearVelocity"),
    DirectChannel("/robot/accel",   "robot_accel",   "stream_velocity", "ballalgo.RobotLinearAcceleration"),
    DirectChannel("/robot/angular", "robot_angular", "stream_velocity", "ballalgo.RobotAngularKinematics"),
    DirectChannel("/ball/twist",    "ball_twist",    "stream_velocity", "ballalgo.BallVelocity"),
    DirectChannel("/ball/range",    "ball_range",    "stream_ball",     "ballalgo.BallRange"),
    DirectChannel("/planner/profile", "planner_profile", "stream_paths", "ballalgo.TrajectorySpeedProfile"),
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _timestamp_from_ns(timestamp_ns: int) -> Any:
    return Timestamp(sec=timestamp_ns // 1_000_000_000, nsec=timestamp_ns % 1_000_000_000)


def _pose_from_mm(x_mm: float, y_mm: float, heading_deg: float) -> Any:
    return Pose(
        position=Vector3(x=x_mm / 1000.0, y=y_mm / 1000.0, z=0.0),
        orientation=yaw_to_quaternion(heading_deg),
    )


def _git_sha(repo_root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(repo_root), "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return ""


# ---------------------------------------------------------------------------
# Sidecar
# ---------------------------------------------------------------------------

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
        self.snapshot_count = 0
        self.last_snapshot_ns = 0
        self.last_transport_report_ns = 0

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _require_sdk(self) -> None:
        if foxglove is None:
            raise RuntimeError(
                "foxglove-sdk is not installed. "
                "Run: pip install foxglove-sdk  (or use foxglove_sim/requirements.txt)"
            )

    def _warn_unimplemented_streams(self) -> None:
        unimplemented: list[str] = []
        if unimplemented:
            print(
                f"Warning: stream flags enabled but not yet implemented in sidecar: "
                f"{', '.join(unimplemented)}"
            )
        if self.cfg.stream_camera and not _HAS_IMAGE_ANNOTATIONS:
            print(
                "Warning: foxglove-sdk image annotation support is unavailable; "
                "camera frames will stream without overlay annotations."
            )

    def _init_channels(self) -> None:
        # Scene / pose channels (typed)
        self.channels = {
            "/field/scene/static":  SceneUpdateChannel("/field/scene/static"),
            "/field/scene/live":    SceneUpdateChannel("/field/scene/live"),
            "/planner/scene/path":  SceneUpdateChannel("/planner/scene/path"),
            "/robot/pose":          PoseInFrameChannel("/robot/pose"),
            "/ball/pose":           PoseInFrameChannel("/ball/pose"),
            "/lidar/scan":          PointCloudChannel("/lidar/scan"),
            "/debug/log":           LogChannel("/debug/log"),
            "/session/info": Channel(
                topic="/session/info",
                message_encoding="json",
                schema=Schema(name="ballalgo.SessionInfo", encoding="jsonschema",
                              data=SCHEMAS["ballalgo.SessionInfo"]),
            ),
        }
        # Camera channels (typed)
        self.channels["/camera/front/image"] = CompressedImageChannel("/camera/front/image")
        if _HAS_IMAGE_ANNOTATIONS:
            self.channels["/camera/front/annotations"] = ImageAnnotationsChannel(
                "/camera/front/annotations"
            )
        # Direct (JSON schema) channels — built from the table
        for ch in DIRECT_CHANNELS:
            self.channels[ch.topic] = Channel(
                topic=ch.topic,
                message_encoding="json",
                schema=Schema(name=ch.schema_name, encoding="jsonschema",
                              data=SCHEMAS[ch.schema_name]),
            )

    # ------------------------------------------------------------------
    # Socket I/O
    # ------------------------------------------------------------------

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
        if readable:
            client, _addr = self.server_socket.accept()
            client.setblocking(False)
            self._close_client()
            self.client_socket = client
            print("BallAlgo runtime connected to Foxglove sidecar socket.")

    def _drain_snapshots(self) -> list[dict[str, Any]]:
        """Read and parse every complete newline-framed JSON snapshot in the buffer."""
        if self.client_socket is None:
            return []
        readable, _, _ = select.select([self.client_socket], [], [], 0.25)
        if not readable:
            return []
        chunk = self.client_socket.recv(1 << 20)
        if not chunk:
            print("BallAlgo runtime disconnected from Foxglove sidecar socket.")
            self._close_client()
            return []
        self.client_buffer.extend(chunk)
        snapshots: list[dict[str, Any]] = []
        while True:
            newline_index = self.client_buffer.find(b"\n")
            if newline_index < 0:
                break
            payload = bytes(self.client_buffer[:newline_index])
            del self.client_buffer[:newline_index + 1]
            snapshots.append(json.loads(payload.decode("utf-8")))
        return snapshots

    # ------------------------------------------------------------------
    # Snapshot enrichment — derive computed keys for direct channels
    # ------------------------------------------------------------------

    def _enrich_snapshot(self, snapshot: dict[str, Any]) -> None:
        """Add derived sub-keys so DirectChannel can log them without custom code."""
        ball = snapshot.get("ball")
        if ball and ball.get("visible"):
            snapshot["ball_range"] = {
                "visible": ball["visible"],
                "angle_deg": ball.get("vision_angle_deg", 0.0),
                "dist_cal_m": ball.get("vision_dist_cal", 0.0),
                "body_x_m": ball.get("body_x_m", 0.0),
                "body_y_m": ball.get("body_y_m", 0.0),
            }

    # ------------------------------------------------------------------
    # Publish methods
    # ------------------------------------------------------------------

    def _publish_session_info(self) -> None:
        self.channels["/session/info"].log({
            "session_label": self.session_label,
            "started_at": datetime.now(timezone.utc).isoformat(),
            "git_sha": _git_sha(self.repo_root),
            "config_path": str(self.config_path),
            "recording_path": str(self.recording_path) if self.recording_path else "",
            "note": self.note,
            "config": asdict(self.cfg),
        })

    def _publish_static_scene(self, field: dict[str, Any], timestamp_ns: int) -> None:
        if self.last_static_scene_ns and timestamp_ns - self.last_static_scene_ns < 2_000_000_000:
            return
        ts = _timestamp_from_ns(timestamp_ns)
        self.channels["/field/scene/static"].log(
            SceneUpdate(entities=build_static_scene_entities(field, ts))
        )
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
        self.channels["/robot/pose"].log(PoseInFrame(
            timestamp=_timestamp_from_ns(timestamp_ns),
            frame_id=snapshot["field"]["frame_id"],
            pose=_pose_from_mm(cx_mm, cy_mm, pose["heading_deg"]),
        ))

    def _publish_ball(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ball = snapshot.get("ball")
        if not ball or not ball.get("field_visible"):
            return
        field = FieldGeometry.from_snapshot(snapshot["field"])
        cx_mm, cy_mm = center_field_mm(ball["field_x_mm"], ball["field_y_mm"], field)
        self.channels["/ball/pose"].log(PoseInFrame(
            timestamp=_timestamp_from_ns(timestamp_ns),
            frame_id=snapshot["field"]["frame_id"],
            pose=_pose_from_mm(cx_mm, cy_mm, 0.0),
        ))

    def _publish_path(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        ts = _timestamp_from_ns(timestamp_ns)
        entities = build_path_scene_entities(snapshot, ts)
        if entities:
            self.channels["/planner/scene/path"].log(SceneUpdate(entities=entities))

    def _publish_direct_channels(self, snapshot: dict[str, Any]) -> None:
        """Publish all table-driven pass-through channels in one loop."""
        for ch in DIRECT_CHANNELS:
            if not getattr(self.cfg, ch.config_attr, False):
                continue
            message = snapshot.get(ch.snapshot_key)
            if message:
                self.channels[ch.topic].log(message)

    def _publish_log(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        payload = snapshot.get("log")
        if not payload:
            return
        self.channels["/debug/log"].log(Log(
            timestamp=_timestamp_from_ns(timestamp_ns),
            level=LOG_LEVELS.get(int(payload.get("level", 2)), LOG_LEVELS[2]),
            name=str(payload.get("name", "ballalgo")),
            message=str(payload.get("message", "")),
        ))

    def _publish_lidar(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        lidar = snapshot.get("lidar")
        if not lidar or not lidar.get("data_b64"):
            return
        raw = base64.b64decode(lidar["data_b64"])
        self.channels["/lidar/scan"].log(PointCloud(
            timestamp=_timestamp_from_ns(timestamp_ns),
            frame_id=lidar.get("frame_id", "robot"),
            point_stride=16,
            fields=_LIDAR_FIELDS,
            data=raw,
        ))

    def _publish_camera(self, snapshot: dict[str, Any], timestamp_ns: int) -> None:
        camera = snapshot.get("camera")
        if not camera or not camera.get("data_b64"):
            return
        ts = _timestamp_from_ns(timestamp_ns)
        from foxglove.messages import CompressedImage
        self.channels["/camera/front/image"].log(CompressedImage(
            timestamp=ts,
            frame_id=camera.get("frame_id", "camera"),
            format=camera.get("format", "jpeg"),
            data=base64.b64decode(camera["data_b64"]),
        ))
        if not _HAS_IMAGE_ANNOTATIONS:
            return
        ball_px = camera.get("ball_px")
        if ball_px and ball_px.get("found"):
            self.channels["/camera/front/annotations"].log(ImageAnnotations(
                timestamp=ts,
                circles=[CircleAnnotation(
                    timestamp=ts,
                    position=Point2(x=float(ball_px["cx"]), y=float(ball_px["cy"])),
                    diameter=40.0,   # visual size in pixels; adjust to taste
                    thickness=2.5,
                    outline_color=Color(r=1.0, g=0.55, b=0.1, a=1.0),
                    fill_color=Color(r=1.0, g=0.55, b=0.1, a=0.25),
                )],
            ))

    # ------------------------------------------------------------------
    # Main snapshot handler
    # ------------------------------------------------------------------

    def _handle_snapshot(self, snapshot: dict[str, Any]) -> None:
        self._enrich_snapshot(snapshot)
        timestamp_ns = int(snapshot["timestamp_ns"])
        self.snapshot_count += 1
        self.last_snapshot_ns = timestamp_ns

        self._publish_static_scene(snapshot["field"], timestamp_ns)
        if self.cfg.stream_pose or self.cfg.stream_ball:
            self._publish_live_scene(snapshot, timestamp_ns)
        if self.cfg.stream_pose:
            self._publish_pose(snapshot, timestamp_ns)
        if self.cfg.stream_ball:
            self._publish_ball(snapshot, timestamp_ns)
        if self.cfg.stream_paths:
            self._publish_path(snapshot, timestamp_ns)
        self._publish_direct_channels(snapshot)
        if self.cfg.stream_lidar:
            self._publish_lidar(snapshot, timestamp_ns)
        if self.cfg.stream_logs:
            self._publish_log(snapshot, timestamp_ns)
        if self.cfg.stream_camera:
            self._publish_camera(snapshot, timestamp_ns)

    def _maybe_report_transport_health(self) -> None:
        now_ns = datetime.now(timezone.utc).timestamp() * 1_000_000_000
        now_ns = int(now_ns)
        if self.last_transport_report_ns and now_ns - self.last_transport_report_ns < 2_000_000_000:
            return
        self.last_transport_report_ns = now_ns

        seconds_since_snapshot = None
        if self.last_snapshot_ns:
            seconds_since_snapshot = (now_ns - self.last_snapshot_ns) / 1_000_000_000.0

        print(
            "Foxglove transport: "
            f"runtime_connected={'yes' if self.client_socket is not None else 'no'} "
            f"snapshots={self.snapshot_count} "
            f"seconds_since_last_snapshot="
            f"{seconds_since_snapshot:.2f}" if seconds_since_snapshot is not None else
            "Foxglove transport: "
            f"runtime_connected={'yes' if self.client_socket is not None else 'no'} "
            f"snapshots={self.snapshot_count} "
            "seconds_since_last_snapshot=never"
        )

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

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

            self.server = (
                foxglove.start_server(host=self.cfg.websocket_host, port=self.cfg.websocket_port)
                if self.cfg.websocket_enabled
                else None
            )
            self._init_channels()
            self._bind_socket()
            self._publish_session_info()

            websocket_desc = (
                f"ws://{self.cfg.websocket_host}:{self.cfg.websocket_port}"
                if self.cfg.websocket_enabled
                else "websocket disabled"
            )
            print(f"Foxglove sidecar  websocket={websocket_desc}  socket={self.cfg.socket_path}")

            try:
                while self.running:
                    self._accept_client_if_ready()
                    for snapshot in self._drain_snapshots():
                        self._handle_snapshot(snapshot)
                    self._maybe_report_transport_health()
            finally:
                self._close_client()
                if self.server_socket is not None:
                    self.server_socket.close()
                socket_path = Path(self.cfg.socket_path)
                if socket_path.exists():
                    socket_path.unlink()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BallAlgo Foxglove live-debugging sidecar")
    parser.add_argument("--config", default="foxglove_sim/foxglove.conf",
                        help="Path to the shared Foxglove config file.")
    parser.add_argument("--session-label", default="",
                        help="Optional recording/session label override.")
    parser.add_argument("--note", default="",
                        help="Optional operator note stored in session metadata.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent.parent
    config_path = Path(args.config)
    if not config_path.is_absolute():
        config_path = (repo_root / config_path).resolve()
    BallAlgoFoxgloveSidecar(
        repo_root=repo_root,
        config_path=config_path,
        session_label=args.session_label or None,
        note=args.note,
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
