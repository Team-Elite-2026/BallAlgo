from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FoxgloveTopic:
    name: str
    schema: str
    panel: str
    nominal_hz: float | None
    producer: str
    purpose: str


TOPICS: tuple[FoxgloveTopic, ...] = (
    FoxgloveTopic(
        name="/session/info",
        schema="ballalgo.SessionInfo",
        panel="Raw Messages panel",
        nominal_hz=0.01,
        producer="foxglove sidecar startup",
        purpose="Record session metadata, config snapshot, and recording path for replay/debug traceability.",
    ),
    FoxgloveTopic(
        name="/field/scene/static",
        schema="foxglove.SceneUpdate",
        panel="3D panel (2D mode)",
        nominal_hz=0.1,
        producer="startup/session init",
        purpose="Publish static field geometry, goal geometry, and any always-on overlays once per session.",
    ),
    FoxgloveTopic(
        name="/field/scene/live",
        schema="foxglove.SceneUpdate",
        panel="3D panel (2D mode)",
        nominal_hz=30.0,
        producer="foxglove sidecar",
        purpose="Render flat 2D robot, heading arrow, ball marker, and live overlays on the field without Foxglove's default 3D pose model.",
    ),
    FoxgloveTopic(
        name="/planner/scene/path",
        schema="foxglove.SceneUpdate",
        panel="3D panel (2D mode)",
        nominal_hz=15.0,
        producer="planner",
        purpose="Visualize generated splines/trajectories and the active target pose on the flat field scene.",
    ),
    FoxgloveTopic(
        name="/robot/pose",
        schema="foxglove.PoseInFrame",
        panel="Raw Messages or debug panel",
        nominal_hz=30.0,
        producer="pose estimator",
        purpose="Keep a raw robot pose topic available for logging and inspection; the 2D field view should use /field/scene/live.",
    ),
    FoxgloveTopic(
        name="/robot/twist",
        schema="ballalgo.RobotLinearVelocity",
        panel="Plot panel",
        nominal_hz=30.0,
        producer="motion stack",
        purpose="Robot linear velocity components for plotting and replay debugging.",
    ),
    FoxgloveTopic(
        name="/robot/accel",
        schema="ballalgo.RobotLinearAcceleration",
        panel="Plot panel",
        nominal_hz=30.0,
        producer="motion stack",
        purpose="Robot linear acceleration components for plotting and replay debugging.",
    ),
    FoxgloveTopic(
        name="/robot/angular",
        schema="ballalgo.RobotAngularKinematics",
        panel="Plot panel",
        nominal_hz=30.0,
        producer="motion stack",
        purpose="Angular velocity and acceleration telemetry for the robot.",
    ),
    FoxgloveTopic(
        name="/ball/pose",
        schema="foxglove.PoseInFrame",
        panel="Raw Messages or debug panel",
        nominal_hz=30.0,
        producer="ball tracker",
        purpose="Keep a raw ball pose topic available for logging and inspection; the 2D field view should use /field/scene/live.",
    ),
    FoxgloveTopic(
        name="/ball/twist",
        schema="ballalgo.BallVelocity",
        panel="Plot panel",
        nominal_hz=30.0,
        producer="ball tracker",
        purpose="Ball velocity telemetry for plotting and replay.",
    ),
    FoxgloveTopic(
        name="/ball/range",
        schema="ballalgo.BallRange",
        panel="Plot panel",
        nominal_hz=30.0,
        producer="ball tracker",
        purpose="Ball angle, calibrated distance, and body-frame position — "
                "the primary graphable source for ball proximity and bearing.",
    ),
    FoxgloveTopic(
        name="/lidar/points",
        schema="foxglove.PointCloud",
        panel="3D panel (2D or 3D)",
        nominal_hz=10.0,
        producer="LiDAR pipeline",
        purpose="Visualize live or downsampled LiDAR returns.",
    ),
    FoxgloveTopic(
        name="/lidar/localization/scene",
        schema="foxglove.SceneUpdate",
        panel="3D panel (2D mode)",
        nominal_hz=10.0,
        producer="LiDAR localizer",
        purpose="Show extracted walls, landmarks, or localization hypotheses.",
    ),
    FoxgloveTopic(
        name="/camera/front/image",
        schema="foxglove.CompressedImage",
        panel="Image panel",
        nominal_hz=15.0,
        producer="camera pipeline",
        purpose="Live camera stream for debugging detections and scene understanding.",
    ),
    FoxgloveTopic(
        name="/camera/front/annotations",
        schema="foxglove.ImageAnnotations",
        panel="Image panel",
        nominal_hz=15.0,
        producer="vision pipeline",
        purpose="Bounding boxes, centroids, labels, and other 2D overlays emitted alongside the camera stream when detections exist.",
    ),
    FoxgloveTopic(
        name="/debug/log",
        schema="foxglove.Log",
        panel="Log panel",
        nominal_hz=None,
        producer="runtime event stream",
        purpose="Human-readable event log for state transitions, planner updates, and runtime warnings, plus a low-rate status heartbeat.",
    ),
)

TOPICS_BY_NAME = {topic.name: topic for topic in TOPICS}
