from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import shutil

from matplotlib import animation as mpl_animation
from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

from .artifact import SimulationArtifact


@dataclass(frozen=True)
class AnimationOptions:
    fps: int = 24
    speed: float = 1.0
    tail_seconds: float = 1.0
    freeze_final_seconds: float = 1.0


def _set_field_axes(ax: Axes, artifact: SimulationArtifact) -> None:
    min_x, max_x, min_y, max_y = artifact.field_bounds_cm()
    ax.set_xlim(min_x - 10.0, max_x + 10.0)
    ax.set_ylim(min_y - 10.0, max_y + 10.0)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    ax.set_xlabel("Field X (centered cm)")
    ax.set_ylabel("Field Y (centered cm)")
    ax.add_patch(
        Rectangle((min_x, min_y), max_x - min_x, max_y - min_y, fill=False, linewidth=2.0)
    )


def _series(samples: list[dict], key: str) -> list[float]:
    return [float(sample[key]) for sample in samples]


def _ball_positions(artifact: SimulationArtifact, trace: list[dict]) -> tuple[list[float], list[float]]:
    start_x, start_y = artifact.ball_field_cm()
    vx, vy = artifact.ball_velocity_cm_s()
    xs: list[float] = []
    ys: list[float] = []
    for sample in trace:
        t_s = float(sample["t_s"])
        xs.append(start_x + vx * t_s)
        ys.append(start_y + vy * t_s)
    return xs, ys


def _writer_for_output(output: Path, fps: int):
    suffix = output.suffix.lower()
    if suffix == ".gif":
        return mpl_animation.PillowWriter(fps=fps)
    if suffix == ".mp4":
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg is required for .mp4 output but is not available")
        return mpl_animation.FFMpegWriter(fps=fps, bitrate=2400)
    raise RuntimeError(f"unsupported animation output format: {output.suffix}")


def build_animation(
    artifact: SimulationArtifact,
    *,
    fps: int = 24,
    speed: float = 1.0,
    tail_seconds: float = 1.0,
    freeze_final_seconds: float = 1.0,
) -> tuple[Figure, mpl_animation.FuncAnimation]:
    trace = artifact.sim_trace
    if len(trace) < 2:
        raise RuntimeError("artifact trace is too short to animate")

    actions = artifact.actions
    ball_x_cm, ball_y_cm = _ball_positions(artifact, trace)
    times = _series(trace, "t_s")
    vx_body = _series(actions, "vx_body_mps")
    vy_body = _series(actions, "vy_body_mps")
    ax_body = _series(actions, "ax_body_mps2")
    ay_body = _series(actions, "ay_body_mps2")
    action_times = _series(actions, "t_s")
    ball_vx_cm_s, ball_vy_cm_s = artifact.ball_velocity_cm_s()
    ball_vx_mps = ball_vx_cm_s / 100.0
    ball_vy_mps = ball_vy_cm_s / 100.0

    fig = plt.figure(figsize=(14, 8))
    grid = fig.add_gridspec(2, 2, width_ratios=(1.4, 1.0), height_ratios=(1.0, 1.0))
    field_ax = fig.add_subplot(grid[:, 0])
    velocity_ax = fig.add_subplot(grid[0, 1])
    accel_ax = fig.add_subplot(grid[1, 1])

    _set_field_axes(field_ax, artifact)
    field_ax.set_title("Top-Down Simulation")
    start_x, start_y = artifact.start_pose_cm()
    goal_x, goal_y = (
        artifact.goal_target_cm() if artifact.mode == "production_ball_plan" else artifact.goal_pose_cm()
    )
    goal_color = "#d4a800" if artifact.goal_identity() == "yellow" else "#1f77b4"
    field_ax.scatter([start_x], [start_y], color="#1f77b4", s=60, label="start")
    field_ax.scatter([goal_x], [goal_y], color=goal_color, s=90, marker="*", label="goal")
    if artifact.mode == "production_ball_plan":
        field_ax.plot(ball_x_cm, ball_y_cm, color="#f28e2b", alpha=0.2, linewidth=1.0)

    robot_trail = Line2D([], [], color="#ff7f0e", linewidth=2.2, label="robot trace")
    field_ax.add_line(robot_trail)
    planned_path = Line2D([], [], color="#2ca02c", linewidth=1.4, alpha=0.55, label="planned path")
    field_ax.add_line(planned_path)

    robot_body = Circle((start_x, start_y), radius=3.0, facecolor="#ff7f0e", edgecolor="black", lw=1.0)
    ball_body = Circle((ball_x_cm[0], ball_y_cm[0]), radius=1.8, facecolor="#f28e2b", edgecolor="black", lw=0.8)
    field_ax.add_patch(robot_body)
    field_ax.add_patch(ball_body)

    heading_arrow = FancyArrowPatch((start_x, start_y), (start_x, start_y), color="black", arrowstyle="->", mutation_scale=12, lw=1.5)
    velocity_arrow = FancyArrowPatch((start_x, start_y), (start_x, start_y), color="#2ca02c", arrowstyle="->", mutation_scale=12, lw=1.8)
    accel_arrow = FancyArrowPatch((start_x, start_y), (start_x, start_y), color="#d62728", arrowstyle="->", mutation_scale=12, lw=1.8)
    field_ax.add_patch(heading_arrow)
    field_ax.add_patch(velocity_arrow)
    field_ax.add_patch(accel_arrow)
    time_text = field_ax.text(0.02, 0.98, "", transform=field_ax.transAxes, va="top")
    field_ax.legend(loc="upper right")

    velocity_ax.plot(action_times, vx_body, color="#1f77b4", label="robot vx")
    velocity_ax.plot(action_times, vy_body, color="#ff7f0e", label="robot vy")
    if artifact.mode == "production_ball_plan":
        velocity_ax.axhline(ball_vx_mps, color="#4e79a7", linestyle="--", alpha=0.7, label="ball vx")
        velocity_ax.axhline(ball_vy_mps, color="#e15759", linestyle="--", alpha=0.7, label="ball vy")
    velocity_cursor = velocity_ax.axvline(action_times[0] if action_times else 0.0, color="black", alpha=0.5)
    velocity_ax.set_title("Velocity")
    velocity_ax.set_xlabel("Time (s)")
    velocity_ax.set_ylabel("m/s")
    velocity_ax.grid(True, alpha=0.25)
    velocity_ax.legend(loc="upper right")

    accel_ax.plot(action_times, ax_body, color="#2ca02c", label="robot ax")
    accel_ax.plot(action_times, ay_body, color="#d62728", label="robot ay")
    accel_cursor = accel_ax.axvline(action_times[0] if action_times else 0.0, color="black", alpha=0.5)
    accel_ax.set_title("Acceleration")
    accel_ax.set_xlabel("Time (s)")
    accel_ax.set_ylabel("m/s^2")
    accel_ax.grid(True, alpha=0.25)
    accel_ax.legend(loc="upper right")

    tail_count = max(2, int(round(max(tail_seconds, 0.1) * fps)))
    hold_frames = max(0, int(round(max(freeze_final_seconds, 0.0) * fps)))
    base_frames = len(trace)
    playback_indices: list[int] = []
    cursor = 0.0
    speed = max(speed, 1e-6)
    while cursor < base_frames:
        playback_indices.append(min(int(cursor), base_frames - 1))
        cursor += speed
    frame_count = len(playback_indices) + hold_frames

    def _path_for_replan(replan_index: int) -> tuple[list[float], list[float]]:
        if not artifact.replans:
            return [], []
        replan_index = max(0, min(replan_index, len(artifact.replans) - 1))
        path_samples = artifact.replans[replan_index].get("path", [])
        xs = [float(sample["x_mm"]) * 0.1 - float(artifact.field["width_mm"]) * 0.05 for sample in path_samples]
        ys = [float(sample["y_mm"]) * 0.1 - float(artifact.field["height_mm"]) * 0.05 for sample in path_samples]
        return xs, ys

    def _arrow_endpoint(x: float, y: float, dx: float, dy: float, scale: float) -> tuple[float, float]:
        return x + dx * scale, y + dy * scale

    def update(frame_index: int):
        if frame_index < len(playback_indices):
            sample_index = playback_indices[frame_index]
        else:
            sample_index = base_frames - 1
        sample = trace[sample_index]
        x_cm = float(sample["x_cm"])
        y_cm = float(sample["y_cm"])
        robot_body.center = (x_cm, y_cm)
        ball_body.center = (ball_x_cm[sample_index], ball_y_cm[sample_index])

        trail_start = max(0, sample_index - tail_count)
        robot_trail.set_data(_series(trace[trail_start:sample_index + 1], "x_cm"),
                             _series(trace[trail_start:sample_index + 1], "y_cm"))

        replan_index = int(sample.get("replan_index", 0))
        path_x, path_y = _path_for_replan(replan_index)
        planned_path.set_data(path_x, path_y)

        heading_rad = math.radians(float(sample["heading_deg"]))
        heading_end = _arrow_endpoint(x_cm, y_cm, math.cos(heading_rad), math.sin(heading_rad), 8.0)
        heading_arrow.set_positions((x_cm, y_cm), heading_end)

        vel_end = _arrow_endpoint(
            x_cm,
            y_cm,
            float(sample["vx_field_mps"]),
            float(sample["vy_field_mps"]),
            20.0,
        )
        velocity_arrow.set_positions((x_cm, y_cm), vel_end)

        heading_cos = math.cos(heading_rad)
        heading_sin = math.sin(heading_rad)
        accel_field_x = (
            heading_cos * float(sample["ax_body_mps2"]) - heading_sin * float(sample["ay_body_mps2"])
        )
        accel_field_y = (
            heading_sin * float(sample["ax_body_mps2"]) + heading_cos * float(sample["ay_body_mps2"])
        )
        accel_end = _arrow_endpoint(
            x_cm,
            y_cm,
            accel_field_x,
            accel_field_y,
            4.0,
        )
        accel_arrow.set_positions((x_cm, y_cm), accel_end)

        current_time = float(sample["t_s"])
        velocity_cursor.set_xdata([current_time, current_time])
        accel_cursor.set_xdata([current_time, current_time])
        time_text.set_text(f"t = {current_time:.2f}s")

        return (
            robot_trail,
            planned_path,
            robot_body,
            ball_body,
            heading_arrow,
            velocity_arrow,
            accel_arrow,
            velocity_cursor,
            accel_cursor,
            time_text,
        )

    interval_ms = 1000.0 / max(1, fps)
    anim = mpl_animation.FuncAnimation(
        fig,
        update,
        frames=frame_count,
        interval=interval_ms,
        blit=False,
        repeat=False,
    )
    fig.suptitle(f"BallAlgo Simulation Animation: {artifact.raw['label']}", fontsize=14)
    return fig, anim


def save_animation(
    artifact: SimulationArtifact,
    output_path: str | Path,
    *,
    fps: int = 24,
    speed: float = 1.0,
    tail_seconds: float = 1.0,
    freeze_final_seconds: float = 1.0,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, anim = build_animation(
        artifact,
        fps=fps,
        speed=speed,
        tail_seconds=tail_seconds,
        freeze_final_seconds=freeze_final_seconds,
    )
    try:
        anim.save(output, writer=_writer_for_output(output, fps))
    finally:
        plt.close(fig)
    return output
