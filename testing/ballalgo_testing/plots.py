from __future__ import annotations

import math
from typing import Iterable

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle

from .artifact import PlannerArtifact


def _series(samples: Iterable[dict], key: str) -> list[float]:
    return [float(sample[key]) for sample in samples]


def _mm_to_centered_cm_x(artifact: PlannerArtifact, value_mm: float) -> float:
    return value_mm * 0.1 - float(artifact.field["width_mm"]) * 0.05


def _mm_to_centered_cm_y(artifact: PlannerArtifact, value_mm: float) -> float:
    return value_mm * 0.1 - float(artifact.field["height_mm"]) * 0.05


def _set_field_axes(ax: Axes, artifact: PlannerArtifact) -> None:
    min_x, max_x, min_y, max_y = artifact.field_bounds_cm()
    ax.set_xlim(min_x - 10, max_x + 10)
    ax.set_ylim(min_y - 10, max_y + 10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("Field X (centered cm)")
    ax.set_ylabel("Field Y (centered cm)")


def plot_field_overview(
    ax: Axes,
    artifact: PlannerArtifact,
    *,
    quiver_stride: int = 5,
    show_action_directions: bool = True,
) -> None:
    trace_label = "executed trace" if artifact.mode == "rolling_replan" else "perfect-state sim"
    min_x, max_x, min_y, max_y = artifact.field_bounds_cm()
    ax.add_patch(
        Rectangle((min_x, min_y), max_x - min_x, max_y - min_y, fill=False, linewidth=2.0)
    )

    _set_field_axes(ax, artifact)

    start_x, start_y = artifact.start_pose_cm()
    goal_x, goal_y = artifact.goal_pose_cm()
    ax.scatter([start_x], [start_y], color="#1f77b4", s=80, label="start")
    ax.scatter([goal_x], [goal_y], color="#d62728", s=80, marker="*", label="goal")

    if artifact.replans:
        replan_start_x = []
        replan_start_y = []
        for index, replan in enumerate(artifact.replans):
            start_pose = replan.get("start_pose")
            if start_pose:
                replan_start_x.append(float(start_pose["x_cm"]))
                replan_start_y.append(float(start_pose["y_cm"]))
            waypoints = replan.get("waypoints", [])
            if waypoints:
                waypoint_x_cm = [_mm_to_centered_cm_x(artifact, float(sample["x_mm"])) for sample in waypoints]
                waypoint_y_cm = [_mm_to_centered_cm_y(artifact, float(sample["y_mm"])) for sample in waypoints]
                ax.plot(
                    waypoint_x_cm,
                    waypoint_y_cm,
                    color="#7f7f7f",
                    linewidth=0.8,
                    alpha=0.12,
                    label="A* waypoints" if index == 0 else None,
                )

            path_samples = replan.get("path", [])
            if path_samples:
                path_x_cm = [_mm_to_centered_cm_x(artifact, float(sample["x_mm"])) for sample in path_samples]
                path_y_cm = [_mm_to_centered_cm_y(artifact, float(sample["y_mm"])) for sample in path_samples]
                ax.plot(
                    path_x_cm,
                    path_y_cm,
                    color="#2ca02c",
                    linewidth=1.4 if index + 1 == len(artifact.replans) else 1.0,
                    alpha=0.22 if index + 1 != len(artifact.replans) else 0.9,
                    label="replanned paths" if index == 0 else None,
                )
        if replan_start_x:
            ax.scatter(
                replan_start_x,
                replan_start_y,
                color="#2ca02c",
                s=16,
                alpha=0.35,
                label="replan starts",
            )
    else:
        if artifact.waypoints:
            waypoint_x_cm = [_mm_to_centered_cm_x(artifact, float(sample["x_mm"])) for sample in artifact.waypoints]
            waypoint_y_cm = [_mm_to_centered_cm_y(artifact, float(sample["y_mm"])) for sample in artifact.waypoints]
            ax.plot(
                waypoint_x_cm,
                waypoint_y_cm,
                color="#7f7f7f",
                linewidth=1.0,
                alpha=0.6,
                label="A* waypoints",
            )

        if artifact.path_samples:
            path_x_cm = [_mm_to_centered_cm_x(artifact, float(sample["x_mm"])) for sample in artifact.path_samples]
            path_y_cm = [_mm_to_centered_cm_y(artifact, float(sample["y_mm"])) for sample in artifact.path_samples]
            ax.plot(path_x_cm, path_y_cm, color="#2ca02c", linewidth=2.0, label="planned path")

    if artifact.sim_trace:
        trace_x = _series(artifact.sim_trace, "x_cm")
        trace_y = _series(artifact.sim_trace, "y_cm")
        ax.plot(trace_x, trace_y, color="#ff7f0e", linewidth=2.0, label=trace_label)

        if show_action_directions and len(artifact.sim_trace) > 1:
            stride = max(1, quiver_stride)
            samples = artifact.sim_trace[1::stride]
            qx = _series(samples, "x_cm")
            qy = _series(samples, "y_cm")
            qu = _series(samples, "vx_field_mps")
            qv = _series(samples, "vy_field_mps")
            ax.quiver(qx, qy, qu, qv, angles="xy", scale_units="xy", scale=1.5, width=0.003)

    ax.set_title("Path Overview")
    ax.legend(loc="best")


def plot_error_timeseries(ax: Axes, artifact: PlannerArtifact) -> None:
    goal = artifact.input["goal"]
    goal_x_mm = float(goal["x_mm"])
    goal_y_mm = float(goal["y_mm"])
    times = _series(artifact.sim_trace, "t_s")
    errors = [
        math.hypot(float(sample["x_mm"]) - goal_x_mm, float(sample["y_mm"]) - goal_y_mm)
        for sample in artifact.sim_trace
    ]
    ax.plot(times, errors, color="#d62728", linewidth=2.0)
    ax.set_title("Position Error")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (mm)")
    ax.grid(True, alpha=0.25)


def plot_velocity_panels(ax: Axes, artifact: PlannerArtifact) -> None:
    actions = artifact.actions
    times = _series(actions, "t_s")
    ax.plot(times, _series(actions, "vx_body_mps"), label="vx_body")
    ax.plot(times, _series(actions, "vy_body_mps"), label="vy_body")
    ax.plot(times, _series(actions, "speed_body_mps"), label="speed_body", linewidth=2.0)
    ax.set_title("Body Velocity Commands")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def plot_acceleration_panels(ax: Axes, artifact: PlannerArtifact) -> None:
    actions = artifact.actions
    times = _series(actions, "t_s")
    ax.plot(times, _series(actions, "ax_body_mps2"), label="ax_body")
    ax.plot(times, _series(actions, "ay_body_mps2"), label="ay_body")
    ax.plot(times, _series(actions, "accel_body_mps2"), label="accel_body", linewidth=2.0)
    ax.set_title("Body Acceleration Commands")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def plot_heading_panel(ax: Axes, artifact: PlannerArtifact) -> None:
    trace = artifact.sim_trace
    times = _series(trace, "t_s")
    headings = _series(trace, "heading_deg")
    ax.plot(times, headings, label="heading")
    ax.axhline(float(artifact.input["goal"]["heading_deg"]), color="#d62728", linestyle="--", label="goal")
    ax.set_title("Heading Trace")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heading (deg)")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def plot_angular_command_panel(ax: Axes, artifact: PlannerArtifact) -> None:
    actions = artifact.actions
    times = _series(actions, "t_s")
    ax.plot(times, _series(actions, "omega_rad_s"), label="omega")
    ax.plot(times, _series(actions, "alpha_rad_s2"), label="alpha")
    ax.set_title("Angular Commands")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular command")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")


def save_or_show(fig: plt.Figure, output: str | None) -> None:
    fig.tight_layout()
    if output:
        fig.savefig(output, dpi=180, bbox_inches="tight")
        return
    plt.show()
