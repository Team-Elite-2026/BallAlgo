#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")

from matplotlib import pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ballalgo_sim import load_artifact
from ballalgo_sim.animation import save_animation
from ballalgo_sim.plots import (
    plot_acceleration_panels,
    plot_angular_command_panel,
    plot_error_timeseries,
    plot_field_overview,
    plot_heading_panel,
    plot_velocity_panels,
    save_or_show,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run ballalgo_sim and generate the standard plots and animation."
    )
    default_sim = SCRIPT_DIR.parent / "build-test" / "ballalgo_sim"
    parser.add_argument("--sim-bin", default=str(default_sim), help="Path to ballalgo_sim.")
    parser.add_argument("--skip-animation", action="store_true", help="Do not render a GIF.")
    parser.add_argument("--animation-ext", default=".gif", choices=[".gif", ".mp4"])
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--tail-seconds", type=float, default=1.0)
    parser.add_argument("--freeze-final-seconds", type=float, default=1.0)
    parser.add_argument(
        "sim_args",
        nargs=argparse.REMAINDER,
        help="Arguments forwarded to ballalgo_sim. Prefix with -- before the sim args.",
    )
    return parser


def _artifact_path_from_args(sim_args: list[str]) -> Path:
    for index, arg in enumerate(sim_args):
        if arg == "--output" and index + 1 < len(sim_args):
            return Path(sim_args[index + 1])
    raise RuntimeError("forwarded simulation args must include --output <artifact.json>")


def _render_plan_plot(artifact_path: Path) -> Path:
    artifact = load_artifact(artifact_path)
    output = artifact_path.with_name(f"{artifact_path.stem}_plan.png")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    plot_field_overview(axes[0, 0], artifact)
    plot_error_timeseries(axes[0, 1], artifact)
    plot_velocity_panels(axes[1, 0], artifact)
    plot_heading_panel(axes[1, 1], artifact)
    fig.suptitle(f"BallAlgo Simulation Overview: {artifact.raw['label']}", fontsize=14)
    save_or_show(fig, str(output))
    plt.close(fig)
    return output


def _render_chunk_plot(artifact_path: Path) -> Path:
    artifact = load_artifact(artifact_path)
    output = artifact_path.with_name(f"{artifact_path.stem}_chunks.png")
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    plot_velocity_panels(axes[0], artifact)
    plot_acceleration_panels(axes[1], artifact)
    plot_angular_command_panel(axes[2], artifact)
    fig.suptitle(f"BallAlgo Chunk Signals: {artifact.raw['label']}", fontsize=14)
    save_or_show(fig, str(output))
    plt.close(fig)
    return output


def main() -> int:
    args = build_parser().parse_args()
    sim_args = list(args.sim_args)
    if sim_args and sim_args[0] == "--":
        sim_args = sim_args[1:]
    if not sim_args:
        raise SystemExit("pass the ballalgo_sim arguments after --")

    artifact_path = _artifact_path_from_args(sim_args)
    subprocess.run([args.sim_bin, *sim_args], check=True)

    plan_output = _render_plan_plot(artifact_path)
    chunk_output = _render_chunk_plot(artifact_path)
    print(f"wrote plot: {plan_output}")
    print(f"wrote plot: {chunk_output}")

    if not args.skip_animation:
        artifact = load_artifact(artifact_path)
        animation_output = artifact_path.with_suffix(args.animation_ext)
        save_animation(
            artifact,
            animation_output,
            fps=max(1, args.fps),
            speed=max(1e-6, args.speed),
            tail_seconds=max(0.05, args.tail_seconds),
            freeze_final_seconds=max(0.0, args.freeze_final_seconds),
        )
        print(f"wrote animation: {animation_output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
