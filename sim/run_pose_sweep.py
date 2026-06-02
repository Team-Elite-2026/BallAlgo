#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from ballalgo_sim import load_artifact


def frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    current = start
    epsilon = abs(step) * 1e-6
    if step == 0:
        raise ValueError("step must be non-zero")
    if step > 0:
        while current <= stop + epsilon:
            values.append(round(current, 6))
            current += step
    else:
        while current >= stop - epsilon:
            values.append(round(current, 6))
            current += step
    return values


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a grid sweep of BallAlgo offline simulations.")
    parser.add_argument("--bench", required=True, help="Path to the compiled ballalgo_sim-compatible executable.")
    parser.add_argument("--output-dir", required=True, help="Directory for generated artifacts and summary.")
    parser.add_argument(
        "--mode",
        choices=["production_ball_plan", "pose_target", "single_chunk", "rolling_replan"],
        default="production_ball_plan",
        help="Simulation mode forwarded to the executable.",
    )
    parser.add_argument("--start-x-min-cm", type=float, required=True)
    parser.add_argument("--start-x-max-cm", type=float, required=True)
    parser.add_argument("--start-y-min-cm", type=float, required=True)
    parser.add_argument("--start-y-max-cm", type=float, required=True)
    parser.add_argument("--step-cm", type=float, required=True)
    parser.add_argument("--start-heading-deg", type=float, default=0.0)
    parser.add_argument("--control-hz", type=float)
    parser.add_argument("--max-replans", type=int)
    parser.add_argument("--max-sim-time-s", type=float)

    parser.add_argument("--goal-x-cm", type=float)
    parser.add_argument("--goal-y-cm", type=float)
    parser.add_argument("--goal-heading-deg", type=float, default=0.0)
    parser.add_argument("--ball-x-cm", type=float)
    parser.add_argument("--ball-y-cm", type=float)
    parser.add_argument("--ball-vx-cm-s", type=float, default=0.0)
    parser.add_argument("--ball-vy-cm-s", type=float, default=0.0)
    parser.add_argument("--goal-target-x-cm", type=float)
    parser.add_argument("--goal-target-y-cm", type=float)
    return parser


def require_production_args(args: argparse.Namespace) -> None:
    missing = [
        name
        for name in ("ball_x_cm", "ball_y_cm", "goal_target_x_cm", "goal_target_y_cm")
        if getattr(args, name) is None
    ]
    if missing:
        raise SystemExit(f"production_ball_plan requires: {', '.join('--' + name.replace('_', '-') for name in missing)}")


def require_pose_target_args(args: argparse.Namespace) -> None:
    missing = [name for name in ("goal_x_cm", "goal_y_cm") if getattr(args, name) is None]
    if missing:
        raise SystemExit(f"pose target modes require: {', '.join('--' + name.replace('_', '-') for name in missing)}")


def append_optional(command: list[str], flag: str, value: float | int | None) -> None:
    if value is None:
        return
    command.extend([flag, str(value)])


def main() -> int:
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    production_mode = args.mode == "production_ball_plan"
    if production_mode:
        require_production_args(args)
    else:
        require_pose_target_args(args)

    cases: list[dict] = []
    for start_x_cm in frange(args.start_x_min_cm, args.start_x_max_cm, args.step_cm):
        for start_y_cm in frange(args.start_y_min_cm, args.start_y_max_cm, args.step_cm):
            label = f"sx_{start_x_cm:g}_sy_{start_y_cm:g}"
            artifact_path = output_dir / f"{label}.json"
            command = [
                args.bench,
                "--start-x-cm",
                str(start_x_cm),
                "--start-y-cm",
                str(start_y_cm),
                "--start-heading-deg",
                str(args.start_heading_deg),
                "--mode",
                args.mode,
                "--output",
                str(artifact_path),
                "--label",
                label,
            ]
            append_optional(command, "--control-hz", args.control_hz)
            append_optional(command, "--max-replans", args.max_replans)
            append_optional(command, "--max-sim-time-s", args.max_sim_time_s)

            if production_mode:
                command.extend(
                    [
                        "--ball-x-cm",
                        str(args.ball_x_cm),
                        "--ball-y-cm",
                        str(args.ball_y_cm),
                        "--ball-vx-cm-s",
                        str(args.ball_vx_cm_s),
                        "--ball-vy-cm-s",
                        str(args.ball_vy_cm_s),
                        "--goal-target-x-cm",
                        str(args.goal_target_x_cm),
                        "--goal-target-y-cm",
                        str(args.goal_target_y_cm),
                    ]
                )
            else:
                command.extend(
                    [
                        "--goal-x-cm",
                        str(args.goal_x_cm),
                        "--goal-y-cm",
                        str(args.goal_y_cm),
                        "--goal-heading-deg",
                        str(args.goal_heading_deg),
                    ]
                )

            subprocess.run(command, check=True)
            artifact = load_artifact(artifact_path)
            cases.append(
                {
                    "label": label,
                    "artifact": str(artifact_path),
                    "start_x_cm": start_x_cm,
                    "start_y_cm": start_y_cm,
                    "mode": artifact.mode,
                    "final_target_error_mm": artifact.summary["final_target_error_mm"],
                    "executed_action_count": artifact.summary["executed_action_count"],
                    "sim_duration_s": artifact.summary["sim_duration_s"],
                }
            )

    summary_path = output_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump({"cases": cases}, handle, indent=2)

    print(f"wrote {len(cases)} cases to {output_dir}")
    print(f"summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
