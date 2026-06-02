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

from ballalgo_testing import load_artifact


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
    parser = argparse.ArgumentParser(description="Run a grid sweep of perfect-state planner simulations.")
    parser.add_argument("--bench", required=True, help="Path to the compiled ballalgo_planner_bench executable.")
    parser.add_argument("--output-dir", required=True, help="Directory for generated artifacts and summary.")
    parser.add_argument(
        "--mode",
        choices=["single_chunk", "rolling_replan"],
        default="rolling_replan",
        help="Simulation mode forwarded to the planner bench.",
    )
    parser.add_argument("--goal-x-cm", type=float, required=True)
    parser.add_argument("--goal-y-cm", type=float, required=True)
    parser.add_argument("--goal-heading-deg", type=float, default=0.0)
    parser.add_argument("--start-x-min-cm", type=float, required=True)
    parser.add_argument("--start-x-max-cm", type=float, required=True)
    parser.add_argument("--start-y-min-cm", type=float, required=True)
    parser.add_argument("--start-y-max-cm", type=float, required=True)
    parser.add_argument("--step-cm", type=float, required=True)
    parser.add_argument("--start-heading-deg", type=float, default=0.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

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
                "--goal-x-cm",
                str(args.goal_x_cm),
                "--goal-y-cm",
                str(args.goal_y_cm),
                "--goal-heading-deg",
                str(args.goal_heading_deg),
                "--output",
                str(artifact_path),
                "--label",
                label,
            ]
            subprocess.run(command, check=True)
            artifact = load_artifact(artifact_path)
            cases.append(
                {
                    "label": label,
                    "artifact": str(artifact_path),
                    "start_x_cm": start_x_cm,
                    "start_y_cm": start_y_cm,
                    "final_position_error_mm": artifact.summary["final_position_error_mm"],
                    "action_count": artifact.summary["action_count"],
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
