from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Any

from artifact_io import write_artifact


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SPEC_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "specs"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "generated"
DEFAULT_BUILD_DIR = REPO_ROOT / "build"
PLANNER_BINARY = "ballalgo_planner_case"
MAX_ACTIONS = 50


def field_to_body(vx_mm_s: float, vy_mm_s: float, heading_deg: float) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    return ((c * vx_mm_s + s * vy_mm_s) / 1000.0, (-s * vx_mm_s + c * vy_mm_s) / 1000.0)


def _wrap_heading_deg(angle_deg: float) -> float:
    wrapped = math.fmod(angle_deg + 180.0, 360.0)
    if wrapped < 0.0:
        wrapped += 360.0
    return wrapped - 180.0


def populate_manual_preview(artifact: dict[str, Any]) -> None:
    chunks = artifact["chunks"]
    all_actions = [action for chunk in chunks for action in chunk["actions"]]
    total_actions = len(all_actions)
    x_mm = float(artifact["start_pose"]["x_mm"])
    y_mm = float(artifact["start_pose"]["y_mm"])
    heading_deg = float(artifact["start_heading_deg"])
    s_mm = 0.0

    path: list[dict[str, float]] = [
        {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "heading_deg": heading_deg,
            "s_mm": s_mm,
        }
    ]
    profile: list[dict[str, float]] = []

    action_cursor = 0
    for chunk in chunks:
        dt_s = max(float(chunk["dt_ms"]) / 1000.0, 1e-6)
        for action in chunk["actions"]:
            speed_m_s = math.hypot(action["vx"], action["vy"])
            progress = 1.0 if total_actions <= 1 else action_cursor / float(total_actions - 1)
            profile.append({"progress_01": progress, "speed_m_s": speed_m_s})

            x_mm += action["vx"] * dt_s * 1000.0
            y_mm += action["vy"] * dt_s * 1000.0
            heading_deg = _wrap_heading_deg(heading_deg + math.degrees(action["omega"] * dt_s))
            s_mm += speed_m_s * dt_s * 1000.0
            path.append(
                {
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "heading_deg": heading_deg,
                    "s_mm": s_mm,
                }
            )
            action_cursor += 1

    artifact["path"] = path
    artifact["trajectory_speed_profile"] = profile
    artifact["goal_pose"] = {
        "enabled": True,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "heading_deg": heading_deg,
    }


def build_manual_artifact(spec: dict[str, Any]) -> dict[str, Any]:
    start_pose = dict(spec["start_pose"])
    vx_mm_s = float(start_pose.get("vx_mm_s", 0.0))
    vy_mm_s = float(start_pose.get("vy_mm_s", 0.0))
    start_heading_deg = float(start_pose.get("heading_deg", 0.0))
    vx_body, vy_body = field_to_body(vx_mm_s, vy_mm_s, start_heading_deg)

    artifact: dict[str, Any] = {
        "case_name": spec["case_name"],
        "case_kind": spec["kind"],
        "start_pose": {
            "valid": bool(start_pose.get("valid", True)),
            "x_mm": float(start_pose["x_mm"]),
            "y_mm": float(start_pose["y_mm"]),
            "vx_mm_s": vx_mm_s,
            "vy_mm_s": vy_mm_s,
            "vx_body_m_s": vx_body,
            "vy_body_m_s": vy_body,
        },
        "start_heading_deg": start_heading_deg,
        "goal_pose": {"enabled": False},
        "obstacle": {"enabled": False},
        "path": [],
        "trajectory_speed_profile": [],
        "chunks": [],
    }

    for chunk_index, chunk_spec in enumerate(spec["chunks"], start=1):
        dt_ms = int(chunk_spec.get("dt_ms", 4))
        if dt_ms <= 0:
            raise ValueError(f"{spec['case_name']}: dt_ms must be positive")
        stop_chunk = bool(chunk_spec.get("stop_chunk", False))
        actions: list[dict[str, float]] = []
        if not stop_chunk:
            for segment in chunk_spec.get("segments", []):
                duration_ms = int(segment["duration_ms"])
                steps = max(1, round(duration_ms / dt_ms))
                duration_s = max(steps * dt_ms / 1000.0, 1e-6)

                vx0 = float(segment.get("vx_start", segment.get("vx", 0.0)))
                vx1 = float(segment.get("vx_end", segment.get("vx", vx0)))
                vy0 = float(segment.get("vy_start", segment.get("vy", 0.0)))
                vy1 = float(segment.get("vy_end", segment.get("vy", vy0)))
                omega0 = float(segment.get("omega_start", segment.get("omega", 0.0)))
                omega1 = float(segment.get("omega_end", segment.get("omega", omega0)))
                ax = float(segment.get("ax", (vx1 - vx0) / duration_s))
                ay = float(segment.get("ay", (vy1 - vy0) / duration_s))
                alpha = float(segment.get("alpha", (omega1 - omega0) / duration_s))

                for step in range(steps):
                    t01 = 0.0 if steps == 1 else step / float(steps - 1)
                    actions.append(
                        {
                            "vx": vx0 + (vx1 - vx0) * t01,
                            "vy": vy0 + (vy1 - vy0) * t01,
                            "omega": omega0 + (omega1 - omega0) * t01,
                            "ax": ax,
                            "ay": ay,
                            "alpha": alpha,
                        }
                    )
        if len(actions) > MAX_ACTIONS:
            raise ValueError(
                f"{spec['case_name']}: chunk {chunk_index} expands to {len(actions)} actions "
                f"but the executor supports at most {MAX_ACTIONS}"
            )

        artifact["chunks"].append(
            {
                "trajectory_id": int(chunk_spec.get("trajectory_id", chunk_index)),
                "start_delay_ms": int(chunk_spec.get("start_delay_ms", 0)),
                "dt_ms": dt_ms,
                "kick": int(chunk_spec.get("kick", 0)),
                "dribbler_power": int(chunk_spec.get("dribbler_power", 0)),
                "actions": actions,
            }
        )

    populate_manual_preview(artifact)
    return artifact


def run_planner_generator(spec: dict[str, Any], build_dir: Path, output_path: Path) -> None:
    binary = build_dir / PLANNER_BINARY
    if not binary.exists():
        raise FileNotFoundError(f"planner binary not found: {binary}")

    start_pose = spec["start_pose"]
    goal_pose = spec["goal_pose"]
    cmd = [
        str(binary),
        "--case-name",
        spec["case_name"],
        "--output",
        str(output_path),
        "--start",
        str(start_pose["x_mm"]),
        str(start_pose["y_mm"]),
        str(start_pose["heading_deg"]),
        "--goal",
        str(goal_pose["x_mm"]),
        str(goal_pose["y_mm"]),
        str(goal_pose["heading_deg"]),
    ]
    vx_mm_s = float(start_pose.get("vx_mm_s", 0.0))
    vy_mm_s = float(start_pose.get("vy_mm_s", 0.0))
    if abs(vx_mm_s) > 1e-6 or abs(vy_mm_s) > 1e-6:
        cmd.extend(["--start-velocity-field", str(vx_mm_s), str(vy_mm_s)])
    subprocess.run(cmd, check=True)


def generate_from_spec(spec_path: Path, build_dir: Path, output_dir: Path) -> Path:
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    case_name = spec["case_name"]
    output_path = output_dir / f"{case_name}.traj"
    output_dir.mkdir(parents=True, exist_ok=True)

    if spec["kind"] == "planner_goal":
        run_planner_generator(spec, build_dir, output_path)
        return output_path

    if spec["kind"] == "manual_sequence":
        artifact = build_manual_artifact(spec)
        write_artifact(output_path, artifact)
        print(f"wrote {output_path}")
        return output_path

    raise ValueError(f"unsupported case kind: {spec['kind']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate trajectory replay artifacts")
    parser.add_argument("--spec", type=Path, help="Generate a single case spec JSON file")
    parser.add_argument(
        "--spec-dir",
        type=Path,
        default=DEFAULT_SPEC_DIR,
        help="Directory of case spec JSON files",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Where to write generated .traj artifacts",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help="BallAlgo build directory containing ballalgo_planner_case",
    )
    args = parser.parse_args()

    spec_paths = [args.spec] if args.spec else sorted(args.spec_dir.glob("*.json"))
    if not spec_paths:
        raise SystemExit("no spec files found")

    for spec_path in spec_paths:
        generate_from_spec(spec_path, args.build_dir, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
