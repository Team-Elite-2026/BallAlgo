from __future__ import annotations

import argparse
import math
from pathlib import Path

from artifact_io import read_artifact


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ARTIFACT_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "generated"
GRACE_HOLD_US = 20_000


def _field_to_body(vx_field: float, vy_field: float, heading_deg: float) -> tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    return c * vx_field + s * vy_field, -s * vx_field + c * vy_field


def sample_chunk(chunk: dict, start_time_pi_us: int, query_pi_us: int,
                 heading_deg: float) -> dict:
    actions = chunk["actions"]
    dt_ms = int(chunk["dt_ms"])
    scheduled = query_pi_us < start_time_pi_us
    stop_chunk = not actions or dt_ms == 0
    sample = {
        "valid": False,
        "active": False,
        "grace_hold": False,
        "scheduled": scheduled,
        "stop_chunk": stop_chunk,
        "trajectory_id": int(chunk["trajectory_id"]),
        "action_index": 0,
    }

    if scheduled:
        return sample
    if stop_chunk:
        sample["valid"] = True
        return sample

    dt_us = dt_ms * 1000
    elapsed_us = query_pi_us - start_time_pi_us
    action_index = elapsed_us // dt_us
    if action_index >= len(actions):
        grace_slots = max(1, GRACE_HOLD_US // dt_us)
        if action_index >= len(actions) + grace_slots:
            return sample
        action_index = len(actions) - 1
        sample["grace_hold"] = True
    else:
        sample["active"] = True

    action = dict(actions[action_index])
    if sample["grace_hold"]:
        action["ax"] = 0.0
        action["ay"] = 0.0
        action["alpha"] = 0.0
    vx_body, vy_body = _field_to_body(action["vx"], action["vy"], heading_deg)
    sample.update(
        {
            "valid": True,
            "action_index": int(action_index),
            "action": action,
            "vx_body": vx_body,
            "vy_body": vy_body,
        }
    )
    return sample


def verify_forward_only(artifact: dict) -> None:
    goal = artifact["goal_pose"]
    assert goal["enabled"]
    assert abs(goal["x_mm"] - artifact["start_pose"]["x_mm"]) < 1.0
    assert goal["y_mm"] > artifact["start_pose"]["y_mm"] + 80.0


def verify_strafe_only(artifact: dict) -> None:
    goal = artifact["goal_pose"]
    assert goal["x_mm"] > artifact["start_pose"]["x_mm"] + 60.0
    assert abs(goal["y_mm"] - artifact["start_pose"]["y_mm"]) < 1.0


def verify_rotate_only(artifact: dict) -> None:
    goal = artifact["goal_pose"]
    assert abs(goal["x_mm"] - artifact["start_pose"]["x_mm"]) < 1.0
    assert abs(goal["y_mm"] - artifact["start_pose"]["y_mm"]) < 1.0
    assert abs(goal["heading_deg"]) > 20.0


def verify_diagonal(artifact: dict) -> None:
    goal = artifact["goal_pose"]
    assert goal["x_mm"] > artifact["start_pose"]["x_mm"] + 30.0
    assert goal["y_mm"] > artifact["start_pose"]["y_mm"] + 30.0


def verify_translate_while_rotating(artifact: dict) -> None:
    goal = artifact["goal_pose"]
    assert goal["x_mm"] > artifact["start_pose"]["x_mm"] + 20.0
    assert goal["y_mm"] > artifact["start_pose"]["y_mm"] + 40.0
    assert abs(goal["heading_deg"]) > 10.0


def verify_accelerate_then_stop(artifact: dict) -> None:
    assert [len(chunk["actions"]) for chunk in artifact["chunks"]] == [50, 50, 0]
    stop_chunk = artifact["chunks"][2]
    sample = sample_chunk(stop_chunk, 1_000_000 + stop_chunk["start_delay_ms"] * 1000,
                          1_000_000 + stop_chunk["start_delay_ms"] * 1000 + 4_000, 0.0)
    assert sample["valid"] and sample["stop_chunk"]


def verify_chunk_boundary_handoff(artifact: dict) -> None:
    assert [len(chunk["actions"]) for chunk in artifact["chunks"]] == [50, 50, 0]
    first_chunk = artifact["chunks"][0]
    second_chunk = artifact["chunks"][1]
    first_sample = sample_chunk(first_chunk, 1_000_000, 1_000_000 + 196_000, 0.0)
    second_sample = sample_chunk(second_chunk, 1_200_000, 1_200_000 + 4_000, 0.0)
    assert first_sample["valid"] and first_sample["trajectory_id"] == 8
    assert second_sample["valid"] and second_sample["trajectory_id"] == 9
    assert second_sample["action_index"] == 1


def verify_num_actions_stop_chunk(artifact: dict) -> None:
    assert [len(chunk["actions"]) for chunk in artifact["chunks"]] == [50, 0]
    stop_chunk = artifact["chunks"][1]
    sample = sample_chunk(stop_chunk, 1_200_000, 1_204_000, 0.0)
    assert sample["valid"] and sample["stop_chunk"]


def verify_trapezoid_velocity(artifact: dict) -> None:
    assert [len(chunk["actions"]) for chunk in artifact["chunks"]] == [50] * 8 + [0]
    goal = artifact["goal_pose"]
    assert goal["enabled"]
    assert abs(goal["x_mm"] - artifact["start_pose"]["x_mm"]) < 1.0
    assert goal["y_mm"] > artifact["start_pose"]["y_mm"] + 400.0
    stop_chunk = artifact["chunks"][-1]
    sample = sample_chunk(stop_chunk, 1_000_000 + stop_chunk["start_delay_ms"] * 1000,
                          1_000_000 + stop_chunk["start_delay_ms"] * 1000 + 4_000, 0.0)
    assert sample["valid"] and sample["stop_chunk"]


def verify_rectangle_loop(artifact: dict) -> None:
    # 40 driving chunks (50 actions each) + a final stop chunk, forming a closed box.
    assert [len(chunk["actions"]) for chunk in artifact["chunks"]] == [50] * 40 + [0]
    start = artifact["start_pose"]
    goal = artifact["goal_pose"]
    assert goal["enabled"]
    assert abs(goal["x_mm"] - start["x_mm"]) < 1.0, "rectangle must return to start x"
    assert abs(goal["y_mm"] - start["y_mm"]) < 1.0, "rectangle must return to start y"
    xs = [p["x_mm"] for p in artifact["path"]]
    ys = [p["y_mm"] for p in artifact["path"]]
    assert (max(xs) - min(xs)) > 400.0 and (max(ys) - min(ys)) > 400.0, "expected a box-sized extent"
    stop_chunk = artifact["chunks"][-1]
    sample = sample_chunk(stop_chunk, 1_000_000 + stop_chunk["start_delay_ms"] * 1000,
                          1_000_000 + stop_chunk["start_delay_ms"] * 1000 + 4_000, 0.0)
    assert sample["valid"] and sample["stop_chunk"]


def verify_planner_demo(artifact: dict) -> None:
    assert artifact["case_kind"] == "planner_goal"
    assert len(artifact["path"]) > 1
    assert len(artifact["chunks"]) == 1
    assert len(artifact["chunks"][0]["actions"]) > 0


VERIFY_BY_CASE = {
    "forward_only": verify_forward_only,
    "strafe_only": verify_strafe_only,
    "rotate_only": verify_rotate_only,
    "diagonal": verify_diagonal,
    "translate_while_rotating": verify_translate_while_rotating,
    "accelerate_then_stop": verify_accelerate_then_stop,
    "chunk_boundary_handoff": verify_chunk_boundary_handoff,
    "num_actions_0_stop_chunk": verify_num_actions_stop_chunk,
    "trapezoid_velocity": verify_trapezoid_velocity,
    "rectangle_loop": verify_rectangle_loop,
    "planner_to_pose_demo": verify_planner_demo,
}


def verify_artifact(path: Path) -> None:
    artifact = read_artifact(path)
    assert artifact["path"], f"{path.name}: expected non-empty path preview"
    for chunk in artifact["chunks"]:
        assert len(chunk["actions"]) <= 50, f"{path.name}: chunk exceeds executor limit"
    verifier = VERIFY_BY_CASE.get(artifact["case_name"])
    if verifier is None:
        raise AssertionError(f"{path.name}: no verifier registered")
    verifier(artifact)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify generated trajectory replay artifacts")
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=DEFAULT_ARTIFACT_DIR,
        help="Directory of generated .traj artifacts",
    )
    parser.add_argument(
        "--artifact",
        type=Path,
        help="Verify a single artifact instead of scanning the directory",
    )
    args = parser.parse_args()

    artifact_paths = [args.artifact] if args.artifact else sorted(args.artifact_dir.glob("*.traj"))
    if not artifact_paths:
        raise SystemExit("no .traj artifacts found to verify")

    for artifact_path in artifact_paths:
        verify_artifact(artifact_path)
        print(f"verified {artifact_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
