from __future__ import annotations

from pathlib import Path
from typing import Any


def _parse_bool(token: str) -> bool:
    lowered = token.strip().lower()
    if lowered in {"1", "true"}:
        return True
    if lowered in {"0", "false"}:
        return False
    raise ValueError(f"invalid boolean token: {token!r}")


def read_artifact(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    lines = path.read_text(encoding="utf-8").splitlines()
    if not lines or lines[0].strip() != "BALLALGO_TRAJECTORY_REPLAY 1":
        raise ValueError(f"{path} is not a BALLALGO_TRAJECTORY_REPLAY v1 artifact")

    artifact: dict[str, Any] = {
        "case_name": "",
        "case_kind": "",
        "start_pose": {},
        "start_heading_deg": 0.0,
        "goal_pose": {"enabled": False},
        "obstacle": {"enabled": False},
        "path": [],
        "trajectory_speed_profile": [],
        "chunks": [],
    }
    current_chunk: dict[str, Any] | None = None

    for raw in lines[1:]:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        tag = parts[0]
        if tag == "case_name":
            artifact["case_name"] = parts[1]
        elif tag == "case_kind":
            artifact["case_kind"] = parts[1]
        elif tag == "start_pose":
            artifact["start_pose"] = {
                "valid": _parse_bool(parts[1]),
                "x_mm": float(parts[2]),
                "y_mm": float(parts[3]),
                "vx_mm_s": float(parts[4]),
                "vy_mm_s": float(parts[5]),
                "vx_body_m_s": float(parts[6]),
                "vy_body_m_s": float(parts[7]),
            }
        elif tag == "start_heading_deg":
            artifact["start_heading_deg"] = float(parts[1])
        elif tag == "goal_pose":
            artifact["goal_pose"] = {
                "enabled": _parse_bool(parts[1]),
                "x_mm": float(parts[2]),
                "y_mm": float(parts[3]),
                "heading_deg": float(parts[4]),
            }
        elif tag == "obstacle":
            artifact["obstacle"] = {
                "enabled": _parse_bool(parts[1]),
                "x_mm": float(parts[2]),
                "y_mm": float(parts[3]),
                "clear_mm": float(parts[4]),
            }
        elif tag == "path_count":
            artifact["path"] = []
        elif tag == "path":
            artifact["path"].append({
                "x_mm": float(parts[1]),
                "y_mm": float(parts[2]),
                "heading_deg": float(parts[3]),
                "s_mm": float(parts[4]),
            })
        elif tag == "profile_count":
            artifact["trajectory_speed_profile"] = []
        elif tag == "profile":
            artifact["trajectory_speed_profile"].append({
                "progress_01": float(parts[1]),
                "speed_m_s": float(parts[2]),
            })
        elif tag == "chunk_count":
            artifact["chunks"] = []
        elif tag == "chunk":
            current_chunk = {
                "trajectory_id": int(parts[1]),
                "start_delay_ms": int(parts[2]),
                "dt_ms": int(parts[3]),
                "kick": int(parts[4]),
                "dribbler_power": int(parts[5]),
                "expected_actions": int(parts[6]),
                "actions": [],
            }
            artifact["chunks"].append(current_chunk)
        elif tag == "action":
            if current_chunk is None:
                raise ValueError("action line outside of chunk block")
            current_chunk["actions"].append({
                "vx": float(parts[1]),
                "vy": float(parts[2]),
                "omega": float(parts[3]),
                "ax": float(parts[4]),
                "ay": float(parts[5]),
                "alpha": float(parts[6]),
            })
        elif tag == "endchunk":
            current_chunk = None
        else:
            raise ValueError(f"unknown artifact tag: {tag}")

    for chunk in artifact["chunks"]:
        if len(chunk["actions"]) != chunk["expected_actions"]:
            raise ValueError(
                f"chunk {chunk['trajectory_id']} expected {chunk['expected_actions']} actions "
                f"but found {len(chunk['actions'])}"
            )
    return artifact


def write_artifact(path: str | Path, artifact: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines: list[str] = ["BALLALGO_TRAJECTORY_REPLAY 1"]
    lines.append(f"case_name {artifact['case_name']}")
    lines.append(f"case_kind {artifact['case_kind']}")

    start_pose = artifact["start_pose"]
    lines.append(
        "start_pose "
        f"{1 if start_pose.get('valid', True) else 0} "
        f"{start_pose.get('x_mm', 0.0):.6f} "
        f"{start_pose.get('y_mm', 0.0):.6f} "
        f"{start_pose.get('vx_mm_s', 0.0):.6f} "
        f"{start_pose.get('vy_mm_s', 0.0):.6f} "
        f"{start_pose.get('vx_body_m_s', 0.0):.6f} "
        f"{start_pose.get('vy_body_m_s', 0.0):.6f}"
    )
    lines.append(f"start_heading_deg {artifact.get('start_heading_deg', 0.0):.6f}")

    goal_pose = artifact.get("goal_pose", {"enabled": False})
    lines.append(
        "goal_pose "
        f"{1 if goal_pose.get('enabled', False) else 0} "
        f"{goal_pose.get('x_mm', 0.0):.6f} "
        f"{goal_pose.get('y_mm', 0.0):.6f} "
        f"{goal_pose.get('heading_deg', 0.0):.6f}"
    )

    obstacle = artifact.get("obstacle", {"enabled": False})
    lines.append(
        "obstacle "
        f"{1 if obstacle.get('enabled', False) else 0} "
        f"{obstacle.get('x_mm', 0.0):.6f} "
        f"{obstacle.get('y_mm', 0.0):.6f} "
        f"{obstacle.get('clear_mm', 0.0):.6f}"
    )

    path_points = artifact.get("path", [])
    lines.append(f"path_count {len(path_points)}")
    for point in path_points:
        lines.append(
            "path "
            f"{point.get('x_mm', 0.0):.6f} "
            f"{point.get('y_mm', 0.0):.6f} "
            f"{point.get('heading_deg', 0.0):.6f} "
            f"{point.get('s_mm', 0.0):.6f}"
        )

    profile = artifact.get("trajectory_speed_profile", [])
    lines.append(f"profile_count {len(profile)}")
    for sample in profile:
        lines.append(
            "profile "
            f"{sample.get('progress_01', 0.0):.6f} "
            f"{sample.get('speed_m_s', 0.0):.6f}"
        )

    chunks = artifact.get("chunks", [])
    lines.append(f"chunk_count {len(chunks)}")
    for chunk in chunks:
        actions = chunk.get("actions", [])
        lines.append(
            "chunk "
            f"{chunk.get('trajectory_id', 0)} "
            f"{chunk.get('start_delay_ms', 0)} "
            f"{chunk.get('dt_ms', 0)} "
            f"{chunk.get('kick', 0)} "
            f"{chunk.get('dribbler_power', 0)} "
            f"{len(actions)}"
        )
        for action in actions:
            lines.append(
                "action "
                f"{action.get('vx', 0.0):.6f} "
                f"{action.get('vy', 0.0):.6f} "
                f"{action.get('omega', 0.0):.6f} "
                f"{action.get('ax', 0.0):.6f} "
                f"{action.get('ay', 0.0):.6f} "
                f"{action.get('alpha', 0.0):.6f}"
            )
        lines.append("endchunk")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
