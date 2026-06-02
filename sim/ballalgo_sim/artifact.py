from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class SimulationArtifact:
    path: Path
    raw: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "SimulationArtifact":
        artifact_path = Path(path)
        with artifact_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
        return cls(path=artifact_path, raw=raw)

    @property
    def field(self) -> dict[str, Any]:
        return self.raw["field"]

    @property
    def input(self) -> dict[str, Any]:
        return self.raw["input"]

    @property
    def summary(self) -> dict[str, Any]:
        return self.raw["summary"]

    @property
    def chunk(self) -> dict[str, Any]:
        return self.raw["chunk"]

    @property
    def waypoints(self) -> list[dict[str, Any]]:
        return self.raw.get("waypoints", [])

    @property
    def path_samples(self) -> list[dict[str, Any]]:
        if "path" in self.raw:
            return self.raw["path"]
        replans = self.replans
        return replans[0]["path"] if replans else []

    @property
    def profile_samples(self) -> list[dict[str, Any]]:
        if "profile" in self.raw:
            return self.raw["profile"]
        replans = self.replans
        return replans[0]["profile"] if replans else []

    @property
    def actions(self) -> list[dict[str, Any]]:
        if "executed_actions" in self.raw:
            return self.raw["executed_actions"]
        return self.raw.get("actions", [])

    @property
    def sim_trace(self) -> list[dict[str, Any]]:
        return self.raw.get("executed_trace", self.raw.get("sim_trace", []))

    @property
    def replans(self) -> list[dict[str, Any]]:
        return self.raw.get("replans", [])

    @property
    def mode(self) -> str:
        return str(self.raw.get("mode", "pose_target"))

    def start_pose_cm(self) -> tuple[float, float]:
        start = self.input["start"]
        return float(start["x_cm"]), float(start["y_cm"])

    def goal_pose_cm(self) -> tuple[float, float]:
        goal = self.input["goal"]
        return float(goal["x_cm"]), float(goal["y_cm"])

    def goal_target_cm(self) -> tuple[float, float]:
        goal_target = self.input.get("goal_target", {})
        return float(goal_target.get("x_cm", 0.0)), float(goal_target.get("y_cm", 0.0))

    def ball_field_cm(self) -> tuple[float, float]:
        ball = self.input["ball"]
        return float(ball["x_cm"]), float(ball["y_cm"])

    def ball_velocity_cm_s(self) -> tuple[float, float]:
        ball = self.input["ball"]
        return float(ball["vx_cm_s"]), float(ball["vy_cm_s"])

    def field_bounds_cm(self) -> tuple[float, float, float, float]:
        half_width = float(self.field["width_mm"]) * 0.05
        half_height = float(self.field["height_mm"]) * 0.05
        return -half_width, half_width, -half_height, half_height


def load_artifact(path: str | Path) -> SimulationArtifact:
    return SimulationArtifact.load(path)
