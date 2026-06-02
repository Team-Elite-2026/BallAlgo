from __future__ import annotations

from pathlib import Path
import sys

SIM_DIR = Path(__file__).resolve().parents[2] / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from ballalgo_sim.artifact import SimulationArtifact, load_artifact

PlannerArtifact = SimulationArtifact

__all__ = ["PlannerArtifact", "SimulationArtifact", "load_artifact"]
