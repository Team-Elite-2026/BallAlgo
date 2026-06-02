"""Compatibility re-exports for the BallAlgo sim package."""

from .artifact import SimulationArtifact, load_artifact

PlannerArtifact = SimulationArtifact

__all__ = ["PlannerArtifact", "SimulationArtifact", "load_artifact"]
