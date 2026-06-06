#pragma once

#include "motion/AStar3D.hpp"

#include <vector>

namespace ballalgo {

struct PathSample {
  float xMm, yMm, thetaDeg, sMm;
};

// Continuous spline state at a single s, all SI (metres / radians / 1/m).
// Mirrors docs/pipeline_context.md Step 4.1 geometry extraction.
struct SplineDerivState {
  double x = 0, y = 0;        // position (m)
  double theta = 0;           // orientation (rad, unwrapped)
  double dx_ds = 0, dy_ds = 0;
  double dtheta_ds = 0;
  double d2x_ds2 = 0, d2y_ds2 = 0;
  double d2theta_ds2 = 0;
  double kappa = 0;           // curvature (1/m)
};

// One Hermite control node. Positions stored in mm; converted to m in evalState.
struct HermiteNode {
  double wxMm = 0, wyMm = 0;
  double thetaRad = 0;        // unwrapped orientation
  double ptxMm = 0, ptyMm = 0;  // position tangent w.r.t. local segment t (mm)
  double ptTheta = 0;           // theta tangent w.r.t. local segment t (rad)
};

// Full spline: discrete samples (for telemetry / fallback) plus per-node data
// for continuous analytical evaluation used by the velocity profiler.
struct HermiteSplineData {
  std::vector<PathSample> samples;
  std::vector<HermiteNode> nodes;
  int numSegments = 0;
  // Evaluate position/derivatives/curvature at global s in [0,1] (clamped).
  SplineDerivState evalState(double s) const;
};

class HermiteSpline {
 public:
  // Retains per-node data so the spline can be evaluated analytically
  // (derivatives + curvature) at any s. Boundary position tangents are set from
  // the supplied velocity vectors (mm/s) so S'(0)=v_start and S'(1)=v_end.
  HermiteSplineData buildData(const std::vector<Waypoint3>& waypoints, float goalThetaDeg,
                              double vxStartMmS, double vyStartMmS, double vxEndMmS,
                              double vyEndMmS, int samplesPerSegment = 10);
};

}  // namespace ballalgo
