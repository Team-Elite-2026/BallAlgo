#include "motion/HermiteSpline.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

namespace {
// Cubic Hermite basis and derivatives.
inline double h00(double t) { return 2 * t * t * t - 3 * t * t + 1; }
inline double h10(double t) { return t * t * t - 2 * t * t + t; }
inline double h01(double t) { return -2 * t * t * t + 3 * t * t; }
inline double h11(double t) { return t * t * t - t * t; }
inline double dh00(double t) { return 6 * t * t - 6 * t; }
inline double dh10(double t) { return 3 * t * t - 4 * t + 1; }
inline double dh01(double t) { return -6 * t * t + 6 * t; }
inline double dh11(double t) { return 3 * t * t - 2 * t; }
inline double ddh00(double t) { return 12 * t - 6; }
inline double ddh10(double t) { return 6 * t - 4; }
inline double ddh01(double t) { return -12 * t + 6; }
inline double ddh11(double t) { return 6 * t - 2; }
}  // namespace

SplineDerivState HermiteSplineData::evalState(double s) const {
  SplineDerivState st;
  if (nodes.empty() || numSegments == 0) return st;
  s = std::clamp(s, 0.0, 1.0);

  const int N = numSegments;
  const double sN = s * N;
  const int seg = std::min(static_cast<int>(sN), N - 1);
  const double t = sN - seg;
  const HermiteNode& n0 = nodes[seg];
  const HermiteNode& n1 = nodes[seg + 1];

  const double p00 = h00(t), p10 = h10(t), p01 = h01(t), p11 = h11(t);
  const double d00 = dh00(t), d10 = dh10(t), d01 = dh01(t), d11 = dh11(t);
  const double e00 = ddh00(t), e10 = ddh10(t), e01 = ddh01(t), e11 = ddh11(t);

  const double xMm = p00 * n0.wxMm + p10 * n0.ptxMm + p01 * n1.wxMm + p11 * n1.ptxMm;
  const double yMm = p00 * n0.wyMm + p10 * n0.ptyMm + p01 * n1.wyMm + p11 * n1.ptyMm;
  const double theta =
      p00 * n0.thetaRad + p10 * n0.ptTheta + p01 * n1.thetaRad + p11 * n1.ptTheta;

  const double dxdt = d00 * n0.wxMm + d10 * n0.ptxMm + d01 * n1.wxMm + d11 * n1.ptxMm;
  const double dydt = d00 * n0.wyMm + d10 * n0.ptyMm + d01 * n1.wyMm + d11 * n1.ptyMm;
  const double dthdt = d00 * n0.thetaRad + d10 * n0.ptTheta + d01 * n1.thetaRad + d11 * n1.ptTheta;

  const double d2xdt2 = e00 * n0.wxMm + e10 * n0.ptxMm + e01 * n1.wxMm + e11 * n1.ptxMm;
  const double d2ydt2 = e00 * n0.wyMm + e10 * n0.ptyMm + e01 * n1.wyMm + e11 * n1.ptyMm;
  const double d2thdt2 =
      e00 * n0.thetaRad + e10 * n0.ptTheta + e01 * n1.thetaRad + e11 * n1.ptTheta;

  // Chain rule s = t/N, plus mm -> m for positional terms.
  const double MM_TO_M = 0.001;
  st.x = xMm * MM_TO_M;
  st.y = yMm * MM_TO_M;
  st.theta = theta;
  st.dx_ds = dxdt * N * MM_TO_M;
  st.dy_ds = dydt * N * MM_TO_M;
  st.dtheta_ds = dthdt * N;
  st.d2x_ds2 = d2xdt2 * N * N * MM_TO_M;
  st.d2y_ds2 = d2ydt2 * N * N * MM_TO_M;
  st.d2theta_ds2 = d2thdt2 * N * N;

  const double denomSq = st.dx_ds * st.dx_ds + st.dy_ds * st.dy_ds;
  if (denomSq > 1e-18) {
    const double denom = std::pow(denomSq, 1.5);
    st.kappa = std::fabs(st.dx_ds * st.d2y_ds2 - st.dy_ds * st.d2x_ds2) / denom;
  }
  return st;
}

HermiteSplineData HermiteSpline::buildData(const std::vector<Waypoint3>& waypoints,
                                           float goalThetaDeg, double vxStartMmS,
                                           double vyStartMmS, double vxEndMmS, double vyEndMmS,
                                           int samplesPerSegment) {
  HermiteSplineData data;
  const size_t n = waypoints.size();
  if (n < 2) {
    if (n == 1) {
      data.nodes.push_back({waypoints[0].xMm, waypoints[0].yMm,
                            goalThetaDeg * M_PI / 180.0, 0, 0, 0});
      data.samples.push_back({waypoints[0].xMm, waypoints[0].yMm, goalThetaDeg, 0});
    }
    return data;
  }

  std::vector<double> wx(n), wy(n);
  for (size_t i = 0; i < n; ++i) {
    wx[i] = waypoints[i].xMm;
    wy[i] = waypoints[i].yMm;
  }

  // Per-node tangent scale = average adjacent chord length (C1 continuity).
  std::vector<double> scale(n);
  for (size_t i = 0; i < n; ++i) {
    const double prev = (i > 0) ? std::hypot(wx[i] - wx[i - 1], wy[i] - wy[i - 1]) : 0.0;
    const double next = (i + 1 < n) ? std::hypot(wx[i + 1] - wx[i], wy[i + 1] - wy[i]) : 0.0;
    if (i == 0)
      scale[i] = next;
    else if (i == n - 1)
      scale[i] = prev;
    else
      scale[i] = (prev + next) * 0.5;
  }

  // Position tangents. Boundaries follow the supplied velocity vectors so that
  // S'(0)=v_start and S'(1)=v_end; when velocity is ~0 fall back to the chord
  // direction at chord scale so the spline shape stays well behaved.
  std::vector<double> ptx(n), pty(n);
  // Boundary tangent: take the velocity direction (so the path leaves/arrives
  // without an instantaneous direction change, satisfying the S'(0)=v_start
  // constraint in direction) but scale the magnitude to the chord length to
  // avoid Hermite overshoot when |v| differs greatly from the node spacing.
  auto setBoundaryTangent = [](double vx, double vy, double chordScale, double dirx, double diry,
                               double& ox, double& oy) {
    const double vmag = std::hypot(vx, vy);
    if (vmag > 1e-6) {
      ox = vx / vmag * chordScale;
      oy = vy / vmag * chordScale;
    } else {
      const double dmag = std::hypot(dirx, diry);
      if (dmag > 1e-9) {
        ox = dirx / dmag * chordScale;
        oy = diry / dmag * chordScale;
      } else {
        ox = oy = 0;
      }
    }
  };
  setBoundaryTangent(vxStartMmS, vyStartMmS, scale[0], wx[1] - wx[0], wy[1] - wy[0], ptx[0], pty[0]);
  setBoundaryTangent(vxEndMmS, vyEndMmS, scale[n - 1], wx[n - 1] - wx[n - 2],
                     wy[n - 1] - wy[n - 2], ptx[n - 1], pty[n - 1]);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double dx = wx[i + 1] - wx[i - 1];
    const double dy = wy[i + 1] - wy[i - 1];
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) {
      ptx[i] = pty[i] = 0;
      continue;
    }
    ptx[i] = (dx / len) * scale[i];
    pty[i] = (dy / len) * scale[i];
  }

  // Orientation: per-node heading (deg) -> radians, unwrapped; last node forced
  // to the goal heading.
  std::vector<double> thetaRad(n);
  thetaRad[0] = waypoints[0].thetaDeg * M_PI / 180.0;
  for (size_t i = 1; i < n; ++i) {
    const double target =
        ((i == n - 1) ? goalThetaDeg : waypoints[i].thetaDeg) * M_PI / 180.0;
    double delta = target - thetaRad[i - 1];
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    thetaRad[i] = thetaRad[i - 1] + delta;
  }

  // Theta tangents: Catmull-Rom interior, one-sided at boundaries.
  std::vector<double> ptTheta(n);
  ptTheta[0] = thetaRad[1] - thetaRad[0];
  ptTheta[n - 1] = thetaRad[n - 1] - thetaRad[n - 2];
  for (size_t i = 1; i + 1 < n; ++i) ptTheta[i] = (thetaRad[i + 1] - thetaRad[i - 1]) * 0.5;

  data.nodes.resize(n);
  for (size_t i = 0; i < n; ++i)
    data.nodes[i] = {wx[i], wy[i], thetaRad[i], ptx[i], pty[i], ptTheta[i]};
  data.numSegments = static_cast<int>(n) - 1;

  // Discrete samples with cumulative arc length (sMm) for telemetry / fallback.
  const int S = std::max(1, samplesPerSegment);
  const int totalSamples = (static_cast<int>(n) - 1) * S;
  data.samples.reserve(totalSamples + 1);
  double sAcc = 0;
  double prevX = wx[0];
  double prevY = wy[0];
  data.samples.push_back({static_cast<float>(wx[0]), static_cast<float>(wy[0]),
                          static_cast<float>(waypoints[0].thetaDeg), 0});
  for (int i = 0; i < totalSamples; ++i) {
    const double sGlobal = static_cast<double>(i + 1) / totalSamples;
    const SplineDerivState stt = data.evalState(sGlobal);
    const double xMm = stt.x * 1000.0;
    const double yMm = stt.y * 1000.0;
    sAcc += std::hypot(xMm - prevX, yMm - prevY);
    prevX = xMm;
    prevY = yMm;
    float thDeg = static_cast<float>(stt.theta * 180.0 / M_PI);
    data.samples.push_back({static_cast<float>(xMm), static_cast<float>(yMm), thDeg,
                            static_cast<float>(sAcc)});
  }
  return data;
}

}  // namespace ballalgo
