#include "motion/VelocityProfile.hpp"

#include "config.hpp"
#include "params.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ballalgo {

// ── Motor-model S-curve profiler (Step 4.3) ────────────────────────────────

namespace {

struct MotorModel {
  float maxWheelVel;  // m/s
  std::array<float, 4> alpha;
};

MotorModel makeMotorModel() {
  MotorModel m;
  const auto& p = params::get();
  const float omegaNoLoad =
      (p.motorNoLoadRpm * 2.0f * static_cast<float>(M_PI)) / 60.0f;
  m.maxWheelVel = (omegaNoLoad - p.motorLoadCompOmega) * p.wheelRadiusM;
  m.alpha = {p.wheelAngle0, p.wheelAngle1, p.wheelAngle2, p.wheelAngle3};
  return m;
}

// Hard ceiling on path speed s_dot: min of per-wheel back-EMF limit and the
// centripetal traction limit.
float calcCeiling(const SplineDerivState& st, const MotorModel& m) {
  float sDotLim = 9999.0f;
  for (int i = 0; i < 4; ++i) {
    const float projTrans = std::sin(m.alpha[i]) * static_cast<float>(st.dx_ds) +
                            std::cos(m.alpha[i]) * static_cast<float>(st.dy_ds);
    const float projRot = -params::get().chassisRadiusM * static_cast<float>(st.dtheta_ds);
    const float Ki = projTrans + projRot;
    if (std::fabs(Ki) > 0.001f) sDotLim = std::min(sDotLim, m.maxWheelVel / std::fabs(Ki));
  }
  if (std::fabs(st.kappa) > 0.001f) {
    const float vCurve = std::sqrt(params::get().aMaxGripAccel / static_cast<float>(std::fabs(st.kappa)));
    const float pathRatio =
        std::sqrt(static_cast<float>(st.dx_ds * st.dx_ds + st.dy_ds * st.dy_ds));
    if (pathRatio > 0.001f) sDotLim = std::min(sDotLim, vCurve / pathRatio);
  }
  return sDotLim;
}

struct DynLimits {
  float accel;
  float decel;
};

DynLimits calcDynLimits(const SplineDerivState& st, float sDot, float vx, float vy, float omega,
                        const MotorModel& m) {
  DynLimits lim{999.0f, 999.0f};
  const float sDotSq = sDot * sDot;
  constexpr float EPS = 0.001f;
  for (int i = 0; i < 4; ++i) {
    const auto& p = params::get();
    const float vWheel = std::sin(m.alpha[i]) * vx + std::cos(m.alpha[i]) * vy -
                         p.chassisRadiusM * omega;
    const float wMotor = vWheel / p.wheelRadiusM;
    const float vEmf = p.motorKv * wMotor;
    const float friction =
        (wMotor > EPS) ? p.motorKs : (wMotor < -EPS) ? -p.motorKs : 0.0f;
    const float vPos = p.motorVBus - friction - vEmf;
    const float vNeg = -p.motorVBus - friction - vEmf;
    const float aWhPos = (vPos * p.wheelRadiusM) / p.motorKa;
    const float aWhNeg = (vNeg * p.wheelRadiusM) / p.motorKa;

    const float Ki = std::sin(m.alpha[i]) * static_cast<float>(st.dx_ds) +
                     std::cos(m.alpha[i]) * static_cast<float>(st.dy_ds) -
                     p.chassisRadiusM * static_cast<float>(st.dtheta_ds);
    const float Ci = std::sin(m.alpha[i]) * static_cast<float>(st.d2x_ds2) +
                     std::cos(m.alpha[i]) * static_cast<float>(st.d2y_ds2) -
                     p.chassisRadiusM * static_cast<float>(st.d2theta_ds2);
    const float curveTerm = Ci * sDotSq;
    if (std::fabs(Ki) > EPS) {
      const float b1 = (aWhPos - curveTerm) / Ki;
      const float b2 = (aWhNeg - curveTerm) / Ki;
      const float localMax = std::max(b1, b2);
      const float localMin = std::min(b1, b2);
      lim.accel = (localMax > 0.0f) ? std::min(lim.accel, localMax) : 0.0f;
      lim.decel = (localMin < 0.0f) ? std::min(lim.decel, std::fabs(localMin)) : 0.0f;
    } else if (curveTerm > aWhPos || curveTerm < aWhNeg) {
      lim.accel = 0.0f;
      lim.decel = 0.0f;
    }
  }
  const float pathRatio =
      std::sqrt(static_cast<float>(st.dx_ds * st.dx_ds + st.dy_ds * st.dy_ds));
  if (pathRatio > EPS) {
    const float vSq = vx * vx + vy * vy;
    const float aLat = std::min(vSq * static_cast<float>(std::fabs(st.kappa)),
                                params::get().aMaxGripAccel);
    const float aTang = std::sqrt(std::max(
        0.0f, params::get().aMaxGripAccel * params::get().aMaxGripAccel - aLat * aLat));
    const float sDdotTr = aTang / pathRatio;
    lim.accel = std::min(lim.accel, sDdotTr);
    lim.decel = std::min(lim.decel, sDdotTr);
  }
  return lim;
}

}  // namespace

std::vector<ProfilePointS> VelocityProfile::computeMotorModel(const HermiteSplineData& spline,
                                                              float vStartMps, float vEndMps,
                                                              int numSteps) {
  if (spline.numSegments == 0 || spline.nodes.empty()) return {};
  const MotorModel m = makeMotorModel();
  const int N = std::max(numSteps, 2);
  const float ds = 1.0f / (N - 1);
  constexpr float EPS = 1e-6f;

  std::vector<SplineDerivState> states(N);
  for (int i = 0; i < N; ++i) states[i] = spline.evalState(static_cast<double>(i) / (N - 1));

  std::vector<float> ceiling(N);
  for (int i = 0; i < N; ++i) ceiling[i] = calcCeiling(states[i], m);

  auto pathRatio = [&](int i) {
    return std::sqrt(
        static_cast<float>(states[i].dx_ds * states[i].dx_ds + states[i].dy_ds * states[i].dy_ds));
  };

  const float pr0 = pathRatio(0);
  float sDotStart = (pr0 > EPS) ? vStartMps / pr0 : 0.0f;
  sDotStart = std::min(sDotStart, ceiling[0]);
  const float prN = pathRatio(N - 1);
  float sDotEnd = (prN > EPS) ? vEndMps / prN : 0.0f;
  sDotEnd = std::min(sDotEnd, ceiling[N - 1]);

  // Forward pass.
  std::vector<float> fwd(N);
  {
    float sDot = sDotStart;
    float sDdot = 0.0f;
    for (int i = 0; i < N; ++i) {
      fwd[i] = sDot;
      if (i == N - 1) break;
      const auto& st = states[i];
      const float vx = static_cast<float>(st.dx_ds) * sDot;
      const float vy = static_cast<float>(st.dy_ds) * sDot;
      const float omega = static_cast<float>(st.dtheta_ds) * sDot;
      const DynLimits dl = calcDynLimits(st, sDot, vx, vy, omega, m);
      const float sdotSafe = std::max(sDot, EPS);
      const float dSdot = params::get().profilerJMax * ds / sdotSafe;
      sDdot += (sDdot < dl.accel) ? dSdot : -dSdot;
      sDdot = std::clamp(sDdot, 0.0f, dl.accel);
      const float v2 = sDot * sDot + 2.0f * sDdot * ds;
      sDot = std::sqrt(std::max(0.0f, v2));
      sDot = std::min(sDot, ceiling[i + 1]);
    }
  }

  // Backward pass.
  std::vector<float> bwd(N);
  {
    float sDot = sDotEnd;
    float sDdotDecel = 0.0f;
    bwd[N - 1] = sDot;
    for (int i = N - 2; i >= 0; --i) {
      const auto& st = states[i + 1];
      const float vx = static_cast<float>(st.dx_ds) * sDot;
      const float vy = static_cast<float>(st.dy_ds) * sDot;
      const float omega = static_cast<float>(st.dtheta_ds) * sDot;
      const DynLimits dl = calcDynLimits(st, sDot, vx, vy, omega, m);
      const float sdotSafe = std::max(sDot, EPS);
      const float dSdot = params::get().profilerJMax * ds / sdotSafe;
      sDdotDecel += dSdot;
      sDdotDecel = std::clamp(sDdotDecel, 0.0f, dl.decel);
      const float v2 = sDot * sDot + 2.0f * sDdotDecel * ds;
      sDot = std::sqrt(std::max(0.0f, v2));
      sDot = std::min(sDot, ceiling[i]);
      bwd[i] = sDot;
    }
  }

  // Intersection + physical command derivation.
  std::vector<ProfilePointS> out(N);
  for (int i = 0; i < N; ++i) {
    const float sDot = std::min({fwd[i], bwd[i], ceiling[i]});
    float sDdot = 0.0f;
    if (i < N - 1) {
      const float sDotNext = std::min({fwd[i + 1], bwd[i + 1], ceiling[i + 1]});
      sDdot = (sDotNext * sDotNext - sDot * sDot) / (2.0f * ds);
    }
    const auto& st = states[i];
    ProfilePointS p;
    p.s = static_cast<float>(i) / (N - 1);
    p.sDot = sDot;
    p.sDdot = sDdot;
    p.vx = static_cast<float>(st.dx_ds) * sDot;
    p.vy = static_cast<float>(st.dy_ds) * sDot;
    p.omega = static_cast<float>(st.dtheta_ds) * sDot;
    p.ax = static_cast<float>(st.d2x_ds2) * sDot * sDot + static_cast<float>(st.dx_ds) * sDdot;
    p.ay = static_cast<float>(st.d2y_ds2) * sDot * sDot + static_cast<float>(st.dy_ds) * sDdot;
    p.alpha =
        static_cast<float>(st.d2theta_ds2) * sDot * sDot + static_cast<float>(st.dtheta_ds) * sDdot;
    out[i] = p;
  }
  return out;
}

std::vector<MotionAction> VelocityProfile::discretizeGlobal(const std::vector<ProfilePointS>& prof,
                                                            int dtMs, int maxActions) {
  std::vector<MotionAction> actions;
  if (prof.size() < 2) {
    actions.push_back({});
    while (static_cast<int>(actions.size()) < maxActions) actions.push_back({});
    return actions;
  }
  const int N = static_cast<int>(prof.size());
  const float ds = 1.0f / (N - 1);
  const float dt = dtMs / 1000.0f;

  // Build cumulative time at each s node from dt = ds / s_dot_avg.
  std::vector<float> tAtNode(N, 0.0f);
  for (int i = 1; i < N; ++i) {
    const float avg = 0.5f * (prof[i - 1].sDot + prof[i].sDot);
    const float segDt = (avg > 1e-4f) ? ds / avg : dt;
    tAtNode[i] = tAtNode[i - 1] + segDt;
  }
  const float totalT = tAtNode[N - 1];

  auto sampleAt = [&](float tQuery) -> MotionAction {
    // Find the node interval containing tQuery and linearly interpolate.
    int hi = 1;
    while (hi < N && tAtNode[hi] < tQuery) ++hi;
    if (hi >= N) hi = N - 1;
    const int lo = hi - 1;
    const float span = std::max(1e-6f, tAtNode[hi] - tAtNode[lo]);
    const float frac = std::clamp((tQuery - tAtNode[lo]) / span, 0.0f, 1.0f);
    MotionAction a;
    a.vx = prof[lo].vx + frac * (prof[hi].vx - prof[lo].vx);
    a.vy = prof[lo].vy + frac * (prof[hi].vy - prof[lo].vy);
    a.omega = prof[lo].omega + frac * (prof[hi].omega - prof[lo].omega);
    a.ax = prof[lo].ax + frac * (prof[hi].ax - prof[lo].ax);
    a.ay = prof[lo].ay + frac * (prof[hi].ay - prof[lo].ay);
    a.alpha = prof[lo].alpha + frac * (prof[hi].alpha - prof[lo].alpha);
    return a;
  };

  for (int k = 0; k < maxActions; ++k) {
    const float tQuery = k * dt;
    if (tQuery > totalT) break;
    actions.push_back(sampleAt(tQuery));
  }
  if (actions.empty()) actions.push_back(sampleAt(0.0f));

  // Pad: hold terminal velocity, zero feed-forward accel.
  MotionAction hold = actions.back();
  hold.ax = hold.ay = hold.alpha = 0.0f;
  while (static_cast<int>(actions.size()) < maxActions) actions.push_back(hold);
  return actions;
}

}  // namespace ballalgo
