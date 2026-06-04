#include "motion/VelocityProfile.hpp"

#include "config.hpp"
#include "motion/MotionLimits.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

namespace {

float solveSevenPhaseReach(float v0, float v1, float dist, float aMax, float jMax) {
  if (dist < 1e-6f) return v1;
  float v0sq = v0 * v0, v1sq = v1 * v1;
  float delta = v1sq - v0sq;
  float aEff = std::min(aMax, std::sqrt(std::max(0.f, jMax * dist)));
  if (std::fabs(delta) < 2.f * aEff * dist) {
    float vm = std::sqrt(std::max(0.f, 0.5f * (v0sq + v1sq + 2.f * aEff * dist * (delta > 0 ? 1 : -1))));
    return std::clamp(vm, std::min(v0, v1), std::max(v0, v1));
  }
  if (delta > 0) return std::sqrt(std::max(v0sq + 2.f * aEff * dist, v1sq));
  return std::sqrt(std::max(v1sq, v0sq - 2.f * aEff * dist));
}

}  // namespace

std::vector<ProfileSample> VelocityProfile::build(const std::vector<PathSample>& path,
                                                  float vStart) {
  std::vector<ProfileSample> prof;
  if (path.size() < 2) return prof;
  const int n = static_cast<int>(path.size());
  prof.resize(n);
  for (int i = 0; i < n; ++i) {
    prof[i].sMm = path[i].sMm;
    float phi = 0;
    if (i < n - 1) {
      phi = std::atan2(path[i + 1].yMm - path[i].yMm, path[i + 1].xMm - path[i].xMm);
    } else if (i > 0) {
      phi = std::atan2(path[i].yMm - path[i - 1].yMm, path[i].xMm - path[i - 1].xMm);
    }
    prof[i].phi = phi;
    prof[i].kappa = 0;
    if (i >= 2) {
      float dx1 = path[i].xMm - path[i - 1].xMm;
      float dy1 = path[i].yMm - path[i - 1].yMm;
      float dx2 = path[i - 1].xMm - path[i - 2].xMm;
      float dy2 = path[i - 1].yMm - path[i - 2].yMm;
      float cross = std::fabs(dx1 * dy2 - dy1 * dx2);
      float denom = std::hypot(dx1, dy1) * std::hypot(dx2, dy2) * std::hypot(dx1 + dx2, dy1 + dy2);
      // Geometry is in millimeters; convert curvature from 1/mm to 1/m so
      // the lateral-acceleration speed cap stays dimensionally correct.
      if (denom > 1e-3f) prof[i].kappa = 1000.f * cross / (denom + 1e-9f);
    }
    float vcurve = (prof[i].kappa > 1e-6f)
                       ? std::sqrt(config::kAMaxLateral / prof[i].kappa)
                       : config::kVMaxX;
    prof[i].vCap = std::min(motion::vMaxDir(phi, config::kVMaxX, config::kVMaxY), vcurve);
    prof[i].v = prof[i].vCap;
  }
  prof[n - 1].v = 0;
  for (int i = n - 2; i >= 0; --i) {
    float ds = (prof[i + 1].sMm - prof[i].sMm) / 1000.f;
    float am = motion::aMaxDir(prof[i].phi, config::kAMaxX, config::kAMaxY);
    float reachable = solveSevenPhaseReach(prof[i + 1].v, prof[i].vCap, ds, am, config::kJMaxTangential);
    prof[i].v = std::min(prof[i].vCap, reachable);
  }
  prof[0].v = std::min(prof[0].vCap, vStart);
  for (int i = 0; i < n - 1; ++i) {
    float ds = (prof[i + 1].sMm - prof[i].sMm) / 1000.f;
    float am = motion::aMaxDir(prof[i].phi, config::kAMaxX, config::kAMaxY);
    float reachable = solveSevenPhaseReach(prof[i].v, prof[i + 1].v, ds, am, config::kJMaxTangential);
    if (reachable < prof[i + 1].v) prof[i + 1].v = reachable;
    prof[i].a = (ds > 1e-6f) ? (prof[i + 1].v * prof[i + 1].v - prof[i].v * prof[i].v) / (2.f * ds) : 0;
  }
  return prof;
}

std::vector<MotionAction> VelocityProfile::discretize(const std::vector<ProfileSample>& prof,
                                                      const std::vector<PathSample>& path,
                                                      float headingDeg, int dtMs, int maxActions) {
  (void)path;
  std::vector<MotionAction> actions;
  if (prof.empty()) {
    actions.push_back({});
    return actions;
  }
  const float dt = dtMs / 1000.f;
  float t = 0;
  size_t seg = 0;
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h), s = std::sin(h);
  while (static_cast<int>(actions.size()) < maxActions && seg < prof.size() - 1) {
    float v = prof[seg].v;
    float phi = prof[seg].phi;
    float vxF = v * std::cos(phi);
    float vyF = v * std::sin(phi);
    MotionAction a;
    a.vx = static_cast<float>(c * vxF + s * vyF);
    a.vy = static_cast<float>(-s * vxF + c * vyF);
    a.ax = static_cast<float>(prof[seg].a * std::cos(phi));
    a.ay = static_cast<float>(prof[seg].a * std::sin(phi));
    a.omega = 0;
    a.alpha = 0;
    actions.push_back(a);
    float ds = (prof[seg + 1].sMm - prof[seg].sMm) / 1000.f;
    float segT = (v > 0.05f) ? ds / v : dt;
    t += dt;
    if (t >= segT) {
      t = 0;
      ++seg;
    }
  }
  MotionAction hold = actions.empty() ? MotionAction{} : actions.back();
  // Hold terminal velocity without repeating the last segment's acceleration
  // spike into padded trailing actions.
  hold.ax = 0;
  hold.ay = 0;
  hold.alpha = 0;
  while (static_cast<int>(actions.size()) < maxActions) actions.push_back(hold);
  return actions;
}

}  // namespace ballalgo
