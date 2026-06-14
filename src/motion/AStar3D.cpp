#include "motion/AStar3D.hpp"

#include "config.hpp"
#include "motion/MotionLimits.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace ballalgo {

namespace {

int wrapBinDistance(int from, int to, int bins) {
  const int diff = std::abs(to - from);
  return std::min(diff, bins - diff);
}

int headingBinFromRadians(float radians, int bins) {
  const float angleDeg = radians * 180.f / static_cast<float>(M_PI);
  const float wrapped = std::fmod(std::fmod(angleDeg, 360.f) + 360.f, 360.f);
  const float stepDeg = 360.f / static_cast<float>(bins);
  return static_cast<int>(wrapped / stepDeg) % bins;
}

// Heading-change penalty (seconds). Δθ in radians times the K_rot weight.
float rotationCost(int from, int to, int bins) {
  const float dThetaRad = static_cast<float>(wrapBinDistance(from, to, bins)) *
                          (2.0f * static_cast<float>(M_PI) / static_cast<float>(bins));
  return config::kAstarKRotSPerRad * dThetaRad;
}

}  // namespace

AStar3D::AStar3D(float fieldW, float fieldH, int cellMm, int headingBins)
    : fieldW_(fieldW), fieldH_(fieldH), cellMm_(cellMm), hBins_(headingBins) {
  cols_ = static_cast<int>(fieldW / cellMm) + 1;
  rows_ = static_cast<int>(fieldH / cellMm) + 1;
  maxStraightVMps_ = std::max(0.05f, motion::wheelProjVMaxStraight());
}

void AStar3D::setObstacle(float xMm, float yMm, float clearMm) {
  hasObstacle_ = clearMm > 0.f;
  obsXMm_ = xMm;
  obsYMm_ = yMm;
  obsClearMm_ = clearMm;
}

void AStar3D::clearObstacle() { hasObstacle_ = false; }

bool AStar3D::blocked(int ix, int iy) const {
  if (!hasObstacle_) return false;
  const float wx = ix * cellMm_;
  const float wy = iy * cellMm_;
  const float dx = wx - obsXMm_;
  const float dy = wy - obsYMm_;
  return (dx * dx + dy * dy) < (obsClearMm_ * obsClearMm_);
}

int AStar3D::index(int ix, int iy, int it) const {
  return ix + iy * cols_ + it * cols_ * rows_;
}

float AStar3D::heuristic(int ix, int iy, int it, int gx, int gy, int gt) const {
  float dx = (gx - ix) * cellMm_;
  float dy = (gy - iy) * cellMm_;
  // Time-to-go assuming straight-line travel at the absolute max speed
  // (independent of drive angle) plus the minimum rotation cost.
  return std::hypot(dx, dy) / 1000.f / maxStraightVMps_ + rotationCost(it, gt, hBins_);
}

bool AStar3D::plan(float sx, float sy, int stheta, float gx, float gy, int gtheta,
                   std::vector<Waypoint3>& out, float* costS) {
  out.clear();
  auto toCell = [&](float x, float y) {
    return std::pair{
        std::clamp(static_cast<int>(x / cellMm_), 0, cols_ - 1),
        std::clamp(static_cast<int>(y / cellMm_), 0, rows_ - 1)};
  };
  auto [six, siy] = toCell(sx, sy);
  auto [gix, giy] = toCell(gx, gy);
  int st = std::clamp(stheta, 0, hBins_ - 1);
  int gt = std::clamp(gtheta, 0, hBins_ - 1);
  const int N = cols_ * rows_ * hBins_;
  gScore_.assign(N, std::numeric_limits<float>::infinity());
  parent_.assign(N, -1);
  closed_.assign(N, 0);
  const int start = index(six, siy, st);
  const int goal = index(gix, giy, gt);
  gScore_[start] = 0;
  using QItem = std::pair<float, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  open.push({heuristic(six, siy, st, gix, giy, gt), start});
  const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
  while (!open.empty()) {
    int cur = open.top().second;
    open.pop();
    if (closed_[cur]) continue;
    closed_[cur] = 1;
    if (cur == goal) break;
    int ix = cur % cols_;
    int iy = (cur / cols_) % rows_;
    int it = cur / (cols_ * rows_);
    float wx0 = ix * cellMm_, wy0 = iy * cellMm_;

    for (int turn : {-1, 1}) {
      int nit = (it + turn + hBins_) % hBins_;
      int nb = index(ix, iy, nit);
      if (closed_[nb]) continue;
      float tg = gScore_[cur] + rotationCost(it, nit, hBins_);
      if (tg < gScore_[nb]) {
        gScore_[nb] = tg;
        parent_[nb] = cur;
        float f = tg + heuristic(ix, iy, nit, gix, giy, gt);
        open.push({f, nb});
      }
    }

    // Travel direction used to ARRIVE at this node (for the K_curve penalty).
    const int parIdx = parent_[cur];
    bool hasTravelIn = false;
    float phiInSpec = 0.f;
    if (parIdx >= 0) {
      const int pix = parIdx % cols_;
      const int piy = (parIdx / cols_) % rows_;
      if (pix != ix || piy != iy) {
        phiInSpec = std::atan2(static_cast<float>(ix - pix), static_cast<float>(iy - piy));
        hasTravelIn = true;
      }
    }

    for (int k = 0; k < 8; ++k) {
      int nix = ix + dx[k], niy = iy + dy[k];
      if (nix < 0 || niy < 0 || nix >= cols_ || niy >= rows_) continue;
      if (blocked(nix, niy)) continue;  // ball / enemy inflation
      float wx1 = nix * cellMm_, wy1 = niy * cellMm_;
      const float dxw = wx1 - wx0;
      const float dyw = wy1 - wy0;
      float dist = std::hypot(dxw, dyw) / 1000.f;

      // Asymmetric velocity ceiling: travel direction mapped into the robot
      // body frame at node A (spec: phi_local = phi_global + theta_A).
      const float phiTravelSpec = std::atan2(dxw, dyw);  // 0 = +y forward
      const float thetaA = static_cast<float>(it) * (2.0f * static_cast<float>(M_PI) /
                                                     static_cast<float>(hBins_));
      const float vlim = motion::wheelProjVMax(phiTravelSpec + thetaA);

      int nit = it;  // holonomic: heading is independent of travel direction

      // Curve penalty: change in travel direction from the incoming move.
      float curveCost = 0.f;
      if (hasTravelIn) {
        float dphi = phiTravelSpec - phiInSpec;
        while (dphi > static_cast<float>(M_PI)) dphi -= 2.0f * static_cast<float>(M_PI);
        while (dphi < -static_cast<float>(M_PI)) dphi += 2.0f * static_cast<float>(M_PI);
        curveCost = config::kAstarKCurveSPerRad * std::fabs(dphi);
      }

      float step = dist / std::max(vlim, 0.05f) + rotationCost(it, nit, hBins_) + curveCost;
      int nb = index(nix, niy, nit);
      if (closed_[nb]) continue;
      float tg = gScore_[cur] + step;
      if (tg < gScore_[nb]) {
        gScore_[nb] = tg;
        parent_[nb] = cur;
        float f = tg + heuristic(nix, niy, nit, gix, giy, gt);
        open.push({f, nb});
      }
    }
  }
  if (parent_[goal] < 0 && goal != start) {
    if (costS != nullptr) *costS = heuristic(six, siy, st, gix, giy, gt);
    out.push_back({sx, sy, static_cast<float>(st * 360 / hBins_)});
    out.push_back({gx, gy, static_cast<float>(gt * 360 / hBins_)});
    return true;
  }
  if (costS != nullptr) *costS = gScore_[goal];
  std::vector<int> chain;
  for (int c = goal; c >= 0; c = parent_[c]) chain.push_back(c);
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
    int idx = *it;
    int ix = idx % cols_;
    int iy = (idx / cols_) % rows_;
    int ih = idx / (cols_ * rows_);
    Waypoint3 w;
    w.xMm = ix * cellMm_;
    w.yMm = iy * cellMm_;
    w.thetaDeg = ih * (360.f / hBins_);
    out.push_back(w);
  }
  return !out.empty();
}

}  // namespace ballalgo
