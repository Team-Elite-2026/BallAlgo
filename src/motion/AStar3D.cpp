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

float rotationCost(int from, int to, int bins) {
  constexpr float kQuarterTurnCostS = 0.08f;
  const int quarterTurnBins = std::max(1, bins / 4);
  return kQuarterTurnCostS * static_cast<float>(wrapBinDistance(from, to, bins)) /
         static_cast<float>(quarterTurnBins);
}

}  // namespace

AStar3D::AStar3D(float fieldW, float fieldH, int cellMm, int headingBins)
    : fieldW_(fieldW), fieldH_(fieldH), cellMm_(cellMm), hBins_(headingBins) {
  cols_ = static_cast<int>(fieldW / cellMm) + 1;
  rows_ = static_cast<int>(fieldH / cellMm) + 1;
}

int AStar3D::index(int ix, int iy, int it) const {
  return ix + iy * cols_ + it * cols_ * rows_;
}

float AStar3D::heuristic(int ix, int iy, int it, int gx, int gy, int gt) const {
  float dx = (gx - ix) * cellMm_;
  float dy = (gy - iy) * cellMm_;
  return std::hypot(dx, dy) / 1000.f / std::max(config::kVMaxX, config::kVMaxY) +
         rotationCost(it, gt, hBins_);
}

bool AStar3D::plan(float sx, float sy, int stheta, float gx, float gy, int gtheta,
                   std::vector<Waypoint3>& out) {
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

    for (int k = 0; k < 8; ++k) {
      int nix = ix + dx[k], niy = iy + dy[k];
      if (nix < 0 || niy < 0 || nix >= cols_ || niy >= rows_) continue;
      float wx1 = nix * cellMm_, wy1 = niy * cellMm_;
      float phi = std::atan2(wy1 - wy0, wx1 - wx0);
      float dist = std::hypot(wx1 - wx0, wy1 - wy0) / 1000.f;
      float vlim = motion::vMaxDir(phi, config::kVMaxX, config::kVMaxY);
      int nit = headingBinFromRadians(phi, hBins_);
      float step = dist / std::max(vlim, 0.05f) + rotationCost(it, nit, hBins_);
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
    out.push_back({sx, sy, static_cast<float>(st * 360 / hBins_)});
    out.push_back({gx, gy, static_cast<float>(gt * 360 / hBins_)});
    return true;
  }
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
