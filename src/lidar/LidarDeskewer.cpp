#include "lidar/LidarDeskewer.hpp"

#include "config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ballalgo {

// ---------------------------------------------------------------------------
// addMotionSample
// ---------------------------------------------------------------------------

void LidarDeskewer::addMotionSample(uint64_t timestampUs,
                                    float vxBodyMmS,
                                    float vyBodyMmS,
                                    float omegaRadS) {
  // Discard samples that go backward in time (steady_clock jitter guard).
  if (!history_.empty() && timestampUs <= history_.back().timestampUs) return;

  history_.push_back({timestampUs, vxBodyMmS, vyBodyMmS, omegaRadS});

  while (static_cast<int>(history_.size()) > kMaxHistorySize) {
    history_.pop_front();
  }
}

// ---------------------------------------------------------------------------
// interpolateMotion  (Cartographer PoseExtrapolator style)
// ---------------------------------------------------------------------------

bool LidarDeskewer::interpolateMotion(uint64_t queryUs,
                                      float& vxOut,
                                      float& vyOut,
                                      float& omegaOut,
                                      bool& wasExtrapolated) const {
  if (history_.empty()) return false;

  const uint64_t maxAgeUs    = static_cast<uint64_t>(config::kMaxMotionDataAgeS * 1e6f);
  const uint64_t maxExtrapUs = static_cast<uint64_t>(config::kMaxExtrapolationS  * 1e6f);

  const uint64_t oldestUs = history_.front().timestampUs;
  const uint64_t newestUs = history_.back().timestampUs;

  // Staleness check: if query is far outside any valid range, refuse.
  if (queryUs > newestUs && (queryUs - newestUs) > maxAgeUs) return false;
  if (queryUs < oldestUs && (oldestUs - queryUs) > maxAgeUs) return false;

  // Extrapolate backward from oldest sample.
  if (queryUs <= oldestUs) {
    if ((oldestUs - queryUs) > maxExtrapUs) return false;
    vxOut    = history_.front().vxBodyMmS;
    vyOut    = history_.front().vyBodyMmS;
    omegaOut = history_.front().omegaRadS;
    wasExtrapolated = true;
    return true;
  }

  // Extrapolate forward from newest sample.
  if (queryUs >= newestUs) {
    if ((queryUs - newestUs) > maxExtrapUs) return false;
    vxOut    = history_.back().vxBodyMmS;
    vyOut    = history_.back().vyBodyMmS;
    omegaOut = history_.back().omegaRadS;
    wasExtrapolated = true;
    return true;
  }

  // Interpolate between the two bracketing samples.
  auto it = std::lower_bound(
      history_.begin(), history_.end(), queryUs,
      [](const MotionSample& s, uint64_t t) { return s.timestampUs < t; });

  const MotionSample& hi = *it;
  const MotionSample& lo = *std::prev(it);
  const float span = static_cast<float>(hi.timestampUs - lo.timestampUs);
  const float alpha = (span > 0.f)
      ? static_cast<float>(queryUs - lo.timestampUs) / span
      : 0.f;

  vxOut    = lo.vxBodyMmS + alpha * (hi.vxBodyMmS - lo.vxBodyMmS);
  vyOut    = lo.vyBodyMmS + alpha * (hi.vyBodyMmS - lo.vyBodyMmS);
  omegaOut = lo.omegaRadS + alpha * (hi.omegaRadS  - lo.omegaRadS);
  wasExtrapolated = false;
  return true;
}

// ---------------------------------------------------------------------------
// computeRefTime
// ---------------------------------------------------------------------------

uint64_t LidarDeskewer::computeRefTime(const std::vector<LidarPoint>& pts, int mode) {
  if (pts.empty()) return 0;
  switch (mode) {
    case 0:  return pts.front().timestampUs;
    case 2:  return pts.back().timestampUs;
    case 1:
    default:
      // Divide before adding to avoid uint64_t overflow.
      return pts.front().timestampUs / 2u + pts.back().timestampUs / 2u;
  }
}

// ---------------------------------------------------------------------------
// correctPoint  —  2D SE(2) per-point motion compensation
// ---------------------------------------------------------------------------
//
// Body-frame convention (matches RobotSerial / PoseKalman):
//   angleCd = 0 → forward → +y body axis
//   angleCd = 9000 → right → +x body axis
//
// Formula (undo robot motion from t_ref to t_i):
//   p_ref = R(−Δθ) · (p_i − [Δx, Δy]ᵀ)
//
// where Δθ = omegaRadS * (t_i - t_ref)
//       Δx = vxBodyMmS * (t_i - t_ref)
//       Δy = vyBodyMmS * (t_i - t_ref)

void LidarDeskewer::correctPoint(LidarPoint& p,
                                  float deltaTx,
                                  float deltaTy,
                                  float deltaTheta) {
  // Body-frame Cartesian: sin/cos match angleCd=0 → forward (+y).
  const float angleRad = static_cast<float>(p.angleCd) * 0.01f *
                         static_cast<float>(M_PI / 180.0);
  const float px_i = static_cast<float>(p.distanceMm) * std::sin(angleRad);  // +x = right
  const float py_i = static_cast<float>(p.distanceMm) * std::cos(angleRad);  // +y = forward

  // Remove translational displacement (subtract motion from t_ref to t_i).
  const float px_c = px_i - deltaTx;
  const float py_c = py_i - deltaTy;

  // Rotate back by −deltaTheta (undo rotational displacement).
  const float cosD  = std::cos(-deltaTheta);
  const float sinD  = std::sin(-deltaTheta);
  const float px_ref = cosD * px_c - sinD * py_c;
  const float py_ref = sinD * px_c + cosD * py_c;

  // Convert corrected Cartesian back to polar.
  const float newDist = std::hypot(px_ref, py_ref);
  if (newDist < 1.0f) return;  // degenerate: point collapsed to origin, leave unchanged

  // atan2(x, y) with x=right, y=forward gives 0 for forward direction.
  float newAngleDeg = std::atan2(px_ref, py_ref) * static_cast<float>(180.0 / M_PI);
  if (newAngleDeg < 0.f) newAngleDeg += 360.f;

  p.distanceMm = static_cast<uint16_t>(std::lround(newDist));
  p.angleCd    = static_cast<uint16_t>(static_cast<uint32_t>(std::lround(newAngleDeg * 100.f)) % 36000u);
}

// ---------------------------------------------------------------------------
// deskew  —  main entry point
// ---------------------------------------------------------------------------

DeskewStats LidarDeskewer::deskew(std::vector<LidarPoint>& pts) const {
  DeskewStats stats;
  stats.enabled = config::kEnableLidarDeskew;

  if (!config::kEnableLidarDeskew) return stats;
  if (pts.empty()) return stats;

  // Require at least one point with a non-zero hardware timestamp.
  bool hasTimestamps = false;
  for (const auto& p : pts) {
    if (p.timestampUs != 0) { hasTimestamps = true; break; }
  }
  if (!hasTimestamps || history_.empty()) {
    stats.fallbackUsed = true;
    std::fprintf(stderr, "[DESKEW] WARNING: no timestamps or motion data — raw scan\n");
    std::fflush(stderr);
    return stats;
  }

  const uint64_t refTimeUs = computeRefTime(pts, config::kDeskewRefTimeMode);
  stats.refTimeUs   = refTimeUs;
  stats.scanStartUs = pts.front().timestampUs;
  stats.scanEndUs   = pts.back().timestampUs;

  // Scan-level staleness: refuse if reference time is too far from newest history.
  const uint64_t maxAgeUs = static_cast<uint64_t>(config::kMaxMotionDataAgeS * 1e6f);
  if (refTimeUs > history_.back().timestampUs &&
      (refTimeUs - history_.back().timestampUs) > maxAgeUs) {
    stats.fallbackUsed    = true;
    stats.motionDataStale = true;
    std::fprintf(stderr, "[DESKEW] WARNING: motion data stale — raw scan\n");
    std::fflush(stderr);
    return stats;
  }

  float totalTx = 0.f, totalTy = 0.f, totalRot = 0.f;

  for (auto& p : pts) {
    if (p.timestampUs == 0) continue;  // skip unset timestamps

    float vx = 0.f, vy = 0.f, omega = 0.f;
    bool wasExtrapolated = false;
    if (!interpolateMotion(p.timestampUs, vx, vy, omega, wasExtrapolated)) {
      // Per-point extrapolation limit exceeded: leave this point uncorrected
      // but do NOT trigger a scan-level fallback — a few edge points are acceptable.
      continue;
    }

    // Δt in seconds: positive if point is after ref, negative if before.
    const float dtS = static_cast<float>(
        static_cast<int64_t>(p.timestampUs) - static_cast<int64_t>(refTimeUs)) * 1e-6f;

    const float deltaTx    = vx    * dtS;
    const float deltaTy    = vy    * dtS;
    const float deltaTheta = omega * dtS;

    correctPoint(p, deltaTx, deltaTy, deltaTheta);

    totalTx  += deltaTx;
    totalTy  += deltaTy;
    totalRot += deltaTheta;
    ++stats.pointsCorrected;
    if (wasExtrapolated) ++stats.extrapolatedCount;
    else                 ++stats.interpolatedCount;
  }

  if (stats.pointsCorrected > 0) {
    stats.estimatedTxMm  = totalTx  / static_cast<float>(stats.pointsCorrected);
    stats.estimatedTyMm  = totalTy  / static_cast<float>(stats.pointsCorrected);
    stats.estimatedRotRad= totalRot / static_cast<float>(stats.pointsCorrected);
  }

  if (config::kDebugDeskew) {
    std::fprintf(stderr,
        "[DESKEW] enabled=1 fallback=0 start=%llu end=%llu ref=%llu "
        "corrected=%d interp=%d extrap=%d tx=%.2f ty=%.2f rot=%.4f stale=0\n",
        static_cast<unsigned long long>(stats.scanStartUs),
        static_cast<unsigned long long>(stats.scanEndUs),
        static_cast<unsigned long long>(stats.refTimeUs),
        stats.pointsCorrected,
        stats.interpolatedCount,
        stats.extrapolatedCount,
        stats.estimatedTxMm,
        stats.estimatedTyMm,
        stats.estimatedRotRad);
    std::fflush(stderr);
  }

  return stats;
}

}  // namespace ballalgo
