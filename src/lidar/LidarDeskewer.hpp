#pragma once

#include "lidar/Ld19Reader.hpp"

#include <cstdint>
#include <deque>
#include <vector>

namespace ballalgo {

// One timestamped body-frame motion state sample fed by MotionPipeline::predictStep.
struct MotionSample {
  uint64_t timestampUs = 0;
  float    vxBodyMmS   = 0;  // lateral velocity (+right),   mm/s
  float    vyBodyMmS   = 0;  // forward  velocity (+forward), mm/s
  float    omegaRadS   = 0;  // yaw rate, rad/s (CCW positive in math convention)
};

// Per-update statistics; useful with kDebugDeskew = true.
struct DeskewStats {
  bool     enabled           = false;
  bool     fallbackUsed      = false;  // true → raw scan passed through unchanged
  uint64_t scanStartUs       = 0;
  uint64_t scanEndUs         = 0;
  uint64_t refTimeUs         = 0;
  int      pointsCorrected   = 0;
  float    estimatedTxMm     = 0;  // mean translational correction x over window
  float    estimatedTyMm     = 0;  // mean translational correction y over window
  float    estimatedRotRad   = 0;  // mean rotational correction over window
  int      interpolatedCount = 0;
  int      extrapolatedCount = 0;
  bool     motionDataStale   = false;
};

// LidarDeskewer applies 2D SE(2) per-point motion compensation before scan matching.
//
// Algorithm (LIO-SAM / Autoware 2D distortion corrector style):
//   Each LidarPoint carries a timestampUs set by Ld19Reader::parseFrame from the
//   LD19 hardware clock.  For every point p_i at time t_i, the deskewer estimates
//   the body-frame displacement (Δx, Δy, Δθ) between the reference time t_ref and
//   t_i using the motion history buffer, then corrects the point so it appears as if
//   it were measured at t_ref:
//
//       p_ref = R(+Δθ) · p_i + [Δx, Δy]ᵀ
//
//   Falls back to the raw scan (no crash) if timestamps or motion data are missing.
//
// Thread-safety: NOT thread-safe. Owned and used exclusively by MotionPipeline on
// the single main loop thread.
class LidarDeskewer {
 public:
  LidarDeskewer() = default;

  // Call every main-loop iteration from MotionPipeline::predictStep.
  // timestampUs must be monotonically non-decreasing (steady_clock microseconds).
  void addMotionSample(uint64_t timestampUs,
                       float vxBodyMmS,
                       float vyBodyMmS,
                       float omegaRadS);

  // Correct pts in-place using 2D SE(2) per-point motion compensation.
  // Returns statistics.  If kEnableLidarDeskew is false, or motion/timestamp data
  // is unavailable/stale, pts is returned unchanged with fallbackUsed = true.
  DeskewStats deskew(std::vector<LidarPoint>& pts) const;

 private:
  // Interpolate (or extrapolate within kMaxExtrapolationS) the motion state at
  // queryUs.  Returns false if interpolation is impossible.
  bool interpolateMotion(uint64_t queryUs,
                         float& vxOut,
                         float& vyOut,
                         float& omegaOut,
                         bool& wasExtrapolated) const;

  // Choose the reference timestamp from the point window.
  static uint64_t computeRefTime(const std::vector<LidarPoint>& pts, int mode);

  // Apply the SE(2) correction in-place to a single point.
  // deltaTx/Ty: body-frame translation from t_ref to t_i (mm).
  // deltaTheta: yaw rotation from t_ref to t_i (rad, CCW positive).
  static void correctPoint(LidarPoint& p,
                           float deltaTx,
                           float deltaTy,
                           float deltaTheta);

  // ~4 s of history at 120 Hz is more than enough for a 100 ms scan window.
  static constexpr int kMaxHistorySize = 512;

  std::deque<MotionSample> history_;
};

}  // namespace ballalgo
