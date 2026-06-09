#include "lidar/LidarLocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>
#include <unordered_map>

namespace ballalgo {

LidarLocalizer::LidarLocalizer(float fieldW, float fieldH, float yawOffsetDeg)
    : fieldW_(fieldW),
      fieldH_(fieldH),
      yawOffset_(yawOffsetDeg),
      houghAnglesX_(buildHoughAngles(0.f)),
      houghAnglesY_(buildHoughAngles(90.f)) {}

float LidarLocalizer::clamp(float value, float lower, float upper) {
  return std::max(lower, std::min(upper, value));
}

float LidarLocalizer::median(std::vector<float> values) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  if ((values.size() % 2) == 0) {
    return 0.5f * (values[middle - 1] + values[middle]);
  }
  return values[middle];
}

float LidarLocalizer::percentile(std::vector<float> values, float fraction) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  std::sort(values.begin(), values.end());
  if (values.size() == 1) return values.front();
  const float position = static_cast<float>(values.size() - 1) * fraction;
  const auto lower = static_cast<size_t>(std::floor(position));
  const auto upper = static_cast<size_t>(std::ceil(position));
  if (lower == upper) return values[lower];
  const float ratio = position - static_cast<float>(lower);
  return values[lower] * (1.f - ratio) + values[upper] * ratio;
}

float LidarLocalizer::wallAxisEstimate(float distanceMm, float directionComponent,
                                       float axisLimitMm) {
  return (directionComponent > 0.f) ? (axisLimitMm - distanceMm * directionComponent)
                                    : (-distanceMm * directionComponent);
}

float LidarLocalizer::rayEndpointGap(const Ray& a, const Ray& b) {
  return std::hypot(a.relXMm - b.relXMm, a.relYMm - b.relYMm);
}

std::vector<LidarLocalizer::HoughAngle> LidarLocalizer::buildHoughAngles(
    float centerAngleDeg) const {
  std::vector<HoughAngle> angles;
  const int steps = static_cast<int>((houghTiltDeg_ * 2.f) / houghAngleStepDeg_) + 1;
  const float startAngle = centerAngleDeg - houghTiltDeg_;
  angles.reserve(static_cast<size_t>(steps));
  for (int index = 0; index < steps; ++index) {
    const float angleDeg = startAngle + static_cast<float>(index) * houghAngleStepDeg_;
    const float radians = angleDeg * static_cast<float>(M_PI / 180.0);
    angles.push_back({angleDeg, std::cos(radians), std::sin(radians)});
  }
  return angles;
}

std::vector<LidarLocalizer::Ray> LidarLocalizer::fieldRays(
    const std::vector<LidarPoint>& points, float headingDeg) const {
  std::vector<Ray> rays;
  rays.reserve(points.size());
  for (const auto& point : points) {
    const float distanceMm = static_cast<float>(point.distanceMm);
    if (distanceMm < minDist_ || distanceMm > maxDist_ || point.intensity < minIntensity_) {
      continue;
    }

    const float fieldAngleDeg = headingDeg + yawOffset_ + point.angleCd * 0.01f;
    const float radians = fieldAngleDeg * static_cast<float>(M_PI / 180.0);
    const float dx = std::cos(radians);
    const float dy = std::sin(radians);
    rays.push_back({point.angleCd, distanceMm, dx, dy, distanceMm * dx, distanceMm * dy});
  }
  return filterIsolatedRays(std::move(rays));
}

std::vector<LidarLocalizer::Ray> LidarLocalizer::filterIsolatedRays(std::vector<Ray> rays) const {
  if (rays.size() < 3) return rays;

  std::sort(rays.begin(), rays.end(),
            [](const Ray& lhs, const Ray& rhs) { return lhs.angleCd < rhs.angleCd; });

  std::vector<Ray> kept;
  kept.reserve(rays.size());
  for (size_t index = 0; index < rays.size(); ++index) {
    const Ray& ray = rays[index];
    const Ray& prev = rays[(index + rays.size() - 1) % rays.size()];
    const Ray& next = rays[(index + 1) % rays.size()];
    if (rayEndpointGap(ray, prev) <= neighborGapMm_ ||
        rayEndpointGap(ray, next) <= neighborGapMm_) {
      kept.push_back(ray);
    }
  }
  return kept;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::clusterEstimates(
    const std::vector<float>& estimates) const {
  if (estimates.empty()) return {};

  std::vector<float> ordered = estimates;
  std::sort(ordered.begin(), ordered.end());

  std::vector<AxisCandidate> clusters;
  size_t start = 0;
  for (size_t index = 1; index <= ordered.size(); ++index) {
    if (index < ordered.size() && (ordered[index] - ordered[index - 1]) <= clusterGapMm_) continue;

    std::vector<float> clusterValues(ordered.begin() + static_cast<std::ptrdiff_t>(start),
                                     ordered.begin() + static_cast<std::ptrdiff_t>(index));
    const size_t q1Index = clusterValues.size() / 4;
    const size_t q3Index = (3 * clusterValues.size()) / 4;
    clusters.push_back(
        {median(clusterValues), static_cast<int>(clusterValues.size()),
         clusterValues[q3Index] - clusterValues[q1Index]});
    start = index;
  }
  return clusters;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::candidateAxisClusters(
    const std::vector<float>& estimates, std::optional<float> prevValue, float axisLimitMm) const {
  if (static_cast<int>(estimates.size()) < minAxisSamples_) return {};

  std::vector<AxisCandidate> clusters;
  for (const auto& cluster : clusterEstimates(estimates)) {
    if (cluster.count >= minAxisSamples_) clusters.push_back(cluster);
  }
  if (clusters.empty()) return {};

  const float center = axisLimitMm * 0.5f;
  std::sort(clusters.begin(), clusters.end(),
            [&](const AxisCandidate& lhs, const AxisCandidate& rhs) {
              const float lhsContinuity =
                  prevValue ? std::fabs(lhs.median - *prevValue) : std::fabs(lhs.median - center);
              const float rhsContinuity =
                  prevValue ? std::fabs(rhs.median - *prevValue) : std::fabs(rhs.median - center);
              if (lhs.count != rhs.count) return lhs.count > rhs.count;
              if (lhs.spread != rhs.spread) return lhs.spread < rhs.spread;
              return lhsContinuity < rhsContinuity;
            });

  if (clusters.size() > static_cast<size_t>(maxCandidateClusters_)) {
    clusters.resize(static_cast<size_t>(maxCandidateClusters_));
  }
  return clusters;
}

std::optional<LidarLocalizer::AxisCandidate> LidarLocalizer::lineFeatureFromRays(
    const std::vector<Ray>& rays, bool xAxis) const {
  if (static_cast<int>(rays.size()) < minLineSamples_) return std::nullopt;

  std::vector<float> coords;
  std::vector<float> perps;
  coords.reserve(rays.size());
  perps.reserve(rays.size());
  for (const auto& ray : rays) {
    coords.push_back(xAxis ? ray.relXMm : ray.relYMm);
    perps.push_back(xAxis ? ray.relYMm : ray.relXMm);
  }
  std::sort(coords.begin(), coords.end());
  std::sort(perps.begin(), perps.end());

  const float extent = percentile(perps, 0.90f) - percentile(perps, 0.10f);
  if (extent < minLineExtentMm_) return std::nullopt;

  const size_t q1Index = coords.size() / 4;
  const size_t q3Index = (3 * coords.size()) / 4;
  AxisCandidate feature;
  feature.median = median(coords);
  feature.count = static_cast<int>(rays.size());
  feature.spread = coords[q3Index] - coords[q1Index];
  feature.lineExtentMm = extent;
  return feature;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::candidateLineAxisClusters(
    const std::vector<Ray>& rays, bool xAxis, std::optional<float> prevValue,
    float axisLimitMm) const {
  if (static_cast<int>(rays.size()) < minLineSamples_) return {};

  std::vector<Ray> ordered = rays;
  std::sort(ordered.begin(), ordered.end(), [&](const Ray& lhs, const Ray& rhs) {
    return (xAxis ? lhs.relXMm : lhs.relYMm) < (xAxis ? rhs.relXMm : rhs.relYMm);
  });

  std::vector<AxisCandidate> features;
  size_t start = 0;
  for (size_t index = 1; index <= ordered.size(); ++index) {
    if (index < ordered.size()) {
      const float current = xAxis ? ordered[index].relXMm : ordered[index].relYMm;
      const float previous = xAxis ? ordered[index - 1].relXMm : ordered[index - 1].relYMm;
      if ((current - previous) <= lineClusterGapMm_) continue;
    }

    std::vector<Ray> group(ordered.begin() + static_cast<std::ptrdiff_t>(start),
                           ordered.begin() + static_cast<std::ptrdiff_t>(index));
    if (auto feature = lineFeatureFromRays(group, xAxis)) features.push_back(*feature);
    start = index;
  }

  std::vector<AxisCandidate> candidates;
  for (auto feature : features) {
    for (float wallPosition : {0.f, axisLimitMm}) {
      const float poseValue = wallPosition - feature.median;
      if (poseValue < 0.f || poseValue > axisLimitMm) continue;
      feature.median = poseValue;
      feature.lineOffsetMm = wallPosition - poseValue;
      feature.wallPositionMm = wallPosition;
      feature.hasWallPosition = true;
      feature.source = CandidateSource::Line;
      candidates.push_back(feature);
    }
  }

  if (candidates.empty()) return {};

  const float center = axisLimitMm * 0.5f;
  std::sort(candidates.begin(), candidates.end(),
            [&](const AxisCandidate& lhs, const AxisCandidate& rhs) {
              const float lhsContinuity =
                  prevValue ? std::fabs(lhs.median - *prevValue) : std::fabs(lhs.median - center);
              const float rhsContinuity =
                  prevValue ? std::fabs(rhs.median - *prevValue) : std::fabs(rhs.median - center);
              if (lhs.count != rhs.count) return lhs.count > rhs.count;
              if (lhs.lineExtentMm != rhs.lineExtentMm) return lhs.lineExtentMm > rhs.lineExtentMm;
              if (lhs.spread != rhs.spread) return lhs.spread < rhs.spread;
              return lhsContinuity < rhsContinuity;
            });

  if (candidates.size() > static_cast<size_t>(maxCandidateClusters_)) {
    candidates.resize(static_cast<size_t>(maxCandidateClusters_));
  }
  return candidates;
}

std::optional<LidarLocalizer::AxisCandidate> LidarLocalizer::fitHoughLine(
    const std::vector<Ray>& rays, bool xAxis, const HoughAngle& angle, float peakRhoMm,
    int voteCount) const {
  std::vector<float> inlierRhos;
  std::vector<float> inlierPerps;
  inlierRhos.reserve(rays.size());
  inlierPerps.reserve(rays.size());

  for (const auto& ray : rays) {
    const float rho = ray.relXMm * angle.cosValue + ray.relYMm * angle.sinValue;
    if (std::fabs(rho - peakRhoMm) > houghLineToleranceMm_) continue;
    inlierRhos.push_back(rho);
    inlierPerps.push_back(xAxis ? ray.relYMm : ray.relXMm);
  }

  if (static_cast<int>(inlierRhos.size()) < minLineSamples_) return std::nullopt;

  const float medianRho = median(inlierRhos);
  std::vector<float> filteredRhos;
  std::vector<float> filteredPerps;
  filteredRhos.reserve(inlierRhos.size());
  filteredPerps.reserve(inlierPerps.size());
  for (size_t index = 0; index < inlierRhos.size(); ++index) {
    if (std::fabs(inlierRhos[index] - medianRho) > houghLineToleranceMm_) continue;
    filteredRhos.push_back(inlierRhos[index]);
    filteredPerps.push_back(inlierPerps[index]);
  }

  if (static_cast<int>(filteredRhos.size()) < minLineSamples_) return std::nullopt;

  const float extent = percentile(filteredPerps, 0.90f) - percentile(filteredPerps, 0.10f);
  if (extent < minLineExtentMm_) return std::nullopt;

  const float axisComponent = xAxis ? angle.cosValue : angle.sinValue;
  if (std::fabs(axisComponent) < 0.65f) return std::nullopt;

  std::vector<float> orderedRhos = filteredRhos;
  std::sort(orderedRhos.begin(), orderedRhos.end());
  const size_t q1Index = orderedRhos.size() / 4;
  const size_t q3Index = (3 * orderedRhos.size()) / 4;

  AxisCandidate line;
  line.lineOffsetMm = median(filteredRhos) / axisComponent;
  line.lineAngleDeg = angle.angleDeg;
  line.count = static_cast<int>(filteredRhos.size());
  line.houghVotes = voteCount;
  line.spread = orderedRhos[q3Index] - orderedRhos[q1Index];
  line.lineExtentMm = extent;
  line.source = CandidateSource::Hough;
  return line;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::houghAxisLines(
    const std::vector<Ray>& rays, bool xAxis) const {
  if (static_cast<int>(rays.size()) < minLineSamples_) return {};

  struct Peak {
    int angleIndex = 0;
    int bin = 0;
    int votes = 0;
  };

  const auto& angles = xAxis ? houghAnglesX_ : houghAnglesY_;
  std::vector<Peak> peaks;
  for (int angleIndex = 0; angleIndex < static_cast<int>(angles.size()); ++angleIndex) {
    std::unordered_map<int, int> voteBins;
    for (const auto& ray : rays) {
      const float rho = ray.relXMm * angles[angleIndex].cosValue +
                        ray.relYMm * angles[angleIndex].sinValue;
      const int bin = static_cast<int>(std::lround(rho / houghBinMm_));
      ++voteBins[bin];
    }
    for (const auto& [bin, votes] : voteBins) {
      if (votes >= houghMinVotes_) peaks.push_back({angleIndex, bin, votes});
    }
  }

  if (peaks.empty()) return {};

  std::sort(peaks.begin(), peaks.end(), [](const Peak& lhs, const Peak& rhs) {
    return std::tie(lhs.votes, rhs.angleIndex) > std::tie(rhs.votes, lhs.angleIndex);
  });

  std::vector<AxisCandidate> accepted;
  std::vector<Peak> keptPeaks;
  for (const auto& peak : peaks) {
    bool suppressed = false;
    for (const auto& existing : keptPeaks) {
      if (std::abs(peak.bin - existing.bin) <= houghPeakSuppressBins_ &&
          std::abs(peak.angleIndex - existing.angleIndex) <= houghPeakSuppressAngles_) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) continue;

    const float peakRhoMm = static_cast<float>(peak.bin) * houghBinMm_;
    if (auto line = fitHoughLine(rays, xAxis, angles[peak.angleIndex], peakRhoMm, peak.votes)) {
      accepted.push_back(*line);
      keptPeaks.push_back(peak);
      if (accepted.size() >= static_cast<size_t>(houghMaxLinesPerAxis_)) break;
    }
  }

  return accepted;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::candidateHoughAxisClusters(
    const std::vector<Ray>& rays, bool xAxis, std::optional<float> prevValue,
    float axisLimitMm) const {
  auto lines = houghAxisLines(rays, xAxis);
  if (lines.empty()) return {};

  std::vector<AxisCandidate> candidates;
  for (auto line : lines) {
    for (float wallPosition : {0.f, axisLimitMm}) {
      const float poseValue = wallPosition - line.lineOffsetMm;
      if (poseValue < 0.f || poseValue > axisLimitMm) continue;
      line.median = poseValue;
      line.wallPositionMm = wallPosition;
      line.hasWallPosition = true;
      candidates.push_back(line);
    }
  }

  const float center = axisLimitMm * 0.5f;
  std::sort(candidates.begin(), candidates.end(),
            [&](const AxisCandidate& lhs, const AxisCandidate& rhs) {
              const float lhsContinuity =
                  prevValue ? std::fabs(lhs.median - *prevValue) : std::fabs(lhs.median - center);
              const float rhsContinuity =
                  prevValue ? std::fabs(rhs.median - *prevValue) : std::fabs(rhs.median - center);
              if (lhs.houghVotes != rhs.houghVotes) return lhs.houghVotes > rhs.houghVotes;
              if (lhs.count != rhs.count) return lhs.count > rhs.count;
              if (lhs.lineExtentMm != rhs.lineExtentMm) return lhs.lineExtentMm > rhs.lineExtentMm;
              if (lhs.spread != rhs.spread) return lhs.spread < rhs.spread;
              return lhsContinuity < rhsContinuity;
            });

  if (candidates.size() > static_cast<size_t>(maxCandidateClusters_)) {
    candidates.resize(static_cast<size_t>(maxCandidateClusters_));
  }
  return candidates;
}

std::vector<LidarLocalizer::AxisCandidate> LidarLocalizer::mergeAxisCandidates(
    const std::vector<AxisCandidate>& lineClusters,
    const std::vector<AxisCandidate>& pointClusters, std::optional<float> prevValue,
    float axisLimitMm) const {
  std::vector<AxisCandidate> candidates = lineClusters;
  for (auto pointCluster : pointClusters) {
    bool matched = false;
    for (const auto& lineCluster : lineClusters) {
      if (std::fabs(pointCluster.median - lineCluster.median) <= (clusterMatchRadiusMm_ * 0.5f)) {
        matched = true;
        break;
      }
    }
    if (matched) continue;
    pointCluster.source = CandidateSource::Points;
    pointCluster.lineExtentMm = 0.f;
    pointCluster.lineOffsetMm = 0.f;
    pointCluster.hasWallPosition = false;
    candidates.push_back(pointCluster);
  }

  if (candidates.empty()) return {};

  const float center = axisLimitMm * 0.5f;
  std::sort(candidates.begin(), candidates.end(),
            [&](const AxisCandidate& lhs, const AxisCandidate& rhs) {
              const float lhsContinuity =
                  prevValue ? std::fabs(lhs.median - *prevValue) : std::fabs(lhs.median - center);
              const float rhsContinuity =
                  prevValue ? std::fabs(rhs.median - *prevValue) : std::fabs(rhs.median - center);
              const int lhsSourceBonus = candidateSourceBonus(lhs.source);
              const int rhsSourceBonus = candidateSourceBonus(rhs.source);
              if (lhsSourceBonus != rhsSourceBonus) return lhsSourceBonus > rhsSourceBonus;
              if (lhs.lineExtentMm != rhs.lineExtentMm) return lhs.lineExtentMm > rhs.lineExtentMm;
              if (lhs.count != rhs.count) return lhs.count > rhs.count;
              if (lhs.spread != rhs.spread) return lhs.spread < rhs.spread;
              return lhsContinuity < rhsContinuity;
            });

  if (candidates.size() > static_cast<size_t>(maxCandidateClusters_)) {
    candidates.resize(static_cast<size_t>(maxCandidateClusters_));
  }
  return candidates;
}

std::optional<LidarLocalizer::PoseCandidate> LidarLocalizer::choosePoseFromCandidates(
    const std::vector<AxisCandidate>& xClusters, const std::vector<AxisCandidate>& yClusters,
    const std::vector<Ray>& rays, std::optional<float> prevX, std::optional<float> prevY) const {
  if (xClusters.empty() || yClusters.empty() || rays.empty()) return std::nullopt;

  std::optional<PoseCandidate> bestPose;
  std::tuple<int, int, float, float, int, float, float> bestKey;

  for (const auto& xCluster : xClusters) {
    for (const auto& yCluster : yClusters) {
      const float xMm = clamp(xCluster.median, 0.f, fieldW_);
      const float yMm = clamp(yCluster.median, 0.f, fieldH_);
      const auto score = scorePoseAgainstWalls(xMm, yMm, rays);
      if (score.inliers < minPoseInliers_) continue;
      if (score.xInliers < minPoseAxisInliers_ || score.yInliers < minPoseAxisInliers_) continue;

      const float continuity =
          (prevX && prevY)
              ? std::hypot(xMm - *prevX, yMm - *prevY)
              : std::hypot(xMm - fieldW_ * 0.5f, yMm - fieldH_ * 0.5f);
      const auto key =
          std::make_tuple(score.inliers, std::min(score.xInliers, score.yInliers), -continuity,
                          xCluster.lineExtentMm + yCluster.lineExtentMm,
                          xCluster.count + yCluster.count, -score.medianResidualMm,
                          -score.meanResidualMm);
      if (!bestPose || key > bestKey) {
        bestKey = key;
        bestPose = PoseCandidate{xMm, yMm, xCluster, yCluster, score};
      }
    }
  }

  return bestPose;
}

LidarLocalizer::PoseScore LidarLocalizer::scorePoseAgainstWalls(
    float robotXMm, float robotYMm, const std::vector<Ray>& rays) const {
  std::vector<float> residuals;
  std::vector<float> xResiduals;
  std::vector<float> yResiduals;
  residuals.reserve(rays.size() * 2);
  xResiduals.reserve(rays.size());
  yResiduals.reserve(rays.size());

  for (const auto& ray : rays) {
    const float hitX = robotXMm + ray.relXMm;
    const float hitY = robotYMm + ray.relYMm;

    if (hitY >= -wallAxisMarginMm_ && hitY <= fieldH_ + wallAxisMarginMm_) {
      const float xResidual = std::min(std::fabs(hitX), std::fabs(fieldW_ - hitX));
      if (xResidual <= wallInlierMm_) {
        xResiduals.push_back(xResidual);
        residuals.push_back(xResidual);
      }
    }

    if (hitX >= -wallAxisMarginMm_ && hitX <= fieldW_ + wallAxisMarginMm_) {
      const float yResidual = std::min(std::fabs(hitY), std::fabs(fieldH_ - hitY));
      if (yResidual <= wallInlierMm_) {
        yResiduals.push_back(yResidual);
        residuals.push_back(yResidual);
      }
    }
  }

  PoseScore score;
  score.inliers = static_cast<int>(residuals.size());
  score.xInliers = static_cast<int>(xResiduals.size());
  score.yInliers = static_cast<int>(yResiduals.size());
  if (!residuals.empty()) {
    score.medianResidualMm = median(residuals);
    score.meanResidualMm =
        std::accumulate(residuals.begin(), residuals.end(), 0.f) / residuals.size();
  } else {
    score.medianResidualMm = std::numeric_limits<float>::infinity();
    score.meanResidualMm = std::numeric_limits<float>::infinity();
  }
  return score;
}

int LidarLocalizer::candidateSourceBonus(CandidateSource source) const {
  switch (source) {
    case CandidateSource::Hough:
      return 2;
    case CandidateSource::Line:
      return 1;
    case CandidateSource::Points:
    default:
      return 0;
  }
}

bool LidarLocalizer::shouldHoldPose(float innovationMm, const PoseScore& score) const {
  if (innovationMm <= poseJumpRejectMm_) return false;
  if (score.medianResidualMm <= goodResidualMm_ && prevPoseQuality_) {
    return score.inliers < (prevPoseQuality_->inliers + 12);
  }
  return true;
}

float LidarLocalizer::poseAlpha(const PoseScore& score) const {
  return (score.medianResidualMm <= goodResidualMm_) ? poseAlphaFast_ : poseAlphaSlow_;
}

PoseEstimate LidarLocalizer::update(const std::vector<LidarPoint>& points, float headingDeg) {
  const auto rays = fieldRays(points, headingDeg);

  std::vector<float> xEstimates;
  std::vector<float> yEstimates;
  xEstimates.reserve(rays.size());
  yEstimates.reserve(rays.size());

  for (const auto& ray : rays) {
    if (std::fabs(ray.dx) >= minWallComponent_) {
      const float xEstimate = wallAxisEstimate(ray.distanceMm, ray.dx, fieldW_);
      if (xEstimate > -200.f && xEstimate < fieldW_ + 200.f) xEstimates.push_back(xEstimate);
    }
    if (std::fabs(ray.dy) >= minWallComponent_) {
      const float yEstimate = wallAxisEstimate(ray.distanceMm, ray.dy, fieldH_);
      if (yEstimate > -200.f && yEstimate < fieldH_ + 200.f) yEstimates.push_back(yEstimate);
    }
  }

  std::optional<float> prevX;
  std::optional<float> prevY;
  if (prevPoseMm_) {
    prevX = prevPoseMm_->first;
    prevY = prevPoseMm_->second;
  }

  const auto xHough = candidateHoughAxisClusters(rays, true, prevX, fieldW_);
  const auto yHough = candidateHoughAxisClusters(rays, false, prevY, fieldH_);
  const auto xLine = candidateLineAxisClusters(rays, true, prevX, fieldW_);
  const auto yLine = candidateLineAxisClusters(rays, false, prevY, fieldH_);
  const auto xPoint = candidateAxisClusters(xEstimates, prevX, fieldW_);
  const auto yPoint = candidateAxisClusters(yEstimates, prevY, fieldH_);
  const auto xCandidates = mergeAxisCandidates(
      [&]() {
        std::vector<AxisCandidate> merged = xHough;
        merged.insert(merged.end(), xLine.begin(), xLine.end());
        return merged;
      }(),
      xPoint, prevX, fieldW_);
  const auto yCandidates = mergeAxisCandidates(
      [&]() {
        std::vector<AxisCandidate> merged = yHough;
        merged.insert(merged.end(), yLine.begin(), yLine.end());
        return merged;
      }(),
      yPoint, prevY, fieldH_);

  PoseEstimate estimate;
  estimate.xSampleCount = static_cast<uint16_t>(xEstimates.size());
  estimate.ySampleCount = static_cast<uint16_t>(yEstimates.size());

  const auto pose = choosePoseFromCandidates(xCandidates, yCandidates, rays, prevX, prevY);
  if (!pose) {
    if (!xCandidates.empty()) estimate.rawXMm = clamp(xCandidates.front().median, 0.f, fieldW_);
    if (!yCandidates.empty()) estimate.rawYMm = clamp(yCandidates.front().median, 0.f, fieldH_);
    if (!prevPoseMm_) return estimate;

    estimate.valid = true;
    estimate.held = true;
    estimate.xMm = prevPoseMm_->first;
    estimate.yMm = prevPoseMm_->second;
    return estimate;
  }

  const float rawX = clamp(pose->xMm, 0.f, fieldW_);
  const float rawY = clamp(pose->yMm, 0.f, fieldH_);
  estimate.rawXMm = rawX;
  estimate.rawYMm = rawY;
  estimate.inliers = pose->score.inliers;
  estimate.xInliers = pose->score.xInliers;
  estimate.yInliers = pose->score.yInliers;
  estimate.medianResidualMm = pose->score.medianResidualMm;
  estimate.meanResidualMm = pose->score.meanResidualMm;

  if (!prevPoseMm_) {
    estimate.valid = true;
    estimate.shouldFuse = true;
    estimate.poseAlpha = 1.f;
    estimate.xMm = rawX;
    estimate.yMm = rawY;
    prevPoseMm_ = std::make_pair(estimate.xMm, estimate.yMm);
    prevPoseQuality_ = PoseQuality{pose->score.inliers, pose->score.medianResidualMm};
    return estimate;
  }

  const float innovationMm = std::hypot(rawX - prevPoseMm_->first, rawY - prevPoseMm_->second);
  if (shouldHoldPose(innovationMm, pose->score)) {
    estimate.valid = true;
    estimate.held = true;
    estimate.xMm = prevPoseMm_->first;
    estimate.yMm = prevPoseMm_->second;
    prevPoseMm_ = std::make_pair(estimate.xMm, estimate.yMm);
    prevPoseQuality_ = PoseQuality{pose->score.inliers, pose->score.medianResidualMm};
    return estimate;
  }

  const float alpha = poseAlpha(pose->score);
  float nextX = prevPoseMm_->first + (rawX - prevPoseMm_->first) * alpha;
  float nextY = prevPoseMm_->second + (rawY - prevPoseMm_->second) * alpha;

  const float stepX = nextX - prevPoseMm_->first;
  const float stepY = nextY - prevPoseMm_->second;
  const float stepMag = std::hypot(stepX, stepY);
  if (stepMag > maxPoseStepMm_) {
    const float scale = maxPoseStepMm_ / stepMag;
    nextX = prevPoseMm_->first + stepX * scale;
    nextY = prevPoseMm_->second + stepY * scale;
  }

  estimate.valid = true;
  estimate.shouldFuse = true;
  estimate.poseAlpha = alpha;
  estimate.xMm = clamp(nextX, 0.f, fieldW_);
  estimate.yMm = clamp(nextY, 0.f, fieldH_);
  prevPoseMm_ = std::make_pair(estimate.xMm, estimate.yMm);
  prevPoseQuality_ = PoseQuality{pose->score.inliers, pose->score.medianResidualMm};
  return estimate;
}

}  // namespace ballalgo
