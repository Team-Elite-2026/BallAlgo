#pragma once

#include "lidar/Ld19Reader.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace ballalgo {

struct PoseEstimate {
  float xMm = 0;
  float yMm = 0;
  bool valid = false;
  uint16_t xSampleCount = 0;
  uint16_t ySampleCount = 0;
};

class LidarLocalizer {
 public:
  LidarLocalizer(float fieldW, float fieldH, float yawOffsetDeg);
  PoseEstimate update(const std::vector<LidarPoint>& points, float headingDeg);

 private:
  struct Ray {
    uint16_t angleCd = 0;
    float distanceMm = 0;
    float dx = 0;
    float dy = 0;
    float relXMm = 0;
    float relYMm = 0;
  };

  enum class CandidateSource { Points, Line, Hough };

  struct AxisCandidate {
    float median = 0;
    int count = 0;
    float spread = 0;
    float lineExtentMm = 0;
    float lineOffsetMm = 0;
    float wallPositionMm = 0;
    float lineAngleDeg = 0;
    int houghVotes = 0;
    CandidateSource source = CandidateSource::Points;
    bool hasWallPosition = false;
  };

  struct PoseScore {
    int inliers = 0;
    int xInliers = 0;
    int yInliers = 0;
    float medianResidualMm = 0;
    float meanResidualMm = 0;
  };

  struct PoseCandidate {
    float xMm = 0;
    float yMm = 0;
    AxisCandidate xCluster;
    AxisCandidate yCluster;
    PoseScore score;
  };

  struct HoughAngle {
    float angleDeg = 0;
    float cosValue = 0;
    float sinValue = 0;
  };

  static float clamp(float value, float lower, float upper);
  static float median(std::vector<float> values);
  static float percentile(std::vector<float> values, float fraction);
  static float wallAxisEstimate(float distanceMm, float directionComponent, float axisLimitMm);
  static float rayEndpointGap(const Ray& a, const Ray& b);

  std::vector<HoughAngle> buildHoughAngles(float centerAngleDeg) const;
  std::vector<Ray> fieldRays(const std::vector<LidarPoint>& points, float headingDeg) const;
  std::vector<Ray> filterIsolatedRays(std::vector<Ray> rays) const;
  std::vector<AxisCandidate> clusterEstimates(const std::vector<float>& estimates) const;
  std::vector<AxisCandidate> candidateAxisClusters(const std::vector<float>& estimates,
                                                   std::optional<float> prevValue,
                                                   float axisLimitMm) const;
  std::vector<AxisCandidate> candidateLineAxisClusters(const std::vector<Ray>& rays, bool xAxis,
                                                       std::optional<float> prevValue,
                                                       float axisLimitMm) const;
  std::optional<AxisCandidate> lineFeatureFromRays(const std::vector<Ray>& rays, bool xAxis) const;
  std::vector<AxisCandidate> candidateHoughAxisClusters(const std::vector<Ray>& rays, bool xAxis,
                                                        std::optional<float> prevValue,
                                                        float axisLimitMm) const;
  std::vector<AxisCandidate> houghAxisLines(const std::vector<Ray>& rays, bool xAxis) const;
  std::optional<AxisCandidate> fitHoughLine(const std::vector<Ray>& rays, bool xAxis,
                                            const HoughAngle& angle, float peakRhoMm,
                                            int voteCount) const;
  std::vector<AxisCandidate> mergeAxisCandidates(const std::vector<AxisCandidate>& lineClusters,
                                                 const std::vector<AxisCandidate>& pointClusters,
                                                 std::optional<float> prevValue,
                                                 float axisLimitMm) const;
  std::optional<PoseCandidate> choosePoseFromCandidates(
      const std::vector<AxisCandidate>& xClusters, const std::vector<AxisCandidate>& yClusters,
      const std::vector<Ray>& rays, std::optional<float> prevX, std::optional<float> prevY) const;
  PoseScore scorePoseAgainstWalls(float robotXMm, float robotYMm,
                                  const std::vector<Ray>& rays) const;
  int candidateSourceBonus(CandidateSource source) const;

  float fieldW_;
  float fieldH_;
  float yawOffset_;
  float minDist_ = 80.f;
  float maxDist_ = 6000.f;
  int minIntensity_ = 20;
  int minAxisSamples_ = 12;
  float clusterGapMm_ = 140.f;
  float clusterMatchRadiusMm_ = 260.f;
  int minLineSamples_ = 10;
  float lineClusterGapMm_ = 90.f;
  float minLineExtentMm_ = 260.f;
  float houghAngleStepDeg_ = 2.f;
  float houghTiltDeg_ = 14.f;
  float houghBinMm_ = 24.f;
  float houghLineToleranceMm_ = 85.f;
  int houghMinVotes_ = 9;
  int houghPeakSuppressBins_ = 5;
  int houghPeakSuppressAngles_ = 2;
  int houghMaxLinesPerAxis_ = 4;
  float minWallComponent_ = 0.35f;
  int maxCandidateClusters_ = 4;
  float neighborGapMm_ = 260.f;
  float wallInlierMm_ = 90.f;
  float wallAxisMarginMm_ = 180.f;
  int minPoseInliers_ = 28;
  int minPoseAxisInliers_ = 8;
  std::vector<HoughAngle> houghAnglesX_;
  std::vector<HoughAngle> houghAnglesY_;
  std::optional<std::pair<float, float>> prevPoseMm_;
};

}  // namespace ballalgo
