#pragma once

#include "vision/Thresholds.hpp"
#include "vision/VisionMath.hpp"

#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace ballalgo {

struct BallTrackResult {
  bool found = false;
  double angleDeg = -5;
  double distPx = -5;
  double distCal = -5;
  int ballVxPx = 0;
  int ballVyPx = 0;
};

struct GoalTrackResult {
  double blueAngle = -5;
  double yellowAngle = -5;
  // Visibility certainty c in [0,1] derived from the detected goal blob area.
  // Feeds the Step 1b bearing-EKF measurement variance R = base + pen*(1-c)^2.
  double blueCertainty = 0;
  double yellowCertainty = 0;
};

class SectorTracker {
 public:
  void buildRoi(const cv::Size& frameSize, const ThresholdsData& thr);
  BallTrackResult trackBall(const cv::Mat& binBall, int xoff, int yoff);
  GoalTrackResult trackGoals(const cv::Mat& binYellow, const cv::Mat& binBlue, int xoff, int yoff);

 private:
  std::optional<cv::Point> findLargestContour(const cv::Mat& bin, int minArea) const;
  std::optional<cv::Point> findLargestContour(const cv::Mat& bin, int minArea,
                                              double& areaOut) const;
  int sectorFromAngle(double angleDeg) const;
  std::vector<int> ringSectors(int seed, int radius) const;

  cv::Point sectorCenter_{0, 0};
  std::vector<cv::Mat> sectorMasks_;
  cv::Mat ellipseMask_;
  bool firstFrame_ = false;
  std::optional<cv::Point> lastPos_;
  std::optional<int> lastSector_;
  double velEmaX_ = 0, velEmaY_ = 0;
};

}  // namespace ballalgo
