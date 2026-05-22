#pragma once

#include <opencv2/core.hpp>
#include <string>

namespace ballalgo {

struct ThresholdSet {
  cv::Scalar lower;
  cv::Scalar upper;
};

struct ThresholdsData {
  ThresholdSet ball;
  ThresholdSet yellowGoal;
  ThresholdSet blueGoal;
  int xoffset = 0;
  int yoffset = 0;
  bool hasMask = false;
  cv::Point maskCenter;
  cv::Size maskAxes;
};

bool loadThresholds(const std::string& path, ThresholdsData& out);

}  // namespace ballalgo
