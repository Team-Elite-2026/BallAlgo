#include "vision/Thresholds.hpp"

#include "config.hpp"

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

namespace ballalgo {

namespace {

void ignoreValueChannel(cv::Scalar& lower, cv::Scalar& upper) {
  lower[2] = 0;
  upper[2] = 255;
}

bool parseHsvArray(const std::string& json, const std::string& key, cv::Scalar& lower, cv::Scalar& upper) {
  std::regex blockRe("\"" + key + "\"\\s*:\\s*\\{([^}]*)\\}");
  std::smatch m;
  if (!std::regex_search(json, m, blockRe)) return false;
  std::string block = m[1].str();
  auto extract = [&](const std::string& lk) {
    std::regex r("\"" + lk + "\"\\s*:\\s*(\\d+)");
    std::smatch mm;
    return std::regex_search(block, mm, r) ? std::stoi(mm[1].str()) : -1;
  };
  int lh = extract("lower") >= 0 ? extract("H") : extract("h");
  if (lh < 0) {
    lh = extract("H");
    int ls = extract("S"), lv = extract("V");
    int uh = extract("upper") >= 0 ? -1 : -1;
  }
  int h0 = extract("H");
  if (h0 < 0) h0 = extract("h");
  int s0 = extract("S");
  if (s0 < 0) s0 = extract("s");
  int v0 = extract("V");
  if (v0 < 0) v0 = extract("v");
  std::regex lowerRe("\"lower\"\\s*:\\s*\\[\\s*(\\d+)\\s*,\\s*(\\d+)\\s*,\\s*(\\d+)\\s*\\]");
  std::regex upperRe("\"upper\"\\s*:\\s*\\[\\s*(\\d+)\\s*,\\s*(\\d+)\\s*,\\s*(\\d+)\\s*\\]");
  std::smatch lm, um;
  if (std::regex_search(block, lm, lowerRe)) {
    lower = cv::Scalar(std::stoi(lm[1]), std::stoi(lm[2]), std::stoi(lm[3]));
  }
  if (std::regex_search(block, um, upperRe)) {
    upper = cv::Scalar(std::stoi(um[1]), std::stoi(um[2]), std::stoi(um[3]));
  }
  return lower[0] >= 0 && upper[0] >= 0;
}

}  // namespace

bool loadThresholds(const std::string& path, ThresholdsData& out) {
  std::ifstream f(path);
  if (!f) {
    std::cerr << "[THR] Missing " << path << ", using fallback\n";
    out.ball.lower = cv::Scalar(5, 120, 120);
    out.ball.upper = cv::Scalar(25, 255, 255);
    out.yellowGoal = out.blueGoal = out.ball;
    out.xoffset = out.yoffset = 0;
    if (config::kIgnoreHsvValue) {
      ignoreValueChannel(out.ball.lower, out.ball.upper);
      ignoreValueChannel(out.yellowGoal.lower, out.yellowGoal.upper);
      ignoreValueChannel(out.blueGoal.lower, out.blueGoal.upper);
    }
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string json = ss.str();
  if (!parseHsvArray(json, "ball", out.ball.lower, out.ball.upper)) return false;
  parseHsvArray(json, "yellowGoal", out.yellowGoal.lower, out.yellowGoal.upper);
  parseHsvArray(json, "blueGoal", out.blueGoal.lower, out.blueGoal.upper);
  std::regex offRe("\"offsets\"\\s*:\\s*\\{[^}]*\"x\"\\s*:\\s*(-?\\d+)[^}]*\"y\"\\s*:\\s*(-?\\d+)");
  std::smatch om;
  if (std::regex_search(json, om, offRe)) {
    out.xoffset = std::stoi(om[1]);
    out.yoffset = std::stoi(om[2]);
  }
  std::regex maskRe("\"mask1\"\\s*:\\s*\\{[^}]*\"x\"\\s*:\\s*(\\d+)[^}]*\"y\"\\s*:\\s*(\\d+)[^}]*\"size1\"\\s*:\\s*(\\d+)[^}]*\"size2\"\\s*:\\s*(\\d+)");
  std::smatch mm;
  if (std::regex_search(json, mm, maskRe)) {
    out.hasMask = true;
    out.maskCenter = cv::Point(std::stoi(mm[1]), std::stoi(mm[2]));
    out.maskAxes = cv::Size(std::stoi(mm[3]), std::stoi(mm[4]));
  }
  if (config::kIgnoreHsvValue) {
    ignoreValueChannel(out.ball.lower, out.ball.upper);
    ignoreValueChannel(out.yellowGoal.lower, out.yellowGoal.upper);
    ignoreValueChannel(out.blueGoal.lower, out.blueGoal.upper);
  }
  return true;
}

}  // namespace ballalgo
