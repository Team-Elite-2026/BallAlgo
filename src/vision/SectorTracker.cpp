#include "vision/SectorTracker.hpp"

#include "config.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>

namespace ballalgo {

void SectorTracker::buildRoi(const cv::Size& frameSize, const ThresholdsData& thr) {
  sectorMasks_.clear();
  ellipseMask_ = cv::Mat();
  if (thr.hasMask) {
    sectorCenter_ = thr.maskCenter;
    ellipseMask_ = cv::Mat::zeros(frameSize, CV_8UC1);
    cv::ellipse(ellipseMask_, sectorCenter_, thr.maskAxes, 0, 0, 360, 255, -1);
  } else {
    sectorCenter_ = cv::Point(frameSize.width / 2, frameSize.height / 2);
  }
  const int radius = static_cast<int>(std::hypot(frameSize.width, frameSize.height));
  for (int s = 0; s < config::kNumSectors; ++s) {
    const int startDeg = s * config::kSectorAngleDeg;
    const int endDeg = (s + 1) * config::kSectorAngleDeg;
    std::vector<cv::Point> pts;
    pts.push_back(sectorCenter_);
    for (int a = startDeg; a <= endDeg; a += 2) {
      const double rad = a * M_PI / 180.0;
      pts.emplace_back(sectorCenter_.x + static_cast<int>(radius * std::cos(rad)),
                       sectorCenter_.y + static_cast<int>(radius * std::sin(rad)));
    }
    pts.push_back(sectorCenter_);
    cv::Mat wedge = cv::Mat::zeros(frameSize, CV_8UC1);
    const std::vector<std::vector<cv::Point>> poly{pts};
    cv::fillPoly(wedge, poly, 255);
    if (!ellipseMask_.empty()) cv::bitwise_and(wedge, ellipseMask_, wedge);
    sectorMasks_.push_back(wedge);
  }
  firstFrame_ = false;
  lastPos_.reset();
  lastSector_.reset();
  velEmaX_ = 0;
  velEmaY_ = 0;
}

std::optional<cv::Point> SectorTracker::findLargestContour(const cv::Mat& bin, int minArea) const {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  double best = 0;
  std::optional<cv::Point> center;
  for (const auto& c : contours) {
    double a = cv::contourArea(c);
    if (a >= minArea && a > best) {
      best = a;
      cv::Moments m = cv::moments(c);
      if (m.m00 > 0) center = cv::Point(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
    }
  }
  return center;
}

int SectorTracker::sectorFromAngle(double angleDeg) const {
  int a = static_cast<int>(angleDeg) % 360;
  if (a < 0) a += 360;
  return (a / config::kSectorAngleDeg) % config::kNumSectors;
}

std::vector<int> SectorTracker::ringSectors(int seed, int radius) const {
  if (radius == 0) return {seed};
  return {(seed - radius + config::kNumSectors) % config::kNumSectors,
          (seed + radius) % config::kNumSectors};
}

BallTrackResult SectorTracker::trackBall(const cv::Mat& binBall, int xoff, int yoff) {
  BallTrackResult r;
  std::optional<cv::Point> best;
  int foundSector = 0;

  if (!firstFrame_) {
    best = findLargestContour(binBall, config::kMinAreaPix);
    firstFrame_ = true;
  } else {
    int predicted = lastSector_.value_or(0);
    if (lastPos_) {
      const double speed = std::hypot(velEmaX_, velEmaY_);
      int lookahead = config::kLookaheadMin + (speed > config::kLookaheadSpeedThresh ? 1 : 0);
      if (lookahead > config::kLookaheadMax) lookahead = config::kLookaheadMax;
      const double px = lastPos_->x + velEmaX_ * lookahead;
      const double py = lastPos_->y + velEmaY_ * lookahead;
      const double ang = std::atan2(py - sectorCenter_.y, px - sectorCenter_.x) * 180.0 / M_PI + 360.0;
      predicted = sectorFromAngle(ang);
    }
    const int maxR = config::kNumSectors / 2 + 1;
    bool hit = false;
    for (int rad = 0; rad < maxR && !hit; ++rad) {
      for (int sec : ringSectors(predicted, rad)) {
        if (sec < 0 || sec >= static_cast<int>(sectorMasks_.size())) continue;
        cv::Mat masked;
        cv::bitwise_and(binBall, sectorMasks_[sec], masked);
        auto c = findLargestContour(masked, config::kMinAreaPix);
        if (c) {
          best = c;
          foundSector = sec;
          hit = true;
          if (config::kStopOnFirstHit) break;
        }
      }
    }
    if (!hit) {
      for (int sec = 0; sec < config::kNumSectors; ++sec) {
        cv::Mat masked;
        cv::bitwise_and(binBall, sectorMasks_[sec], masked);
        auto c = findLargestContour(masked, config::kMinAreaPix);
        if (c) {
          best = c;
          foundSector = sec;
          break;
        }
      }
    }
  }

  if (!best) {
    velEmaX_ *= 0.9;
    velEmaY_ *= 0.9;
    return r;
  }

  r.found = true;
  if (lastPos_) {
    double dvx = best->x - lastPos_->x;
    double dvy = best->y - lastPos_->y;
    velEmaX_ = config::kVelAlpha * velEmaX_ + (1 - config::kVelAlpha) * dvx;
    velEmaY_ = config::kVelAlpha * velEmaY_ + (1 - config::kVelAlpha) * dvy;
    const double mag = std::hypot(velEmaX_, velEmaY_);
    if (mag > config::kVelMaxPx) {
      velEmaX_ *= config::kVelMaxPx / mag;
      velEmaY_ *= config::kVelMaxPx / mag;
    }
  }
  lastPos_ = best;
  lastSector_ = foundSector;
  auto ad = angleAndDistance(best->x, best->y, xoff, yoff);
  r.angleDeg = ad.angleDeg;
  r.distPx = ad.distPx;
  r.distCal = calibrateBallDist(ad.distPx);
  r.ballVxPx = static_cast<int>(std::lround(velEmaX_));
  r.ballVyPx = static_cast<int>(std::lround(velEmaY_));
  return r;
}

GoalTrackResult SectorTracker::trackGoals(const cv::Mat& binYellow, const cv::Mat& binBlue, int xoff,
                                         int yoff) {
  GoalTrackResult g;
  if (auto c = findLargestContour(binYellow, config::kMinAreaPix)) {
    g.yellowAngle = angleAndDistance(c->x, c->y, xoff, yoff).angleDeg;
  }
  if (auto c = findLargestContour(binBlue, config::kMinAreaPix)) {
    g.blueAngle = angleAndDistance(c->x, c->y, xoff, yoff).angleDeg;
  }
  return g;
}

}  // namespace ballalgo
