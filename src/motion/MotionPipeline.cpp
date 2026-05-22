#include "motion/MotionPipeline.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"
#include "vision/VisionMath.hpp"

#include <chrono>

namespace ballalgo {

MotionPipeline::MotionPipeline()
    : localizer_(config::kFieldWidthMm, config::kFieldHeightMm, config::kLidarYawOffsetDeg) {}

PoseState MotionPipeline::updateLidar(const std::vector<LidarPoint>& pts, float headingDeg,
                                      double dtS) {
  poseKf_.predict(dtS);
  auto est = localizer_.update(pts, headingDeg);
  poseKf_.update(est, headingDeg);
  return poseKf_.state(headingDeg);
}

BallState MotionPipeline::updateBall(double angleDeg, double distCal, bool found, double dtS) {
  ballKf_.predict(dtS);
  if (found && distCal >= 0) {
    float x, y;
    polarToBodyXY(angleDeg, static_cast<float>(distCal * config::kBallDistToM), x, y);
    ballKf_.update(x, y, true);
  } else {
    ballKf_.update(0, 0, false);
  }
  return ballKf_.state();
}

bool MotionPipeline::tickPublish(RobotSerial& serial, std::vector<uint8_t>& rx,
                                const PoseState& pose, const BallState& ball, float goalDeg,
                                float headingDeg, bool offenseActive) {
  if (!config::kMotionV2 || !offenseActive) return true;
  serial.readSome(rx);
  clock_.processBuffer(serial, rx);
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration<double>(now.time_since_epoch()).count();
  if (t - lastPublish_ < 1.0 / config::kChunkPublishHz) return true;
  lastPublish_ = t;
  bool full = pose.valid;
  auto chunk = planner_.plan(pose, ball, goalDeg, headingDeg, clock_.latencyUs(), full);
  auto pkt = packActionChunk(chunk.trajectoryId, chunk.startTimePi, chunk.dtMs, chunk.actions,
                             static_cast<int>(chunk.actions.size()), pose.vxBody, pose.vyBody,
                             pose.valid);
  return serial.write(pkt);
}

}  // namespace ballalgo
