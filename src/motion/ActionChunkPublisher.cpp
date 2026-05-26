#include "motion/ActionChunkPublisher.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"

#include <chrono>

namespace ballalgo {

bool ActionChunkPublisher::publish(RobotSerial& serial, std::vector<uint8_t>& rx,
                                   const PoseState& pose, const BallState& ball, float goalDeg,
                                   float headingDeg, bool offenseActive) {
  if (!config::kEnableActionChunks || !offenseActive) return true;
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
