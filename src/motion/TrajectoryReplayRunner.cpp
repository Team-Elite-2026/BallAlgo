#include "motion/TrajectoryReplayRunner.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"

#include <algorithm>

namespace ballalgo {

namespace {

constexpr uint64_t kReplayInitialLeadUs = 50'000;
constexpr uint64_t kReplayQueueLeadUs = 40'000;

}  // namespace

bool TrajectoryReplayRunner::loadArtifact(const std::string& path, std::string* error) {
  TrajectoryReplayArtifact loadedArtifact;
  if (!loadTrajectoryReplayArtifact(path, loadedArtifact, error)) return false;

  artifact_ = std::move(loadedArtifact);
  scheduledChunks_.clear();
  scheduledChunks_.reserve(artifact_.chunks.size());
  for (const auto& chunk : artifact_.chunks) {
    scheduledChunks_.push_back(ScheduledChunk{chunk, 0, false});
  }
  loaded_ = true;
  haveClockSyncPing_ = false;
  armed_ = false;
  replayEpochPiUs_ = 0;
  nextChunkToSend_ = 0;
  buildPlannerSnapshot();
  return true;
}

void TrajectoryReplayRunner::buildPlannerSnapshot() {
  plannerSnapshot_ = {};
  if (artifact_.chunks.empty() || artifact_.path.empty()) return;

  plannerSnapshot_.valid = true;
  plannerSnapshot_.commandedGoalMode = artifact_.hasGoalPose;
  plannerSnapshot_.trajectoryId = artifact_.chunks.front().trajectoryId;
  plannerSnapshot_.dtMs = artifact_.chunks.front().dtMs;
  plannerSnapshot_.startTimePi = 0;
  if (artifact_.hasGoalPose) {
    plannerSnapshot_.targetXMm = artifact_.goalPose.xMm;
    plannerSnapshot_.targetYMm = artifact_.goalPose.yMm;
    plannerSnapshot_.targetHeadingDeg = artifact_.goalPose.headingDeg;
  } else if (!artifact_.path.empty()) {
    const auto& tail = artifact_.path.back();
    plannerSnapshot_.targetXMm = tail.xMm;
    plannerSnapshot_.targetYMm = tail.yMm;
    plannerSnapshot_.targetHeadingDeg = tail.thetaDeg;
  }
  plannerSnapshot_.path = artifact_.path;
  plannerSnapshot_.trajectorySpeedProfile = artifact_.trajectorySpeedProfile;
}

void TrajectoryReplayRunner::armReplay(uint64_t nowPiUs) {
  replayEpochPiUs_ = nowPiUs + kReplayInitialLeadUs;
  for (auto& chunk : scheduledChunks_) {
    chunk.startTimePiUs = replayEpochPiUs_ + static_cast<uint64_t>(chunk.chunk.startDelayMs) * 1000u;
    chunk.sent = false;
  }
  nextChunkToSend_ = 0;
  armed_ = true;
  if (!scheduledChunks_.empty()) {
    plannerSnapshot_.trajectoryId = scheduledChunks_.front().chunk.trajectoryId;
    plannerSnapshot_.dtMs = scheduledChunks_.front().chunk.dtMs;
    plannerSnapshot_.startTimePi = scheduledChunks_.front().startTimePiUs;
  }
}

bool TrajectoryReplayRunner::updateAndPublish(RobotSerial& serial, uint64_t nowPiUs) {
  if (!loaded_) return false;

  std::vector<ProtocolFrame> frames;
  serial.takePendingFrames(frames);
  for (const auto& frame : frames) {
    if (frame.type == kMsgPing && frame.payload.size() >= 8) {
      haveClockSyncPing_ = true;
    }
  }
  clock_.processFrames(serial, frames);

  if (!haveClockSyncPing_ || !serial.isOpen()) return false;
  if (!armed_) armReplay(nowPiUs);
  if (nextChunkToSend_ >= scheduledChunks_.size()) return false;

  auto& nextChunk = scheduledChunks_[nextChunkToSend_];
  if (nextChunk.sent || nowPiUs + kReplayQueueLeadUs < nextChunk.startTimePiUs) return false;

  auto frame = packActionChunk(nextChunk.chunk.trajectoryId, nextChunk.startTimePiUs,
                               nextChunk.chunk.dtMs, nextChunk.chunk.actions,
                               static_cast<int>(nextChunk.chunk.actions.size()),
                               artifact_.startPose.vxBody, artifact_.startPose.vyBody,
                               artifact_.startPose.valid);
  if (!serial.write(frame)) return false;

  nextChunk.sent = true;
  ++nextChunkToSend_;
  plannerSnapshot_.trajectoryId = nextChunk.chunk.trajectoryId;
  plannerSnapshot_.dtMs = nextChunk.chunk.dtMs;
  plannerSnapshot_.startTimePi = nextChunk.startTimePiUs;
  return true;
}

TrajectoryTargetSample TrajectoryReplayRunner::currentTarget(uint64_t queryPiUs,
                                                             float headingDeg) const {
  if (!armed_ || scheduledChunks_.empty()) return {};

  const ScheduledChunk* activeChunk = nullptr;
  for (const auto& chunk : scheduledChunks_) {
    if (!chunk.sent || queryPiUs < chunk.startTimePiUs) break;
    activeChunk = &chunk;
  }
  if (activeChunk == nullptr) return {};
  return sampleTrajectoryTarget(activeChunk->chunk, activeChunk->startTimePiUs, queryPiUs,
                                headingDeg);
}

}  // namespace ballalgo
