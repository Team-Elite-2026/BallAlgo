#include "motion/TrajectoryReplayRunner.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"

#include <algorithm>
#include <cstdio>

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
  lastDebugReportPiUs_ = 0;
  lastSerialOpen_ = false;
  lastHaveClockSyncPing_ = false;
  buildPlannerSnapshot();
  std::fprintf(stderr,
               "[REPLAY] loaded case=%s chunks=%zu path_samples=%zu profile_samples=%zu\n",
               artifact_.caseName.c_str(), artifact_.chunks.size(), artifact_.path.size(),
               artifact_.trajectorySpeedProfile.size());
  std::fflush(stderr);
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
  std::fprintf(stderr,
               "[REPLAY] armed epoch_pi_us=%llu next_chunk_start=%llu total_chunks=%zu\n",
               static_cast<unsigned long long>(replayEpochPiUs_),
               scheduledChunks_.empty()
                   ? 0ull
                   : static_cast<unsigned long long>(scheduledChunks_.front().startTimePiUs),
               scheduledChunks_.size());
  std::fflush(stderr);
}

bool TrajectoryReplayRunner::updateAndPublish(RobotSerial& serial, uint64_t nowPiUs) {
  if (!loaded_) return false;

  const bool serialOpen = serial.isOpen();
  if (serialOpen != lastSerialOpen_) {
    std::fprintf(stderr, "[REPLAY] serial_open=%s\n", serialOpen ? "yes" : "no");
    std::fflush(stderr);
    lastSerialOpen_ = serialOpen;
  }

  std::vector<ProtocolFrame> frames;
  serial.takePendingFrames(frames);
  for (const auto& frame : frames) {
    if (frame.type == kMsgPing && frame.payload.size() >= 8) {
      haveClockSyncPing_ = true;
    }
  }
  clock_.processFrames(serial, frames);

  if (haveClockSyncPing_ != lastHaveClockSyncPing_) {
    std::fprintf(stderr, "[REPLAY] clock_sync_ping_seen=%s\n",
                 haveClockSyncPing_ ? "yes" : "no");
    std::fflush(stderr);
    lastHaveClockSyncPing_ = haveClockSyncPing_;
  }

  if (!haveClockSyncPing_ || !serialOpen) {
    if (lastDebugReportPiUs_ == 0 || nowPiUs - lastDebugReportPiUs_ >= 1'000'000u) {
      std::fprintf(stderr,
                   "[REPLAY] waiting serial_open=%s have_ping=%s armed=%s next_chunk=%zu/%zu\n",
                   serialOpen ? "yes" : "no", haveClockSyncPing_ ? "yes" : "no",
                   armed_ ? "yes" : "no", nextChunkToSend_, scheduledChunks_.size());
      std::fflush(stderr);
      lastDebugReportPiUs_ = nowPiUs;
    }
    return false;
  }

  if (!armed_) armReplay(nowPiUs);
  if (nextChunkToSend_ >= scheduledChunks_.size()) {
    if (lastDebugReportPiUs_ == 0 || nowPiUs - lastDebugReportPiUs_ >= 1'000'000u) {
      std::fprintf(stderr, "[REPLAY] all chunks sent total=%zu\n", scheduledChunks_.size());
      std::fflush(stderr);
      lastDebugReportPiUs_ = nowPiUs;
    }
    return false;
  }

  auto& nextChunk = scheduledChunks_[nextChunkToSend_];
  if (nextChunk.sent || nowPiUs + kReplayQueueLeadUs < nextChunk.startTimePiUs) {
    if (lastDebugReportPiUs_ == 0 || nowPiUs - lastDebugReportPiUs_ >= 1'000'000u) {
      const int64_t waitUs = static_cast<int64_t>(nextChunk.startTimePiUs) -
                             static_cast<int64_t>(nowPiUs + kReplayQueueLeadUs);
      std::fprintf(stderr,
                   "[REPLAY] queued next_chunk=%zu traj=%llu wait_us=%lld start_pi_us=%llu now_pi_us=%llu\n",
                   nextChunkToSend_,
                   static_cast<unsigned long long>(nextChunk.chunk.trajectoryId),
                   static_cast<long long>(waitUs),
                   static_cast<unsigned long long>(nextChunk.startTimePiUs),
                   static_cast<unsigned long long>(nowPiUs));
      std::fflush(stderr);
      lastDebugReportPiUs_ = nowPiUs;
    }
    return false;
  }

  auto frame = packActionChunk(nextChunk.chunk.trajectoryId, nextChunk.startTimePiUs,
                               nextChunk.chunk.dtMs, nextChunk.chunk.actions,
                               static_cast<int>(nextChunk.chunk.actions.size()),
                               artifact_.startPose.vxBody, artifact_.startPose.vyBody,
                               artifact_.startPose.valid);
  if (!serial.write(frame)) {
    std::fprintf(stderr, "[REPLAY] serial write failed traj=%llu chunk_index=%zu\n",
                 static_cast<unsigned long long>(nextChunk.chunk.trajectoryId), nextChunkToSend_);
    std::fflush(stderr);
    return false;
  }

  nextChunk.sent = true;
  ++nextChunkToSend_;
  plannerSnapshot_.trajectoryId = nextChunk.chunk.trajectoryId;
  plannerSnapshot_.dtMs = nextChunk.chunk.dtMs;
  plannerSnapshot_.startTimePi = nextChunk.startTimePiUs;
  std::fprintf(stderr,
               "[REPLAY] sent chunk_index=%zu traj=%llu start_pi_us=%llu dt_ms=%u actions=%zu\n",
               nextChunkToSend_ - 1,
               static_cast<unsigned long long>(nextChunk.chunk.trajectoryId),
               static_cast<unsigned long long>(nextChunk.startTimePiUs),
               static_cast<unsigned int>(nextChunk.chunk.dtMs),
               nextChunk.chunk.actions.size());
  std::fflush(stderr);
  lastDebugReportPiUs_ = nowPiUs;
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
