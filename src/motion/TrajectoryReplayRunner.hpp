#pragma once

#include "io/RobotSerial.hpp"
#include "motion/ActionChunkPublisher.hpp"
#include "motion/ClockSync.hpp"
#include "motion/TrajectoryReplay.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ballalgo {

class TrajectoryReplayRunner {
 public:
  bool loadArtifact(const std::string& path, std::string* error = nullptr);
  bool loaded() const { return loaded_; }
  const TrajectoryReplayArtifact& artifact() const { return artifact_; }
  const PlannerDebugSnapshot& plannerSnapshot() const { return plannerSnapshot_; }
  bool updateAndPublish(RobotSerial& serial, uint64_t nowPiUs);
  TrajectoryTargetSample currentTarget(uint64_t queryPiUs, float headingDeg) const;
  bool armed() const { return armed_; }
  bool haveClockSync() const { return haveClockSyncPing_; }

 private:
  struct ScheduledChunk {
    ReplayChunk chunk;
    uint64_t startTimePiUs = 0;
    bool sent = false;
  };

  void buildPlannerSnapshot();
  void armReplay(uint64_t nowPiUs);

  TrajectoryReplayArtifact artifact_;
  PlannerDebugSnapshot plannerSnapshot_;
  ClockSync clock_;
  std::vector<ScheduledChunk> scheduledChunks_;
  bool loaded_ = false;
  bool haveClockSyncPing_ = false;
  bool armed_ = false;
  uint64_t replayEpochPiUs_ = 0;
  size_t nextChunkToSend_ = 0;
};

}  // namespace ballalgo
