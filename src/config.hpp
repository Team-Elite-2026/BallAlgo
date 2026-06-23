#pragma once

#include <cstdint>
#include <string>

namespace ballalgo::config {

inline constexpr const char* kSerialPort = "/dev/ttyAMA2";
inline constexpr int kSerialBaud = 2000000;
inline constexpr bool kEnableSerial = true;

inline constexpr const char* kLidarPort = "/dev/ttyAMA4";
inline constexpr int kLidarBaud = 230400;
inline constexpr int kLidarPwmGpio = 12;
inline constexpr bool kLidarPwmHoldLow = true;

inline constexpr float kFieldWidthMm = 1820.f;
inline constexpr float kFieldHeightMm = 2430.f;
inline constexpr float kLidarYawOffsetDeg = 180.f;

inline constexpr int kFrameWidth = 655;
inline constexpr int kFrameHeight = 600;
inline constexpr int kCameraFps = 120;
inline constexpr int   kExposureUs   = 5000;
inline constexpr float kExposureSens = 1600.f;  // ISO; AnalogueGain = sens / 100
inline constexpr int kCameraFocus = 108;
inline constexpr int kCameraWhiteBalance = 4000;
inline constexpr int kCameraBrightness = 0;
inline constexpr int kCameraContrast = 1;
inline constexpr int kCameraSaturation = 1;

inline constexpr const char* kThresholdsJson = "thresholds.json";

inline constexpr int kNumSectors = 12;
inline constexpr int kSectorAngleDeg = 30;
inline constexpr int kMinAreaPix = 5;
// Goal blob area (px) treated as "fully visible" (certainty c = 1) for the
// Step 1b bearing-EKF occlusion model. Smaller blobs scale c down linearly.
inline constexpr int kGoalCertaintyRefAreaPix = 1200;
inline constexpr bool kUseMorph = true;
inline constexpr int kMorphIters = 1;
inline constexpr int kBallCloseIters = 1;
inline constexpr int kBallDilateIters = 10;
inline constexpr bool kIgnoreHsvValue = true;
inline constexpr bool kStopOnFirstHit = true;
inline constexpr float kVelAlpha = 0.7f;
inline constexpr float kVelMaxPx = 40.f;
inline constexpr int kLookaheadMin = 1;
inline constexpr int kLookaheadMax = 3;
inline constexpr float kLookaheadSpeedThresh = 12.f;

inline constexpr int kLidarPointsWindow = 440;

// Step 1a — mouse-sensor dead-reckoning enable flag.
inline constexpr bool kEnableMouseFusion = true;

// Step 1b — dual-goal bearing-only EKF enable flag.
inline constexpr bool kEnableGoalBearingFusion = true;
// Absolute field positions of the two goal centres (mm, corner origin).
// Yellow defended/attack split is handled by the caller; these are fixed posts.
inline constexpr float kBlueGoalXMm = kFieldWidthMm * 0.5f;
inline constexpr float kBlueGoalYMm = 0.f;
inline constexpr float kYellowGoalXMm = kFieldWidthMm * 0.5f;
inline constexpr float kYellowGoalYMm = kFieldHeightMm;

inline constexpr float kBallDistToM = 0.001f;
// Camera frame period used to normalise the per-frame friction decay.
inline constexpr float kBallKfCamDtS = 1.0f / static_cast<float>(kCameraFps);
inline constexpr float kOffenseRobotDiameterMm = 180.f;
inline constexpr float kOffenseRobotRadiusMm = 0.5f * kOffenseRobotDiameterMm;
inline constexpr float kOffenseBallDiameterMm = 42.67f;
inline constexpr float kOffenseBallRadiusMm = 0.5f * kOffenseBallDiameterMm;
// Goal-line handling is along the field Y axis because the goals sit on the
// short walls at y = 0 and y = fieldHeight.
inline constexpr float kOffenseGoalLineInsetYMm = 140.f;
inline constexpr float kOffenseLowGoalLineYMm = kOffenseGoalLineInsetYMm;
inline constexpr float kOffenseHighGoalLineYMm = kFieldHeightMm - kOffenseGoalLineInsetYMm;

inline constexpr int kChunkDtMs = 4;
inline constexpr int kChunkMaxActions = 50;
inline constexpr float kChunkPublishHz = 60.f;
inline constexpr int kSerialLatencyMarginUs = 2000;
inline constexpr uint32_t kRobotId = 0;
inline constexpr const char* kPeerBtAddress = "";
inline constexpr int kBtRfcommChannel = 1;
inline constexpr float kTeamPacketHz = 30.f;
inline constexpr uint32_t kPeerStaleMs = 300;

// Spec Step 3: 5 cm grid. Heading split into 8 x 45-degree slices.
inline constexpr int kAstarCellMm = 50;
inline constexpr int kAstarHeadingBins = 8;
// Heading-change (K_rot) and travel-direction-change (K_curve) cost penalties.
// Units: seconds per radian. Tunable; keep small so time dominates.
inline constexpr float kAstarKRotSPerRad = 0.08f;
inline constexpr float kAstarKCurveSPerRad = 0.05f;
inline constexpr float kReplanHz = 15.f;

// Step 2 — trajectory look-ahead pipelining. Constant compute+serial latency
// measured on the Pi (us) and the acceptable tracking-error sigma (mm) used to
// blend the chunk-predicted start state against the live EKF state.
inline constexpr uint64_t kPipelineLatencyUs = 8000;
inline constexpr float kTrackingSigmaMm = 40.f;

inline constexpr bool kEnableActionChunks = true;
inline constexpr bool kEnablePlannerCompare = false;
inline constexpr const char* kFoxgloveConfigPath = "foxglove_sim/foxglove.conf";

inline constexpr int kLostSentinel = -5;

// LiDAR scan deskewing (2D SE(2) motion compensation)
inline constexpr bool  kEnableLidarDeskew  = true;
// TEMPORARY (bench/rectangle test): derive the deskew motion estimate from the
// LiDAR pose Kalman filter (smoothed, one-frame-delayed) instead of the mouse
// sensor. When true, the pose KF is driven purely by LiDAR (no mouse fusion) so
// its velocity reflects LiDAR only. Set back to false to restore mouse-based deskew.
inline constexpr bool  kDeskewVelocityFromLidar = true;
inline constexpr int   kDeskewRefTimeMode  = 1;      // 0=scan start, 1=midpoint, 2=scan end
inline constexpr float kLidarScanPeriodS   = 0.1f;   // LD19 nominal 10 Hz; fallback only
inline constexpr float kMaxMotionDataAgeS  = 0.5f;   // stale motion → fallback to raw scan
inline constexpr float kMaxExtrapolationS  = 0.020f; // max extrapolation beyond history edges
inline constexpr bool  kDebugDeskew        = false;
inline constexpr bool  kUseLinearAccel     = false;  // reserved, not yet implemented
inline constexpr bool  kUseAngularAccel    = false;  // reserved, not yet implemented

}  // namespace ballalgo::config
