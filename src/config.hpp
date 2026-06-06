#pragma once

#include <cstdint>
#include <string>

namespace ballalgo::config {

inline constexpr const char* kSerialPort = "/dev/serial0";
inline constexpr int kSerialBaud = 2000000;
inline constexpr bool kEnableSerial = true;

inline constexpr const char* kLidarPort = "/dev/ttyAMA4";
inline constexpr int kLidarBaud = 230400;
inline constexpr int kLidarPwmGpio = 12;
inline constexpr bool kLidarPwmHoldLow = true;

inline constexpr float kFieldWidthMm = 1820.f;
inline constexpr float kFieldHeightMm = 2430.f;
inline constexpr float kLidarYawOffsetDeg = 0.f;

inline constexpr int kFrameWidth = 655;
inline constexpr int kFrameHeight = 600;
inline constexpr int kCameraFps = 120;
inline constexpr int kExposureUs = 10000;
inline constexpr int kCameraFocus = 108;
inline constexpr int kCameraWhiteBalance = 4000;
inline constexpr int kCameraBrightness = 0;
inline constexpr int kCameraContrast = 1;
inline constexpr int kCameraSaturation = 1;
inline constexpr int kCameraGain = 0;

inline constexpr const char* kThresholdsJson = "thresholds.json";

inline constexpr int kNumSectors = 12;
inline constexpr int kSectorAngleDeg = 30;
inline constexpr int kMinAreaPix = 5;
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

inline constexpr int kLidarPointsWindow = 720;

inline constexpr float kPoseKfProcessPosVar = 50.f;
inline constexpr float kPoseKfProcessVelVar = 200.f;
inline constexpr float kPoseKfMeasPosVar = 400.f;
inline constexpr double kPoseMaxStaleS = 0.18;

inline constexpr float kBallDistToM = 0.001f;
inline constexpr float kBallKfMeasVar = 0.02f;
inline constexpr float kBallKfProcessPosVar = 0.5f;
inline constexpr float kBallKfProcessVelVar = 2.f;
inline constexpr float kStrikeOffsetM = 0.12f;
inline constexpr float kBallPredictionDamping = 0.96f;
inline constexpr float kStrikeInterceptMaxTimeS = 2.0f;
inline constexpr int kStrikeInterceptMaxIterations = 5;
inline constexpr float kStrikeInterceptConvergeS = 0.025f;
// Assumes yellow-goal attack runs in the +X field direction.
// Move this inset to push both offense full-field goal-line guards forward/back together.
inline constexpr float kOffenseGoalLineInsetXMm = 140.f;
inline constexpr float kOffenseOurGoalLineXMm = kOffenseGoalLineInsetXMm;
inline constexpr float kOffenseEnemyGoalLineXMm = kFieldWidthMm - kOffenseGoalLineInsetXMm;
inline constexpr float kOffenseBoundaryInsetXMm = 100.f;
inline constexpr float kOffenseBoundaryInsetYMm = 100.f;
inline constexpr float kDefenseGoalLineYMinCm = 40.f;
inline constexpr float kDefenseGoalLineXMaxCm = 55.f;
inline constexpr float kDefenseGoalLineQuadraticC = 2000.f;
inline constexpr float kDefenseFutureBallMaxTimeS = 0.5f;
inline constexpr float kDefenseImpactMarginS = 0.08f;
inline constexpr float kOmegaMaxRadS = 6.0f;
inline constexpr double kBallMaxStaleS = 0.22;

inline constexpr int kChunkDtMs = 4;
inline constexpr int kChunkMaxActions = 50;
inline constexpr float kChunkPublishHz = 60.f;
inline constexpr int kSerialLatencyMarginUs = 2000;

//velocity
inline constexpr float kVMaxX = 0.8f;
inline constexpr float kVMaxY = 0.6f;

//max accel
inline constexpr float kAMaxX = 1.2f;
inline constexpr float kAMaxY = 1.0f;

inline constexpr float kAMaxLateral = 0.8f;
inline constexpr float kKCurve = 0.15f;
inline constexpr float kJMaxTangential = 4.0f;
inline constexpr int kAstarCellMm = 80;
inline constexpr int kAstarHeadingBins = 8;
inline constexpr float kReplanHz = 15.f;
inline constexpr float kCommandGoalPositionToleranceMm = 60.f;
inline constexpr float kCommandGoalHeadingToleranceDeg = 5.f;

inline constexpr bool kEnableActionChunks = false;
inline constexpr bool kEnablePlannerCompare = false;

inline constexpr int kLostSentinel = -5;

}  // namespace ballalgo::config
