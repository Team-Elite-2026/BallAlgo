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
inline constexpr float kLidarYawOffsetDeg = 0.f;

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

inline constexpr float kPoseKfProcessPosVar = 50.f;
inline constexpr float kPoseKfProcessVelVar = 200.f;
inline constexpr float kPoseKfMeasPosVar = 400.f;
inline constexpr double kPoseMaxStaleS = 0.18;

// Step 1a — mouse-sensor dead-reckoning (1 kHz predict between LiDAR frames).
// Process acceleration noise (DWNA sigma_a, mm/s^2) and the mouse velocity
// measurement variance ((mm/s)^2). Lower meas var = trust the mouse more.
inline constexpr bool kEnableMouseFusion = true;
inline constexpr float kPoseKfMouseAccelVar = 250000.f;  // (mm/s^2)^2
inline constexpr float kPoseKfMouseMeasVar = 2500.f;     // (mm/s)^2

// Step 1b — dual-goal bearing-only EKF correction.
// R_i = base^2 + penalty * (1 - c)^2 (radians^2). Skip the update entirely when
// the vision certainty c falls below the threshold.
inline constexpr bool kEnableGoalBearingFusion = true;
inline constexpr float kGoalBearingBaseVarRad2 = 0.0035f;   // ~3.4 deg sigma
inline constexpr float kGoalBearingPenaltyRad2 = 0.5f;      // blows up under occlusion
inline constexpr float kGoalCertaintyThreshold = 0.25f;
// Absolute field positions of the two goal centres (mm, corner origin).
// Yellow defended/attack split is handled by the caller; these are fixed posts.
inline constexpr float kBlueGoalXMm = kFieldWidthMm * 0.5f;
inline constexpr float kBlueGoalYMm = 0.f;
inline constexpr float kYellowGoalXMm = kFieldWidthMm * 0.5f;
inline constexpr float kYellowGoalYMm = kFieldHeightMm;

// Shared asymmetric drivetrain / motor model (Steps 3 & 4). Holonomic 4-wheel
// layout, wheel axis angles measured from robot-forward (0 = +y). Convention:
// v_wheel_i = sin(alpha_i)*vx + cos(alpha_i)*vy - R_chassis*omega.
inline constexpr float kMotorNoLoadRpm = 1620.f;
inline constexpr float kMotorLoadCompOmega = 50.f;  // rad/s deducted for load
inline constexpr float kWheelRadiusM = 0.025f;
inline constexpr float kChassisRadiusM = 0.090f;
inline constexpr float kMotorKs = 0.119f;    // static-friction voltage (V)
inline constexpr float kMotorKv = 0.139f;    // back-EMF (V per rad/s)
inline constexpr float kMotorKa = 0.00074f;  // provisional motor-only accel term (V per rad/s^2)
inline constexpr float kMotorVBus = 12.0f;
inline constexpr float kAMaxGripAccel = 1.5f;  // total traction budget (m/s^2)
inline constexpr float kProfilerJMax = 10.0f;  // path jerk limit (1/s^3)
inline constexpr float kWheelAngle0 = -2.5307f;
inline constexpr float kWheelAngle1 = -0.6109f;
inline constexpr float kWheelAngle2 = 0.6109f;
inline constexpr float kWheelAngle3 = 2.5307f;

inline constexpr float kBallDistToM = 0.001f;
inline constexpr float kBallKfMeasVar = 0.02f;
inline constexpr float kBallKfProcessPosVar = 0.5f;
inline constexpr float kBallKfProcessVelVar = 2.f;
// Step 1 (ball KF): viscous-friction damping applied to the ball velocity states
// each camera frame. F couples position with gamma-damped velocity:
//   x_{k+1} = x_k + v_k * dt,  v_{k+1} = gamma^(dt/dt_cam) * v_k.
// gamma in (0,1): closer to 1 = less rolling resistance. Spec value 0.95.
inline constexpr float kBallKfFrictionGamma = 0.95f;
// Camera frame period used to normalise the per-frame friction decay.
inline constexpr float kBallKfCamDtS = 1.0f / static_cast<float>(kCameraFps);
// High process noise on the velocity states (spec: "high measurement noise for
// velocity") so the filter reacts quickly to real ball acceleration / kicks.
inline constexpr float kBallKfVelInitVar = 4.0f;
inline constexpr float kStrikeOffsetM = 0.12f;
inline constexpr float kBallPredictionDamping = 0.96f;
inline constexpr float kStrikeInterceptMaxTimeS = 1.0f;
inline constexpr int kStrikeInterceptMaxIterations = 5;
inline constexpr float kStrikeInterceptConvergeS = 0.025f;
inline constexpr float kOffenseRobotDiameterMm = 180.f;
inline constexpr float kOffenseRobotRadiusMm = 0.5f * kOffenseRobotDiameterMm;
inline constexpr float kOffenseBallDiameterMm = 42.67f;
inline constexpr float kOffenseBallRadiusMm = 0.5f * kOffenseBallDiameterMm;
inline constexpr float kOffenseIntakeOffsetMm = 90.f;
// Goal-line handling is along the field Y axis because the goals sit on the
// short walls at y = 0 and y = fieldHeight.
inline constexpr float kOffenseGoalLineInsetYMm = 140.f;
inline constexpr float kOffenseLowGoalLineYMm = kOffenseGoalLineInsetYMm;
inline constexpr float kOffenseHighGoalLineYMm = kFieldHeightMm - kOffenseGoalLineInsetYMm;
inline constexpr float kOffenseBoundaryInsetXMm = 100.f;
inline constexpr float kOffenseBoundaryInsetYMm = 100.f;
inline constexpr uint8_t kDribblerActivePower = 255u;
inline constexpr float kDribblerCaptureDistMm = 220.f;
inline constexpr float kKickAimToleranceDeg = 12.f;
inline constexpr float kKickMinGoalForwardMm = 120.f;
inline constexpr uint64_t kKickCooldownUs = 750000;
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
inline constexpr uint32_t kRobotId = 0;
inline constexpr const char* kPeerBtAddress = "";
inline constexpr int kBtRfcommChannel = 1;
inline constexpr float kTeamPacketHz = 30.f;
inline constexpr uint32_t kPeerStaleMs = 300;
inline constexpr float kRoleSwitchMarginCm = 15.f;
inline constexpr int kRoleConfirmSamples = 3;

//velocity
inline constexpr float kVMaxX = 0.8f;
inline constexpr float kVMaxY = 0.6f;

//max accel
inline constexpr float kAMaxX = 1.2f;
inline constexpr float kAMaxY = 1.0f;

// Spec Step 3: 5 cm grid. Heading split into 8 x 45-degree slices.
inline constexpr int kAstarCellMm = 50;
inline constexpr int kAstarHeadingBins = 8;
// Heading-change (K_rot) and travel-direction-change (K_curve) cost penalties.
// Units: seconds per radian. Tunable; keep small so time dominates.
inline constexpr float kAstarKRotSPerRad = 0.08f;
inline constexpr float kAstarKCurveSPerRad = 0.05f;
// Ball is inflated as a circular obstacle of this radius (mm). Must be smaller
// than the strike offset so the approach pose is reachable.
inline constexpr float kAstarBallClearMm = 80.f;
inline constexpr float kReplanHz = 15.f;
inline constexpr float kCommandGoalPositionToleranceMm = 60.f;
inline constexpr float kCommandGoalHeadingToleranceDeg = 5.f;

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
inline constexpr int   kDeskewRefTimeMode  = 1;      // 0=scan start, 1=midpoint, 2=scan end
inline constexpr float kLidarScanPeriodS   = 0.1f;   // LD19 nominal 10 Hz; fallback only
inline constexpr float kMaxMotionDataAgeS  = 0.5f;   // stale motion → fallback to raw scan
inline constexpr float kMaxExtrapolationS  = 0.020f; // max extrapolation beyond history edges
inline constexpr bool  kDebugDeskew        = false;
inline constexpr bool  kUseLinearAccel     = false;  // reserved, not yet implemented
inline constexpr bool  kUseAngularAccel    = false;  // reserved, not yet implemented

}  // namespace ballalgo::config
