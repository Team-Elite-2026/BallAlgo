#pragma once

#include <cstdint>
#include <string>

namespace ballalgo::params {

struct Params {
    // Motor model
    float    motorNoLoadRpm           = 1620.f;
    float    motorLoadCompOmega       = 50.f;
    float    wheelRadiusM             = 0.025f;
    float    chassisRadiusM           = 0.090f;
    float    motorKs                  = 0.119f;
    float    motorKv                  = 0.139f;
    float    motorKa                  = 0.00074f;
    float    motorVBus                = 12.0f;
    float    aMaxGripAccel            = 1.5f;
    float    profilerJMax             = 10.0f;
    float    wheelAngle0              = -2.5307f;
    float    wheelAngle1              = -0.6109f;
    float    wheelAngle2              =  0.6109f;
    float    wheelAngle3              =  2.5307f;
    float    vMaxX                    = 0.8f;
    float    vMaxY                    = 0.6f;
    float    aMaxX                    = 1.2f;
    float    aMaxY                    = 1.0f;
    float    omegaMaxRadS             = 6.0f;

    // Ball Kalman filter
    float    ballKfMeasVar            = 0.02f;
    float    ballKfProcessPosVar      = 0.5f;
    float    ballKfProcessVelVar      = 2.f;
    float    ballKfFrictionGamma      = 0.95f;
    float    ballKfVelInitVar         = 4.0f;
    double   ballMaxStaleS            = 0.22;

    // Pose Kalman filter
    float    poseKfProcessPosVar      = 50.f;
    float    poseKfProcessVelVar      = 200.f;
    float    poseKfMeasPosVar         = 400.f;
    double   poseMaxStaleS            = 0.18;
    float    poseKfMouseAccelVar      = 250000.f;
    float    poseKfMouseMeasVar       = 2500.f;

    // Goal bearing EKF
    float    goalBearingBaseVarRad2   = 0.0035f;
    float    goalBearingPenaltyRad2   = 0.5f;
    float    goalCertaintyThreshold   = 0.25f;

    // Strategy / gameplay
    float    ballPredictionDamping         = 0.96f;
    float    strikeOffsetM                 = 0.12f;
    float    strikeInterceptMaxTimeS       = 1.0f;
    int      strikeInterceptMaxIterations  = 5;
    float    strikeInterceptConvergeS      = 0.025f;
    int      dribblerActivePower           = 255;
    float    dribblerCaptureDistMm         = 220.f;
    float    kickAimToleranceDeg           = 12.f;
    float    kickMinGoalForwardMm          = 120.f;
    uint64_t kickCooldownUs                = 750000;
    float    defenseGoalLineYMinCm         = 40.f;
    float    defenseGoalLineXMaxCm         = 55.f;
    float    defenseGoalLineQuadraticC     = 2000.f;
    float    defenseFutureBallMaxTimeS     = 0.5f;
    float    defenseImpactMarginS          = 0.08f;
    float    roleSwitchMarginCm            = 15.f;
    int      roleConfirmSamples            = 3;
    float    astarBallClearMm              = 80.f;
    float    commandGoalPositionToleranceMm = 60.f;
    float    commandGoalHeadingToleranceDeg = 5.f;
    float    offenseIntakeOffsetMm         = 90.f;
    float    offenseBoundaryInsetXMm       = 100.f;
    float    offenseBoundaryInsetYMm       = 100.f;
};

bool          load(const std::string& path);
const Params& get();

}  // namespace ballalgo::params
