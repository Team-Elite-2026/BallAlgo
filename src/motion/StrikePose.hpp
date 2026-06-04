#pragma once

namespace ballalgo {

struct PredictedBallPose {
  float xM = 0;
  float yM = 0;
  float vx = 0;
  float vy = 0;
};

void strikePoseBody(float bx, float by, float goalDeg, float& tx, float& ty);
void ballFieldMm(float rx, float ry, float bx, float by, float headingDeg, float& fx, float& fy);
PredictedBallPose predictBallBody(float bx, float by, float vx, float vy, float timeS);

}  // namespace ballalgo
