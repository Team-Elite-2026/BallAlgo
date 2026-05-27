#pragma once

namespace ballalgo {

struct AngleDist {
  double angleDeg = -5;
  double distPx = -5;
};

AngleDist angleAndDistance(int cx, int cy, int xoff, int yoff);
double calibrateBallDist(double distPx);
void polarToBodyXY(double angleDeg, double distM, double& xM, double& yM);
void polarToBodyXY(double angleDeg, double distM, float& xM, float& yM);
void fieldVelToBody(float vxMmS, float vyMmS, float headingDeg, float& vxB, float& vyB);

}  // namespace ballalgo
