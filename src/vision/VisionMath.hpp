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
void bodyToFieldOffsetMm(float bodyXMm, float bodyYMm, float headingDeg, float& fieldDXMm,
                         float& fieldDYMm);
void fieldToBodyOffsetMm(float fieldDXMm, float fieldDYMm, float headingDeg, float& bodyXMm,
                         float& bodyYMm);
void fieldVelToBody(float vxMmS, float vyMmS, float headingDeg, float& vxB, float& vyB);

}  // namespace ballalgo
