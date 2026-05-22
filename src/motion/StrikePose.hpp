#pragma once

namespace ballalgo {

void strikePoseBody(float bx, float by, float goalDeg, float& tx, float& ty);
void ballFieldMm(float rx, float ry, float bx, float by, float headingDeg, float& fx, float& fy);

}  // namespace ballalgo
