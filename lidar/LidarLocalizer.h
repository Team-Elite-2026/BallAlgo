#ifndef LIDAR_LOCALIZER_H
#define LIDAR_LOCALIZER_H

#include <Arduino.h>
#include <lidar_reader.h>
#include <vector>

struct PoseEstimate {
  float xMm;
  float yMm;
  float headingDeg;
  bool valid;
  uint16_t xSampleCount;
  uint16_t ySampleCount;
};

class LidarLocalizer {
 public:
  LidarLocalizer(float fieldWidthMm, float fieldHeightMm, float lidarYawOffsetDeg = 0.0f);

  PoseEstimate update(const std::vector<LidarPoint>& points, float headingDeg);

  void setDistanceRange(float minDistanceMm, float maxDistanceMm);
  void setMinIntensity(uint8_t minIntensity);

 private:
  static float degToRad(float deg);
  static float median(std::vector<float>& values);
  static float clamp(float value, float lower, float upper);
  float robustAxisEstimate(std::vector<float>& estimates);

  float _fieldWidthMm;
  float _fieldHeightMm;
  float _lidarYawOffsetDeg;
  float _minDistanceMm;
  float _maxDistanceMm;
  uint8_t _minIntensity;
};

#endif
