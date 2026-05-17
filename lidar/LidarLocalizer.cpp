#include <LidarLocalizer.h>

#include <algorithm>
#include <cmath>

LidarLocalizer::LidarLocalizer(float fieldWidthMm, float fieldHeightMm, float lidarYawOffsetDeg)
    : _fieldWidthMm(fieldWidthMm),
      _fieldHeightMm(fieldHeightMm),
      _lidarYawOffsetDeg(lidarYawOffsetDeg),
      _minDistanceMm(80.0f),
      _maxDistanceMm(6000.0f),
      _minIntensity(20) {}

PoseEstimate LidarLocalizer::update(const std::vector<LidarPoint>& points, float headingDeg) {
  std::vector<float> xEstimates;
  std::vector<float> yEstimates;
  xEstimates.reserve(points.size());
  yEstimates.reserve(points.size());

  for (const LidarPoint& point : points) {
    const float distance = static_cast<float>(point.distance());
    if (distance < _minDistanceMm || distance > _maxDistanceMm || point.intensity() < _minIntensity) {
      continue;
    }

    // LD19 packet angles are in hundredths of a degree.
    const float lidarAngleDeg = point.angle() * 0.01f;
    const float fieldAngleDeg = headingDeg + _lidarYawOffsetDeg + lidarAngleDeg;
    const float radians = degToRad(fieldAngleDeg);
    const float dx = cosf(radians);
    const float dy = sinf(radians);

    if (fabsf(dx) >= fabsf(dy)) {
      if (fabsf(dx) < 1e-4f) {
        continue;
      }
      float xEstimate = (dx > 0.0f) ? (_fieldWidthMm - distance * dx) : (-distance * dx);
      if (xEstimate > -200.0f && xEstimate < (_fieldWidthMm + 200.0f)) {
        xEstimates.push_back(xEstimate);
      }
    } else {
      if (fabsf(dy) < 1e-4f) {
        continue;
      }
      float yEstimate = (dy > 0.0f) ? (_fieldHeightMm - distance * dy) : (-distance * dy);
      if (yEstimate > -200.0f && yEstimate < (_fieldHeightMm + 200.0f)) {
        yEstimates.push_back(yEstimate);
      }
    }
  }

  PoseEstimate estimate;
  estimate.xSampleCount = xEstimates.size();
  estimate.ySampleCount = yEstimates.size();
  estimate.headingDeg = headingDeg;
  estimate.valid = (estimate.xSampleCount >= 4 && estimate.ySampleCount >= 4);

  if (estimate.valid) {
    estimate.xMm = clamp(robustAxisEstimate(xEstimates), 0.0f, _fieldWidthMm);
    estimate.yMm = clamp(robustAxisEstimate(yEstimates), 0.0f, _fieldHeightMm);
  } else {
    estimate.xMm = NAN;
    estimate.yMm = NAN;
  }

  return estimate;
}

void LidarLocalizer::setDistanceRange(float minDistanceMm, float maxDistanceMm) {
  _minDistanceMm = minDistanceMm;
  _maxDistanceMm = maxDistanceMm;
}

void LidarLocalizer::setMinIntensity(uint8_t minIntensity) {
  _minIntensity = minIntensity;
}

float LidarLocalizer::degToRad(float deg) {
  return deg * 0.01745329251994329577f;
}

float LidarLocalizer::median(std::vector<float>& values) {
  if (values.empty()) {
    return NAN;
  }
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  if (values.size() % 2 == 0) {
    return (values[middle - 1] + values[middle]) * 0.5f;
  }
  return values[middle];
}

float LidarLocalizer::clamp(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

float LidarLocalizer::robustAxisEstimate(std::vector<float>& estimates) {
  if (estimates.empty()) {
    return NAN;
  }

  float med = median(estimates);
  std::vector<float> filtered;
  filtered.reserve(estimates.size());
  for (float value : estimates) {
    if (fabsf(value - med) < 180.0f) {
      filtered.push_back(value);
    }
  }
  if (filtered.size() >= 3) {
    return median(filtered);
  }
  return med;
}
