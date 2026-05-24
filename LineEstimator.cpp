#include "LineEstimator.h"

namespace lf {

LineEstimate LineEstimator::estimate(const LineSensorSample& sample) {
  LineEstimate result = {};
  result.valid = sample.valid;
  result.error = 0;

  if (!sample.valid) {
    result.lost = true;
    return result;
  }

  const bool left = sample.leftBlack;
  const bool right = sample.rightBlack;

  if (RobotConfig::kCenterMode == CenterMode::kBetweenSensors) {
    if (left && !right) {
      result.error = -RobotConfig::kLineErrorUnit;
    } else if (!left && right) {
      result.error = RobotConfig::kLineErrorUnit;
    } else if (left && right) {
      result.intersectionLike = true;
    } else {
      result.ambiguous = true;
    }
    return result;
  }

  if (left && right) {
    result.error = 0;
  } else if (left && !right) {
    result.error = -RobotConfig::kLineErrorUnit;
  } else if (!left && right) {
    result.error = RobotConfig::kLineErrorUnit;
  } else {
    result.valid = false;
    result.lost = true;
  }

  return result;
}

} // namespace lf
