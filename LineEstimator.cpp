#include "LineEstimator.h"

namespace lf {

LineEstimate LineEstimator::estimate(const LineSensorSample& sample) {
  if (!sample.valid) {
    return LineEstimate{LineState::kInvalid, 0};
  }

  const bool left = sample.leftBlack;
  const bool right = sample.rightBlack;

  // kBetweenSensors 模式：线穿过两个传感器之间。单黑 = 线在该侧；双黑 = 交叉/宽线；
  // 双白 = 正常居中或脱轨（区分由 RobotController 的双白超时机制决定）。
  if (RobotConfig::kCenterMode == CenterMode::kBetweenSensors) {
    if (left && !right) {
      return LineEstimate{LineState::kOffsetLeft, -RobotConfig::kLineErrorUnit};
    }
    if (!left && right) {
      return LineEstimate{LineState::kOffsetRight, RobotConfig::kLineErrorUnit};
    }
    if (left && right) {
      return LineEstimate{LineState::kIntersection, 0};
    }
    return LineEstimate{LineState::kAmbiguous, 0};
  }

  // kOnLine 模式：单黑表示线偏到另一侧；双黑视为正中；双白视为脱轨。
  if (left && right) {
    return LineEstimate{LineState::kCentered, 0};
  }
  if (left && !right) {
    return LineEstimate{LineState::kOffsetLeft, -RobotConfig::kLineErrorUnit};
  }
  if (!left && right) {
    return LineEstimate{LineState::kOffsetRight, RobotConfig::kLineErrorUnit};
  }
  return LineEstimate{LineState::kInvalid, 0};
}

} // namespace lf
