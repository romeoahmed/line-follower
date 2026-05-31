#pragma once

#include "LineSensors.h"

namespace lf {

// 互斥的循迹状态。enum 让 RobotController 的 switch 享受编译器穷举守门，
// 避免可表达却非法的状态组合。
enum class LineState : uint8_t {
  kOffsetLeft = 0, // 线在左传感器下方；error = -kLineErrorUnit
  kOffsetRight,    // 线在右传感器下方；error = +kLineErrorUnit
  kCentered,       // 只在 kOnLine 模式出现：双黑视为居中，error = 0
  kAmbiguous,      // 双白；error = 0
  kIntersection,   // 只在 kBetweenSensors 模式出现：双黑视为交叉/宽线，error = 0
  kInvalid,        // 传感器采样失败（例如 ADC 超时），上层按失线处理
};

struct LineEstimate {
  LineState state;
  int16_t error;

  bool isOffset() const {
    return state == LineState::kOffsetLeft || state == LineState::kOffsetRight;
  }
  bool isLost() const {
    return state == LineState::kInvalid;
  }
  bool isAmbiguous() const {
    return state == LineState::kAmbiguous;
  }
  bool isIntersection() const {
    return state == LineState::kIntersection;
  }
};

class LineEstimator {
 public:
  static LineEstimate estimate(const LineSensorSample& sample);
};

} // namespace lf
