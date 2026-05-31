#pragma once

#include "Timer1MotorPwm.h"

namespace lf {

// 有符号 (left, right) 命令 → trim → ramp → 死区跳变 → 方向切换空档 →
// driveMode 映射 → 四路 L9110 输入。无状态计算下沉到 .cpp 的匿名命名空间。
// 见 ADR-010。
class MotorDriver {
 public:
  void begin();
  void setTargetSpeeds(int16_t left, int16_t right);
  void update();
  void stopNow();

  int16_t currentLeft() const {
    return left_.current;
  }
  int16_t currentRight() const {
    return right_.current;
  }

 private:
  struct MotorState {
    int16_t target;
    int16_t current;
    int8_t lastDirection;
    uint8_t blankTicks;
  };

  MotorState left_ = {};
  MotorState right_ = {};

  static void stepMotor(MotorState* state);
  Timer1MotorPwm::DutyFrame makeDutyFrame() const;
};

} // namespace lf
