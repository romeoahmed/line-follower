#pragma once

#include <stdint.h>

namespace lf {

// Q8 定点 PD 控制器：实际增益 = 常量 / 256。
// 不提供 ki：误差是 {-kLineErrorUnit, 0, +kLineErrorUnit} 三态离散值，I 项无物理意义；
// 左右电机的稳态偏置由 k{Left,Right}MotorTrimPermille 在 MotorDriver 直接补偿。
// 详见 ADR-008。
struct PdGainsQ8 {
  int16_t kp;
  int16_t kd;
};

class PdController {
 public:
  void reset();
  int16_t update(int16_t error, const PdGainsQ8& gains, int16_t maxCorrection);

 private:
  int16_t previousError_ = 0;
  bool hasPrevious_ = false;
};

} // namespace lf
