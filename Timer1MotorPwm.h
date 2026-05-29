#pragma once

#include "Pins.h"
#include "RobotConfig.h"

namespace lf {
namespace Timer1MotorPwm {

// 0 表示整周期低电平，255 表示整周期高电平；电机层保证同侧不双高。
struct DutyFrame {
  uint8_t leftIb;
  uint8_t leftIa;
  uint8_t rightIb;
  uint8_t rightIa;
};

void begin();
void submit(const DutyFrame& duty);
void emergencyStop();
uint8_t takeControlTicks();
uint32_t captureTimeTicks();
uint32_t captureTimeTicksFromIsr();

// 小工具保持公开，方便后续 host 测试直接覆盖 duty 映射。
uint16_t dutyToEdgeTick(uint8_t duty);

} // namespace Timer1MotorPwm
} // namespace lf
