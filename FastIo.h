#pragma once

#include "Pins.h"
#include "RobotConfig.h"

#include <avr/interrupt.h>

namespace lf {
namespace FastIo {

constexpr uint8_t kLeftSensorEnableMask = _BV(Pins::kLeftSensorEnableBit);
constexpr uint8_t kRightSensorEnableMask = _BV(Pins::kRightSensorEnableBit);
constexpr uint8_t kLeftSensorOutMask = _BV(Pins::kLeftSensorOutBit);
constexpr uint8_t kRightSensorOutMask = _BV(Pins::kRightSensorOutBit);

inline bool isActiveHigh(const ActiveLevel level) {
  return level == ActiveLevel::kHigh;
}

inline void writeSensorEnable(const bool enabled) {
  const bool driveHigh = (enabled == isActiveHigh(RobotConfig::kSensorEnableActiveLevel));
  const uint8_t oldSreg = SREG;
  cli();
  if (driveHigh) {
    PORTD |= kLeftSensorEnableMask;
    PORTC |= kRightSensorEnableMask;
  } else {
    PORTD &= static_cast<uint8_t>(~kLeftSensorEnableMask);
    PORTC &= static_cast<uint8_t>(~kRightSensorEnableMask);
  }
  SREG = oldSreg;
}

inline void beginSensorPins() {
  // EN 为推挽输出；OUT 为高阻输入，电平由循迹模块自己驱动。
  DDRD |= kLeftSensorEnableMask;
  DDRC |= kRightSensorEnableMask;

  // OUT 引脚保持输入，关闭内部上拉，避免改变外部模块电平。
  DDRC &= static_cast<uint8_t>(~(kLeftSensorOutMask | kRightSensorOutMask));
  PORTC &= static_cast<uint8_t>(~(kLeftSensorOutMask | kRightSensorOutMask));

  writeSensorEnable(false);
}

inline uint8_t readSensorPortC() {
  // 左右 OUT 同在 PORTC，一次读取可减少两路采样的时间偏差。
  return PINC;
}

inline bool readLeftSensorHigh() {
  return (readSensorPortC() & kLeftSensorOutMask) != 0;
}

inline bool readRightSensorHigh() {
  return (readSensorPortC() & kRightSensorOutMask) != 0;
}

} // namespace FastIo
} // namespace lf
