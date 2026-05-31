#pragma once

#include "AdcDriver.h"
#include "FastIo.h"

namespace lf {

struct LineSensorSample {
  bool leftBlack;
  bool rightBlack;
  bool valid;
  uint16_t leftRaw;
  uint16_t rightRaw;
};

class LineSensors {
 public:
  // 跨 control tick 的多数表决深度。改值会自动联动 mask / majority；countBits()
  // 是通用的。仅 LineSensors 内部使用。
  static constexpr uint8_t kHistoryDepth = 3;
  static constexpr uint8_t kHistoryMask = (1u << kHistoryDepth) - 1u;
  static constexpr uint8_t kHistoryMajority = kHistoryDepth / 2 + 1;
  static_assert(kHistoryDepth >= 1 && kHistoryDepth <= 8, "history 必须 fit in uint8_t。");

  void begin();
  LineSensorSample sample();

 private:
  uint8_t leftHistory_ = 0;
  uint8_t rightHistory_ = 0;
  uint8_t sampleCount_ = 0;
  bool leftAnalogBlack_ = false;
  bool rightAnalogBlack_ = false;

  static bool levelMeansBlack(bool levelHigh);
  static bool applyHistory(uint8_t* history, bool black, uint8_t sampleCount);
  static bool adcMeansBlack(uint16_t value, bool previousBlack);
  LineSensorSample sampleDigital();
  LineSensorSample sampleAdc();
};

} // namespace lf
