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
