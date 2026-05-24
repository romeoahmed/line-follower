#pragma once

#include "Pins.h"

namespace lf {
namespace AdcDriver {

enum class Channel : uint8_t {
  kAdc0 = 0,
  kAdc1 = 1,
};

void begin(bool disableDigitalInputBuffers);
bool read(Channel channel, uint16_t* value);

} // namespace AdcDriver
} // namespace lf
