#include "AdcDriver.h"

namespace lf {
namespace AdcDriver {
namespace {

constexpr uint16_t kConversionGuardLoops = 60000;
uint8_t g_lastChannel = 0xFF;

bool convertOnce(uint16_t* value) {
  if (value == nullptr) {
    return false;
  }

  ADCSRA |= _BV(ADSC);

  // 生产控制不能在 ADC 异常时永久卡死；正常转换远小于这个上限。
  uint16_t guard = kConversionGuardLoops;
  while ((ADCSRA & _BV(ADSC)) != 0) {
    if (guard == 0) {
      return false;
    }
    --guard;
  }

  // ATmega328P 要求 10-bit ADC 结果先读 ADCL，再读 ADCH。
  const uint8_t low = ADCL;
  const uint8_t high = ADCH;
  *value = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
  return true;
}

} // namespace

void begin(const bool disableDigitalInputBuffers) {
  // AVcc 参考电压，ADC 时钟 16 MHz / 128 = 125 kHz。
  ADMUX = _BV(REFS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  g_lastChannel = 0xFF;

  if (disableDigitalInputBuffers) {
    DIDR0 |= _BV(ADC0D) | _BV(ADC1D);
  } else {
    DIDR0 &= static_cast<uint8_t>(~(_BV(ADC0D) | _BV(ADC1D)));
  }
}

bool read(const Channel channel, uint16_t* value) {
  const uint8_t rawChannel = static_cast<uint8_t>(channel);
  if (rawChannel > Pins::kLeftSensorAdcChannel) {
    return false;
  }

  ADMUX = static_cast<uint8_t>(_BV(REFS0) | rawChannel);

  if (g_lastChannel != rawChannel) {
    // 切换 MUX 后丢弃第一帧，避免前一通道残留影响阈值判断。
    uint16_t discard = 0;
    if (!convertOnce(&discard)) {
      return false;
    }
    g_lastChannel = rawChannel;
  }

  return convertOnce(value);
}

} // namespace AdcDriver
} // namespace lf
