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

  // 正常 13-cycle 转换在 125 kHz 时钟下约 104 µs；guard 上限远大于此，仅防 ADC
  // 异常（外部干扰让 ADSC 永不清零）导致控制循环死锁。
  uint16_t guard = kConversionGuardLoops;
  while ((ADCSRA & _BV(ADSC)) != 0) {
    if (guard == 0) {
      return false;
    }
    --guard;
  }

  // 读序数据手册强制：先 ADCL 后 ADCH。若反序，CPU 读 ADCL 时 ADC 数据寄存器
  // 会被冻结直到 ADCH 也被读，从而漏一个新转换结果。
  const uint8_t low = ADCL;
  const uint8_t high = ADCH;
  *value = static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
  return true;
}

} // namespace

void begin(const bool disableDigitalInputBuffers) {
  // AVcc 参考电压（REFS0=1, REFS1=0）；ADC 时钟 = 16 MHz / 128 = 125 kHz，
  // 落在 datasheet 推荐的 50-200 kHz 区间内（10-bit 全精度）。
  ADMUX = _BV(REFS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  g_lastChannel = 0xFF;

  // ADC 模式下关掉对应引脚的数字输入缓冲器：模拟电平长期落在数字阈值附近会
  // 让 Schmitt 输入级反复切换、增加功耗并干扰 ADC。
  if (disableDigitalInputBuffers) {
    DIDR0 |= _BV(ADC0D) | _BV(ADC1D);
  } else {
    DIDR0 &= static_cast<uint8_t>(~(_BV(ADC0D) | _BV(ADC1D)));
  }
}

bool read(const Channel channel, uint16_t* value) {
  // 只允许 ADC0/ADC1（两路循迹传感器）。其它通道未经数据手册采样保持电容稳定
  // 时间验证，按 fail 处理避免误用。
  if (channel != Channel::kAdc0 && channel != Channel::kAdc1) {
    return false;
  }
  const uint8_t rawChannel = static_cast<uint8_t>(channel);

  ADMUX = static_cast<uint8_t>(_BV(REFS0) | rawChannel);

  if (g_lastChannel != rawChannel) {
    // MUX 切换后 S/H 电容从前一通道转向新通道，首次转换可能受残留影响。
    // 显式跑一次空转换让 S/H 稳定到新通道——常见的防御性做法。
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
