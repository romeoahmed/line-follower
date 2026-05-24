#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>

#if !defined(ARDUINO_ARCH_AVR)
#error "本项目只支持 Arduino AVR core。"
#endif

#if !defined(__AVR_ATmega328P__)
#error "本项目专门针对 ATmega328P / Arduino UNO 兼容板。"
#endif

#if !defined(F_CPU) || (F_CPU != 16000000UL)
#error "本项目按 16 MHz 时钟计算 Timer1 与 ADC 参数。"
#endif

namespace lf {
namespace BoardProfile {

constexpr uint32_t kCpuHz = F_CPU;
constexpr uint8_t kDigitalPinCount = NUM_DIGITAL_PINS;
constexpr uint8_t kAnalogInputCount = NUM_ANALOG_INPUTS;

static_assert(kCpuHz == 16000000UL, "Timer1 参数依赖 16 MHz 主频。");
static_assert(kDigitalPinCount == 20, "需要 ArduinoCore-avr standard variant 的 UNO 引脚表。");
static_assert(kAnalogInputCount == 6, "UNO standard variant 只把 A0-A5 当普通模拟输入。");
static_assert(PIN_A0 == 14 && PIN_A5 == 19, "A0-A5 映射必须是 UNO standard variant。");
#if defined(PIN_A6) && defined(PIN_A7)
static_assert(PIN_A6 == 20 && PIN_A7 == 21, "A6/A7 仅保留为 ADC-only 硬件事实。");
#endif

} // namespace BoardProfile
} // namespace lf
