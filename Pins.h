#pragma once

#include "BoardProfile.h"

namespace lf {
namespace Pins {

// 接线事实集中在这里：其它模块禁止散落 Arduino 引脚号。
constexpr uint8_t kLeftMotorIbPin = 3;
constexpr uint8_t kLeftMotorIaPin = 5;
constexpr uint8_t kRightMotorIbPin = 9;
constexpr uint8_t kRightMotorIaPin = 10;

constexpr uint8_t kLeftSensorEnablePin = 2;
constexpr uint8_t kRightSensorEnablePin = A5;
constexpr uint8_t kLeftSensorOutPin = A1;
constexpr uint8_t kRightSensorOutPin = A0;

constexpr uint8_t kUltrasonicEchoPin = 12;
constexpr uint8_t kUltrasonicTriggerPin = 13;

// Arduino 引脚到 ATmega328P 端口位的硬绑定，用于直接寄存器访问。
constexpr uint8_t kLeftMotorIbBit = PD3;
constexpr uint8_t kLeftMotorIaBit = PD5;
constexpr uint8_t kRightMotorIbBit = PB1;
constexpr uint8_t kRightMotorIaBit = PB2;

constexpr uint8_t kLeftSensorEnableBit = PD2;
constexpr uint8_t kRightSensorEnableBit = PC5;
constexpr uint8_t kLeftSensorOutBit = PC1;
constexpr uint8_t kRightSensorOutBit = PC0;

constexpr uint8_t kUltrasonicEchoBit = PB4;
constexpr uint8_t kUltrasonicTriggerBit = PB5;
constexpr uint8_t kUltrasonicEchoPcint = PCINT4;

constexpr uint8_t kLeftSensorAdcChannel = 1;
constexpr uint8_t kRightSensorAdcChannel = 0;

// 编译期守门：硬件接线改动必须显式修改本文件和文档。
static_assert(kLeftMotorIbPin == 3, "左电机 IB 必须保持在 D3。");
static_assert(kLeftMotorIaPin == 5, "左电机 IA 必须保持在 D5。");
static_assert(kRightMotorIbPin == 9, "右电机 IB 必须保持在 D9。");
static_assert(kRightMotorIaPin == 10, "右电机 IA 必须保持在 D10。");

static_assert(kLeftSensorEnablePin == 2, "左循迹 EN 必须保持在 D2。");
static_assert(kRightSensorEnablePin == A5, "右循迹 EN 必须保持在 A5/PC5。");
static_assert(kLeftSensorOutPin == A1, "左循迹 OUT 必须保持在 A1/ADC1。");
static_assert(kRightSensorOutPin == A0, "右循迹 OUT 必须保持在 A0/ADC0。");

static_assert(kUltrasonicEchoPin == 12, "超声波 ECHO 必须保持在 D12/PB4。");
static_assert(kUltrasonicTriggerPin == 13, "超声波 TRIG 必须保持在 D13/PB5。");

} // namespace Pins
} // namespace lf
