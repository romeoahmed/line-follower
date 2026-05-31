#include "UltrasonicRangeSensor.h"

#include "AtomicGuard.h"

#include <avr/interrupt.h>

namespace lf {
namespace {

constexpr uint8_t kEchoMask = _BV(Pins::kUltrasonicEchoBit);
constexpr uint8_t kTriggerMask = _BV(Pins::kUltrasonicTriggerBit);

enum class EchoCaptureState : uint8_t {
  kIdle = 0,
  kWaitingForRise,
  kHigh,
};

volatile EchoCaptureState g_captureState = EchoCaptureState::kIdle;
volatile uint32_t g_echoRiseTicks = 0;
volatile uint32_t g_echoPulseTicks = 0;
volatile bool g_pulsePending = false;
volatile bool g_timeoutPending = false;
uint8_t g_instanceCount = 0;

// 模算术差比较：32-bit Timer1 时基约 35 分钟回卷一次。无符号减法在补码上等价于
// (now - since) mod 2^32，所以只要测量间隔 ≪ 2^31 ticks，wrap 后比较仍然正确。
bool elapsedAtLeast(const uint32_t nowTicks, const uint32_t sinceTicks,
                    const uint32_t durationTicks) {
  return static_cast<uint32_t>(nowTicks - sinceTicks) >= durationTicks;
}

// 同理：先做无符号减法再转 int32_t 取符号，能正确处理 deadline 跨过 2^32 的情况，
// 只要 |now - deadline| < 2^31。
bool deadlineReached(const uint32_t nowTicks, const uint32_t deadlineTicks) {
  return static_cast<int32_t>(nowTicks - deadlineTicks) >= 0;
}

bool readEchoHigh() {
  return (PINB & kEchoMask) != 0;
}

void writeTriggerHigh(const bool high) {
  AtomicGuard guard;
  if (high) {
    PORTB |= kTriggerMask;
  } else {
    PORTB &= static_cast<uint8_t>(~kTriggerMask);
  }
}

void armEchoCapture() {
  AtomicGuard guard;
  g_captureState = EchoCaptureState::kWaitingForRise;
  g_echoRiseTicks = 0;
  g_echoPulseTicks = 0;
  g_pulsePending = false;
  g_timeoutPending = false;
}

bool echoCaptureBusy() {
  AtomicGuard guard;
  return g_captureState != EchoCaptureState::kIdle;
}

void markEchoTimeout() {
  AtomicGuard guard;
  if (g_captureState != EchoCaptureState::kIdle) {
    g_captureState = EchoCaptureState::kIdle;
    g_timeoutPending = true;
  }
}

void cancelEchoCapture() {
  AtomicGuard guard;
  g_captureState = EchoCaptureState::kIdle;
  g_echoRiseTicks = 0;
  g_echoPulseTicks = 0;
  g_pulsePending = false;
  g_timeoutPending = false;
}

bool takeEchoResult(uint32_t* pulseTicks, bool* timedOut) {
  if (pulseTicks == nullptr || timedOut == nullptr) {
    return false;
  }

  AtomicGuard guard;
  if (g_pulsePending) {
    *pulseTicks = g_echoPulseTicks;
    *timedOut = false;
    g_pulsePending = false;
    return true;
  }
  if (g_timeoutPending) {
    *pulseTicks = 0;
    *timedOut = true;
    g_timeoutPending = false;
    return true;
  }
  return false;
}

void onEchoPinChangeFromIsr() {
  const bool echoHigh = readEchoHigh();

  if (g_captureState == EchoCaptureState::kWaitingForRise && echoHigh) {
    g_echoRiseTicks = Timer1MotorPwm::captureTimeTicksFromIsr();
    g_captureState = EchoCaptureState::kHigh;
    return;
  }

  if (g_captureState == EchoCaptureState::kHigh && !echoHigh) {
    const uint32_t nowTicks = Timer1MotorPwm::captureTimeTicksFromIsr();
    g_echoPulseTicks = static_cast<uint32_t>(nowTicks - g_echoRiseTicks);
    g_pulsePending = true;
    g_captureState = EchoCaptureState::kIdle;
  }
}

void beginUltrasonicPinsAndInterrupt() {
  AtomicGuard guard;

  DDRB |= kTriggerMask;
  PORTB &= static_cast<uint8_t>(~kTriggerMask);

  DDRB &= static_cast<uint8_t>(~kEchoMask);
  PORTB &= static_cast<uint8_t>(~kEchoMask);

  g_captureState = EchoCaptureState::kIdle;
  g_echoRiseTicks = 0;
  g_echoPulseTicks = 0;
  g_pulsePending = false;
  g_timeoutPending = false;

  PCIFR = _BV(PCIF0);
  PCMSK0 |= _BV(Pins::kUltrasonicEchoPcint);
  PCICR |= _BV(PCIE0);
}

} // namespace

UltrasonicRangeSensor::UltrasonicRangeSensor() {
  ++g_instanceCount;
}

UltrasonicRangeSensor::~UltrasonicRangeSensor() {
  if (g_instanceCount > 0) {
    --g_instanceCount;
  }
}

void UltrasonicRangeSensor::begin() {
  // PCINT0 ISR 状态在文件全局 g_* 中——多实例会互相踩。检查放在 begin() 而不是
  // 构造里：全局对象构造顺序不可控；进 begin() 时 setup() 已完，cli + 死循环
  // 能让硬件调试者立刻发现。
  if (g_instanceCount != 1) {
    cli();
    while (true) {
    }
  }
  beginUltrasonicPinsAndInterrupt();

  // 把"上次 trigger"故意倒推一个完整间隔，让 startMeasurementIfDue() 在首次
  // poll() 时就立刻发起测量，而不是空等一个间隔后才动。
  const uint32_t nowTicks = Timer1MotorPwm::captureTimeTicks();
  lastTriggerTicks_ = nowTicks - RobotConfig::kUltrasonicMeasurementIntervalTimerTicks;
  triggerEndTicks_ = 0;
  triggerHigh_ = false;
  hasDistance_ = false;
  obstaclePresent_ = false;
  distanceMillimeters_ = 0;
  echoMicroseconds_ = 0;
  obstacleSamples_ = 0;
  clearSamples_ = RobotConfig::kObstacleClearSamples;
}

void UltrasonicRangeSensor::restartAfterManeuver() {
  writeTriggerHigh(false);
  cancelEchoCapture();

  // 同 begin()：倒推一个间隔，避障机动结束后立刻重新开始测距。
  const uint32_t nowTicks = Timer1MotorPwm::captureTimeTicks();
  lastTriggerTicks_ = nowTicks - RobotConfig::kUltrasonicMeasurementIntervalTimerTicks;
  triggerEndTicks_ = 0;
  triggerHigh_ = false;
  hasDistance_ = false;
  obstaclePresent_ = false;
  distanceMillimeters_ = 0;
  echoMicroseconds_ = 0;
  obstacleSamples_ = 0;
  clearSamples_ = RobotConfig::kObstacleClearSamples;
}

void UltrasonicRangeSensor::poll() {
  if (!RobotConfig::kObstacleAvoidanceEnabled) {
    return;
  }

  const uint32_t nowTicks = Timer1MotorPwm::captureTimeTicks();
  finishTriggerIfDue(nowTicks);
  handleEchoTimeout(nowTicks);
  consumeEchoResult();
  startMeasurementIfDue(nowTicks);
}

void UltrasonicRangeSensor::startTrigger(const uint32_t nowTicks) {
  if (triggerHigh_ || echoCaptureBusy()) {
    return;
  }

  armEchoCapture();
  writeTriggerHigh(true);
  triggerHigh_ = true;
  triggerEndTicks_ = nowTicks + RobotConfig::kUltrasonicTriggerPulseTimerTicks;
  lastTriggerTicks_ = nowTicks;
}

void UltrasonicRangeSensor::finishTriggerIfDue(const uint32_t nowTicks) {
  if (!triggerHigh_) {
    return;
  }

  if (deadlineReached(nowTicks, triggerEndTicks_)) {
    writeTriggerHigh(false);
    triggerHigh_ = false;
  }
}

void UltrasonicRangeSensor::startMeasurementIfDue(const uint32_t nowTicks) {
  if (triggerHigh_ || echoCaptureBusy()) {
    return;
  }

  if (elapsedAtLeast(nowTicks, lastTriggerTicks_,
                     RobotConfig::kUltrasonicMeasurementIntervalTimerTicks)) {
    startTrigger(nowTicks);
  }
}

void UltrasonicRangeSensor::handleEchoTimeout(const uint32_t nowTicks) {
  if (!echoCaptureBusy()) {
    return;
  }

  if (elapsedAtLeast(nowTicks, lastTriggerTicks_, RobotConfig::kUltrasonicEchoTimeoutTimerTicks)) {
    markEchoTimeout();
  }
}

void UltrasonicRangeSensor::consumeEchoResult() {
  uint32_t pulseTicks = 0;
  bool timedOut = false;
  if (!takeEchoResult(&pulseTicks, &timedOut)) {
    return;
  }

  if (timedOut) {
    hasDistance_ = false;
    updateObstacleLatch(false);
    return;
  }

  echoMicroseconds_ = pulseTicksToMicroseconds(pulseTicks);
  distanceMillimeters_ = pulseTicksToMillimeters(pulseTicks);
  hasDistance_ = distanceMillimeters_ <= RobotConfig::kUltrasonicMaxDistanceMm;
  updateObstacleLatch(hasDistance_ && distanceMillimeters_ <= RobotConfig::kObstacleStopDistanceMm);
}

void UltrasonicRangeSensor::updateObstacleLatch(const bool closeObstacle) {
  if (closeObstacle) {
    if (obstacleSamples_ < 255) {
      ++obstacleSamples_;
    }
    clearSamples_ = 0;
    if (obstacleSamples_ >= RobotConfig::kObstacleConfirmSamples) {
      obstaclePresent_ = true;
    }
    return;
  }

  obstacleSamples_ = 0;
  if (clearSamples_ < 255) {
    ++clearSamples_;
  }
  if (clearSamples_ >= RobotConfig::kObstacleClearSamples) {
    obstaclePresent_ = false;
  }
}

uint16_t UltrasonicRangeSensor::pulseTicksToMicroseconds(const uint32_t pulseTicks) {
  return static_cast<uint16_t>(pulseTicks / RobotConfig::kTimer1TicksPerMicrosecond);
}

uint16_t UltrasonicRangeSensor::pulseTicksToMillimeters(const uint32_t pulseTicks) {
  // HC-SR04: distance_cm = echo_us / 58；tick = 0.5 µs；echo_us = pulseTicks/2；
  // distance_mm = pulseTicks * 5 / 58。+29 (=58/2) = 四舍五入。
  // 上限校验：echo-timeout 38 ms ⇒ pulseTicks ≤ 76000 ⇒ ×5 ≤ 380000，远低于 2^32。
  return static_cast<uint16_t>((pulseTicks * 5UL + 29UL) / 58UL);
}

} // namespace lf

ISR(PCINT0_vect) {
  lf::onEchoPinChangeFromIsr();
}
