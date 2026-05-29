#include "UltrasonicRangeSensor.h"

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

bool elapsedAtLeast(const uint32_t nowTicks, const uint32_t sinceTicks,
                    const uint32_t durationTicks) {
  return static_cast<uint32_t>(nowTicks - sinceTicks) >= durationTicks;
}

bool deadlineReached(const uint32_t nowTicks, const uint32_t deadlineTicks) {
  return static_cast<int32_t>(nowTicks - deadlineTicks) >= 0;
}

bool readEchoHigh() {
  return (PINB & kEchoMask) != 0;
}

void writeTriggerHigh(const bool high) {
  const uint8_t oldSreg = SREG;
  cli();
  if (high) {
    PORTB |= kTriggerMask;
  } else {
    PORTB &= static_cast<uint8_t>(~kTriggerMask);
  }
  SREG = oldSreg;
}

void armEchoCapture() {
  const uint8_t oldSreg = SREG;
  cli();
  g_captureState = EchoCaptureState::kWaitingForRise;
  g_echoRiseTicks = 0;
  g_echoPulseTicks = 0;
  g_pulsePending = false;
  g_timeoutPending = false;
  SREG = oldSreg;
}

bool echoCaptureBusy() {
  const uint8_t oldSreg = SREG;
  cli();
  const bool busy = g_captureState != EchoCaptureState::kIdle;
  SREG = oldSreg;
  return busy;
}

void markEchoTimeout() {
  const uint8_t oldSreg = SREG;
  cli();
  if (g_captureState != EchoCaptureState::kIdle) {
    g_captureState = EchoCaptureState::kIdle;
    g_timeoutPending = true;
  }
  SREG = oldSreg;
}

void cancelEchoCapture() {
  const uint8_t oldSreg = SREG;
  cli();
  g_captureState = EchoCaptureState::kIdle;
  g_echoRiseTicks = 0;
  g_echoPulseTicks = 0;
  g_pulsePending = false;
  g_timeoutPending = false;
  SREG = oldSreg;
}

bool takeEchoResult(uint32_t* pulseTicks, bool* timedOut) {
  if (pulseTicks == nullptr || timedOut == nullptr) {
    return false;
  }

  const uint8_t oldSreg = SREG;
  cli();

  bool hasResult = false;
  if (g_pulsePending) {
    *pulseTicks = g_echoPulseTicks;
    *timedOut = false;
    g_pulsePending = false;
    hasResult = true;
  } else if (g_timeoutPending) {
    *pulseTicks = 0;
    *timedOut = true;
    g_timeoutPending = false;
    hasResult = true;
  }

  SREG = oldSreg;
  return hasResult;
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
  const uint8_t oldSreg = SREG;
  cli();

  DDRB |= kTriggerMask;
  writeTriggerHigh(false);

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

  SREG = oldSreg;
}

} // namespace

void UltrasonicRangeSensor::begin() {
  beginUltrasonicPinsAndInterrupt();

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
  // HC-SR04 family: distance in cm is echo_us / 58. Timer1 tick is 0.5 us.
  return static_cast<uint16_t>((pulseTicks * 5UL + 29UL) / 58UL);
}

} // namespace lf

ISR(PCINT0_vect) {
  lf::onEchoPinChangeFromIsr();
}
