#include "Timer1MotorPwm.h"

#include <avr/interrupt.h>

namespace lf {
namespace Timer1MotorPwm {
namespace {

constexpr uint8_t kMotorPortDMask = _BV(Pins::kLeftMotorIbBit) | _BV(Pins::kLeftMotorIaBit);
constexpr uint8_t kMotorPortBMask = _BV(Pins::kRightMotorIbBit) | _BV(Pins::kRightMotorIaBit);
constexpr uint8_t kMaxEdges = 4;
constexpr uint16_t kMinimumEdgeTick = 2;

// 一个 PWM 周期最多只有四路下降沿；同 tick 事件会合并，减轻 ISR 工作量。
struct EdgeEvent {
  uint16_t tick;
  uint8_t portDLowMask;
  uint8_t portBLowMask;
};

struct PreparedFrame {
  uint8_t portDHighMask;
  uint8_t portBHighMask;
  uint8_t edgeCount;
  EdgeEvent edges[kMaxEdges];
};

volatile PreparedFrame g_activeFrame = {};
volatile PreparedFrame g_shadowFrame = {};
volatile bool g_shadowPending = false;
volatile uint8_t g_edgeIndex = 0;
volatile uint8_t g_controlPeriodCounter = 0;
volatile uint8_t g_controlTicksDue = 0;
volatile uint32_t g_timeBaseTicks = 0;

// active 由 ISR 使用，shadow 由主循环提交；只在周期边界换帧。
PreparedFrame makeEmptyFrame() {
  PreparedFrame frame = {};
  frame.portDHighMask = 0;
  frame.portBHighMask = 0;
  frame.edgeCount = 0;
  return frame;
}

void copyFrame(volatile PreparedFrame& destination, const PreparedFrame& source) {
  destination.portDHighMask = source.portDHighMask;
  destination.portBHighMask = source.portBHighMask;
  destination.edgeCount = source.edgeCount;
  for (uint8_t i = 0; i < kMaxEdges; ++i) {
    destination.edges[i].tick = source.edges[i].tick;
    destination.edges[i].portDLowMask = source.edges[i].portDLowMask;
    destination.edges[i].portBLowMask = source.edges[i].portBLowMask;
  }
}

void copyFrame(volatile PreparedFrame& destination, const volatile PreparedFrame& source) {
  destination.portDHighMask = source.portDHighMask;
  destination.portBHighMask = source.portBHighMask;
  destination.edgeCount = source.edgeCount;
  for (uint8_t i = 0; i < kMaxEdges; ++i) {
    destination.edges[i].tick = source.edges[i].tick;
    destination.edges[i].portDLowMask = source.edges[i].portDLowMask;
    destination.edges[i].portBLowMask = source.edges[i].portBLowMask;
  }
}

void insertEdge(PreparedFrame& frame, const uint16_t tick, const uint8_t portDMask,
                const uint8_t portBMask) {
  // duty 映射和排序在主循环完成，ISR 只按已排好的事件表执行。
  for (uint8_t i = 0; i < frame.edgeCount; ++i) {
    if (frame.edges[i].tick == tick) {
      frame.edges[i].portDLowMask |= portDMask;
      frame.edges[i].portBLowMask |= portBMask;
      return;
    }
  }

  if (frame.edgeCount >= kMaxEdges) {
    return;
  }

  uint8_t index = frame.edgeCount;
  while (index > 0 && frame.edges[index - 1].tick > tick) {
    frame.edges[index] = frame.edges[index - 1];
    --index;
  }

  frame.edges[index].tick = tick;
  frame.edges[index].portDLowMask = portDMask;
  frame.edges[index].portBLowMask = portBMask;
  ++frame.edgeCount;
}

void addChannel(PreparedFrame& frame, const uint8_t duty, const uint8_t portDMask,
                const uint8_t portBMask) {
  if (duty == 0) {
    return;
  }

  frame.portDHighMask |= portDMask;
  frame.portBHighMask |= portBMask;

  if (duty >= RobotConfig::kPwmFullScale) {
    return;
  }

  insertEdge(frame, dutyToEdgeTick(duty), portDMask, portBMask);
}

PreparedFrame prepareFrame(const DutyFrame& duty) {
  PreparedFrame frame = makeEmptyFrame();
  addChannel(frame, duty.leftIb, _BV(Pins::kLeftMotorIbBit), 0);
  addChannel(frame, duty.leftIa, _BV(Pins::kLeftMotorIaBit), 0);
  addChannel(frame, duty.rightIb, 0, _BV(Pins::kRightMotorIbBit));
  addChannel(frame, duty.rightIa, 0, _BV(Pins::kRightMotorIaBit));
  return frame;
}

inline void drivePeriodStartOutputs() {
  PORTD = static_cast<uint8_t>((PORTD & static_cast<uint8_t>(~kMotorPortDMask)) |
                               g_activeFrame.portDHighMask);
  PORTB = static_cast<uint8_t>((PORTB & static_cast<uint8_t>(~kMotorPortBMask)) |
                               g_activeFrame.portBHighMask);
}

inline void clearMotorOutputs(const uint8_t portDMask, const uint8_t portBMask) {
  PORTD &= static_cast<uint8_t>(~portDMask);
  PORTB &= static_cast<uint8_t>(~portBMask);
}

void serviceDueEdgesAndScheduleNext() {
  // COMPA ISR 进入时 TCNT1 已经推进；过期边沿立即清掉，避免小 duty 变成整周期高电平。
  uint16_t threshold = TCNT1 + 1;
  if (threshold > RobotConfig::kTimer1PwmTop) {
    threshold = RobotConfig::kTimer1PwmTop;
  }

  uint8_t portDLowMask = 0;
  uint8_t portBLowMask = 0;
  while (g_edgeIndex < g_activeFrame.edgeCount &&
         g_activeFrame.edges[g_edgeIndex].tick <= threshold) {
    portDLowMask |= g_activeFrame.edges[g_edgeIndex].portDLowMask;
    portBLowMask |= g_activeFrame.edges[g_edgeIndex].portBLowMask;
    ++g_edgeIndex;
  }

  clearMotorOutputs(portDLowMask, portBLowMask);

  if (g_edgeIndex < g_activeFrame.edgeCount) {
    OCR1B = g_activeFrame.edges[g_edgeIndex].tick;
    TIFR1 = _BV(OCF1B);
    TIMSK1 |= _BV(OCIE1B);
  } else {
    TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1B));
  }
}

} // namespace

uint16_t dutyToEdgeTick(const uint8_t duty) {
  if (duty == 0) {
    return 0;
  }
  if (duty >= RobotConfig::kPwmFullScale) {
    return RobotConfig::kTimer1PwmTop;
  }

  uint16_t tick = static_cast<uint16_t>(
      (static_cast<uint32_t>(duty) * RobotConfig::kPwmPeriodTicks) / RobotConfig::kPwmFullScale);
  if (tick < kMinimumEdgeTick) {
    tick = kMinimumEdgeTick;
  }
  if (tick > RobotConfig::kTimer1PwmTop) {
    tick = RobotConfig::kTimer1PwmTop;
  }
  return tick;
}

void begin() {
  const PreparedFrame emptyFrame = makeEmptyFrame();

  const uint8_t oldSreg = SREG;
  cli();

  // 启动定时器前先把四个 L9110S 输入设为低电平，默认滑行停转。
  DDRD |= kMotorPortDMask;
  DDRB |= kMotorPortBMask;
  clearMotorOutputs(kMotorPortDMask, kMotorPortBMask);

  copyFrame(g_activeFrame, emptyFrame);
  copyFrame(g_shadowFrame, emptyFrame);
  g_shadowPending = false;
  g_edgeIndex = 0;
  g_controlPeriodCounter = 0;
  g_controlTicksDue = 0;
  g_timeBaseTicks = 0;

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = RobotConfig::kTimer1PwmTop;
  OCR1B = 0;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
  TIMSK1 = _BV(OCIE1A);
  // 只手写 Timer1：CTC 模式、prescaler=8；Timer0/Timer2 保持 Arduino core 原状。
  TCCR1B = _BV(WGM12) | _BV(CS11);

  SREG = oldSreg;
}

void submit(const DutyFrame& duty) {
  const PreparedFrame prepared = prepareFrame(duty);

  const uint8_t oldSreg = SREG;
  cli();
  copyFrame(g_shadowFrame, prepared);
  g_shadowPending = true;
  SREG = oldSreg;
}

void emergencyStop() {
  const PreparedFrame emptyFrame = makeEmptyFrame();

  const uint8_t oldSreg = SREG;
  cli();
  TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1B));
  copyFrame(g_activeFrame, emptyFrame);
  copyFrame(g_shadowFrame, emptyFrame);
  g_shadowPending = false;
  g_edgeIndex = 0;
  clearMotorOutputs(kMotorPortDMask, kMotorPortBMask);
  SREG = oldSreg;
}

uint8_t takeControlTicks() {
  const uint8_t oldSreg = SREG;
  cli();
  const uint8_t ticks = g_controlTicksDue;
  g_controlTicksDue = 0;
  SREG = oldSreg;
  return ticks;
}

uint32_t captureTimeTicksFromIsr() {
  uint32_t base = g_timeBaseTicks;
  const uint16_t counter = TCNT1;

  // If COMPA is pending while interrupts are masked, CTC may already have wrapped
  // TCNT1 before onPeriodCompareIsr() advances the software epoch.
  if ((TIFR1 & _BV(OCF1A)) != 0 && counter < RobotConfig::kTimer1PwmTop) {
    base += RobotConfig::kPwmPeriodTicks;
  }

  return base + counter;
}

uint32_t captureTimeTicks() {
  const uint8_t oldSreg = SREG;
  cli();
  const uint32_t ticks = captureTimeTicksFromIsr();
  SREG = oldSreg;
  return ticks;
}

void onPeriodCompareIsr() {
  g_timeBaseTicks += RobotConfig::kPwmPeriodTicks;

  // 周期 ISR 是唯一换帧点，避免主循环提交时撕裂 PWM 输出。
  if (g_shadowPending) {
    copyFrame(g_activeFrame, g_shadowFrame);
    g_shadowPending = false;
  }

  drivePeriodStartOutputs();

  g_edgeIndex = 0;
  serviceDueEdgesAndScheduleNext();

  ++g_controlPeriodCounter;
  if (g_controlPeriodCounter >= RobotConfig::kControlPeriodsPerTick) {
    g_controlPeriodCounter = 0;
    if (g_controlTicksDue < 255) {
      ++g_controlTicksDue;
    }
  }
}

void onEdgeCompareIsr() {
  serviceDueEdgesAndScheduleNext();
}

} // namespace Timer1MotorPwm
} // namespace lf

ISR(TIMER1_COMPA_vect) {
  lf::Timer1MotorPwm::onPeriodCompareIsr();
}

ISR(TIMER1_COMPB_vect) {
  lf::Timer1MotorPwm::onEdgeCompareIsr();
}
