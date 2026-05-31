#include "Timer1MotorPwm.h"

#include "AtomicGuard.h"

#include <avr/interrupt.h>

namespace lf {
namespace Timer1MotorPwm {
namespace {

constexpr uint8_t kMotorPortDMask = _BV(Pins::kLeftMotorIbBit) | _BV(Pins::kLeftMotorIaBit);
constexpr uint8_t kMotorPortBMask = _BV(Pins::kRightMotorIbBit) | _BV(Pins::kRightMotorIaBit);
constexpr uint8_t kMaxEdges = 4;
// 边沿 tick 下限 = 1 µs（2 × 0.5 µs）：duty=0 在 addChannel 早跳过，这里只挡极小
// 非零 duty 被四舍五入到 0；L9110 也分辨不出亚 µs 脉冲。
constexpr uint16_t kMinimumEdgeTick = 2;

// 一个 PWM 周期最多四路下降沿；同 tick 事件合并以缩短 ISR。
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

// active 由 ISR 消费；shadow 由主循环 submit() 填；只在 COMPA 周期边界换帧，
// 避免主循环写到一半 ISR 触发导致 PWM 撕裂。
volatile PreparedFrame g_activeFrame = {};
volatile PreparedFrame g_shadowFrame = {};
volatile bool g_shadowPending = false;
volatile uint8_t g_edgeIndex = 0;
volatile uint8_t g_controlPeriodCounter = 0;
volatile uint8_t g_controlTicksDue = 0;
volatile uint32_t g_timeBaseTicks = 0;

PreparedFrame makeEmptyFrame() {
  PreparedFrame frame = {};
  frame.portDHighMask = 0;
  frame.portBHighMask = 0;
  frame.edgeCount = 0;
  return frame;
}

// 单模板覆盖 volatile→volatile（ISR 换帧）与 const&→volatile（submit）两路源。
template <typename Source>
void copyFrame(volatile PreparedFrame& destination, const Source& source) {
  destination.portDHighMask = source.portDHighMask;
  destination.portBHighMask = source.portBHighMask;
  destination.edgeCount = source.edgeCount;
  for (uint8_t i = 0; i < kMaxEdges; ++i) {
    destination.edges[i].tick = source.edges[i].tick;
    destination.edges[i].portDLowMask = source.edges[i].portDLowMask;
    destination.edges[i].portBLowMask = source.edges[i].portBLowMask;
  }
}

// 升序插入：duty 映射与排序在主循环完成，ISR 顺序执行已排好的事件表。
void insertEdge(PreparedFrame& frame, const uint16_t tick, const uint8_t portDMask,
                const uint8_t portBMask) {
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
  // duty == 0：完全不进 high mask，也不排边沿（整周期低）。
  // duty == fullscale：进 high mask，不排边沿（整周期高，没有下降沿可排）。
  // 中间 duty 才需要在周期内排一个下降沿。
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
  // +1 保证 edge.tick == TCNT1 时仍算"已过期"立即清掉；否则极小 duty 会被推到下一
  // 周期，整周期保持高电平。
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
    // 顺序敏感：先写 OCR1B，再"写 1 清"OCF1B（消除前一轮残留 flag），最后才打开
    // OCIE1B。任何反序都可能让 COMPB ISR 立刻为陈旧 flag 触发一次。
    OCR1B = g_activeFrame.edges[g_edgeIndex].tick;
    TIFR1 = _BV(OCF1B);
    TIMSK1 |= _BV(OCIE1B);
  } else {
    // 队列空就关 COMPB ISR——下一次有边沿时 submit() 流程会重新打开。
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

  AtomicGuard guard;

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

  // CTC mode 4 (WGM12=1)：TCNT1→OCR1A 清零并触发 COMPA，周期 (OCR1A+1)×0.5 µs。
  // 最后一行 CS11 装 prescaler=8 启动时钟，避免半配置态下 TCNT1 走偏。
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = RobotConfig::kTimer1PwmTop;
  OCR1B = 0;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B) | _BV(TOV1);
  TIMSK1 = _BV(OCIE1A);
  TCCR1B = _BV(WGM12) | _BV(CS11);
}

void submit(const DutyFrame& duty) {
  const PreparedFrame prepared = prepareFrame(duty);

  AtomicGuard guard;
  copyFrame(g_shadowFrame, prepared);
  g_shadowPending = true;
}

void emergencyStop() {
  const PreparedFrame emptyFrame = makeEmptyFrame();

  AtomicGuard guard;
  TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1B));
  copyFrame(g_activeFrame, emptyFrame);
  copyFrame(g_shadowFrame, emptyFrame);
  g_shadowPending = false;
  g_edgeIndex = 0;
  clearMotorOutputs(kMotorPortDMask, kMotorPortBMask);
}

uint8_t takeControlTicks() {
  AtomicGuard guard;
  const uint8_t ticks = g_controlTicksDue;
  g_controlTicksDue = 0;
  return ticks;
}

uint32_t captureTimeTicksFromIsr() {
  uint32_t base = g_timeBaseTicks;
  const uint16_t counter = TCNT1;

  // 关中断窗口内 COMPA 可能已 pending：CTC 已 wrap TCNT1，但 onPeriodCompareIsr()
  // 还没推进软件时基——手动补一个周期，避免时间戳回退。
  if ((TIFR1 & _BV(OCF1A)) != 0 && counter < RobotConfig::kTimer1PwmTop) {
    base += RobotConfig::kPwmPeriodTicks;
  }

  return base + counter;
}

uint32_t captureTimeTicks() {
  AtomicGuard guard;
  return captureTimeTicksFromIsr();
}

void onPeriodCompareIsr() {
  g_timeBaseTicks += RobotConfig::kPwmPeriodTicks;

  // 周期边界是唯一换帧点：保证 active 在整个 PWM 周期内不变。
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
