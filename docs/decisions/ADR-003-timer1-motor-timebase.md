# ADR-003: 使用 Timer1 作为电机 PWM 与控制 tick 专用时基

## Status

Accepted

## Date

2026-05-24

## Context

当前小车板固定使用 `Car_head.h` 中的电机引脚：

- 左电机：D3/PD3 与 D5/PD5。
- 右电机：D9/PB1 与 D10/PB2。

这些引脚的原生硬件 PWM 分散在 Timer2、Timer0、Timer1 上。如果直接使用 `analogWrite()` 或分散硬件 PWM，会不可避免地依赖 Timer0/Timer2。用户明确要求只手写 Timer1，并尽可能使用 Timer1，同时避免影响 Timer0 相关函数。

项目不追求跨板可移植性，只针对当前 ATmega328P-AU/UNO 兼容开发板。

## Decision

使用 Timer1 CTC 作为唯一手写初始化的 timer：

- Timer0 不写，保留 Arduino core 时间函数。
- Timer2 不写。
- Timer1 设置为 CTC mode，`OCR1A` 为 PWM 周期 TOP，`OCR1B` 为周期内下一次 falling edge。
- D3/D5/D9/D10 全部作为普通 GPIO，由 Timer1 ISR 软件 PWM 输出。
- OC1A/OC1B 硬件输出断开，D9/D10 不使用 Timer1 hardware PWM。
- Timer1 period ISR 每 40 个 PWM 周期置位 10 ms 控制 tick。

默认参数：

- F_CPU = 16 MHz。
- Timer1 prescaler = 8。
- Timer1 tick = 0.5 us。
- `OCR1A = 499`。
- PWM carrier = 4 kHz。
- control tick = 40 PWM periods = 10 ms。

ISR 约束：

- 只做端口置位/清位、edge 调度、计数和 flag。
- 不做 Serial、ADC、PID、动态分配、除法、排序或长循环。
- duty 更新通过双缓冲在 PWM 周期边界提交；duty 到 edge event 的映射和排序在主循环完成，不在 ISR 中完成。
- 方向切换需要至少 1-2 个 PWM 周期的低电平空档。

## Rationale

Timer1 是 ATmega328P 上的 16-bit timer，足够精细地提供电机 PWM 周期、edge 调度和控制 tick。统一四路电机输入到 Timer1 软件 PWM 后，D3/D5 不再依赖 Timer2/Timer0，D9/D10 也不需要占用 OC1A/OC1B hardware PWM。这样整个电机输出时序集中在一个模块中，安全策略和 ISR 预算都更容易审计。

保持 Timer0 不变可以降低破坏 Arduino core 时间函数的风险。保持 Timer2 不变可以避免引入第二个定时器副作用，也给未来可选调试功能留下空间。

4 kHz 是保守默认：最坏约 20k ISR/s，适合 16 MHz AVR 上的短 ISR；如果硬件阶段发现噪声不可接受，再评估 8 kHz，但需要重新计算 CPU 占用。

## Alternatives Considered

### 使用分散硬件 PWM

优点：

- CPU 占用低。
- Arduino 默认 PWM 路径成熟。

缺点：

- D5 依赖 Timer0，D3 依赖 Timer2。
- 与“只手写 Timer1，尽可能使用 Timer1”的要求冲突。
- 电机输出副作用分散，难以统一方向切换空档和失控停车。

结论：拒绝。

### Timer1 hardware PWM 只驱动 D9/D10，D3/D5 软件 PWM

优点：

- 右电机可用硬件 PWM，降低部分 ISR 工作。

缺点：

- 左右电机输出路径不一致。
- Timer1 的 OCR/COM 同时承担 hardware PWM 和软件调度，复杂度上升。
- 安全策略要处理两套输出机制。

结论：拒绝。四路统一软件 PWM 更显式。

### 8 kHz 或更高软件 PWM

优点：

- 电机啸叫可能更低。

缺点：

- ISR 频率更高，主循环和 ADC/PID 余量更少。
- 当前无硬件，无法证明收益。

结论：首版拒绝，保留为硬件阶段调参选项。

## Consequences

正面影响：

- Timer0/Timer2 不受项目影响。
- 四路电机输出时序完全统一。
- 控制 tick 不依赖 `micros()`，减少 Timer0 参与控制闭环。
- emergency stop、方向 dead-time、ramp 都可在统一 PWM 层处理。

代价：

- Timer1 被项目独占，不能使用 Servo 等 Timer1 依赖库。
- 软件 PWM 有 ISR 开销和抖动预算，需要严格限制 ISR 内容。
- PWM 频率、分辨率和 CPU 占用需要硬件阶段验证。

## Verification

- 静态检查：生产代码不写 TCCR0A/B、OCR0A/B、TIMSK0、TCCR2A/B、OCR2A/B、TIMSK2。
- 静态检查：Timer1 寄存器只在 `Timer1MotorPwm` 中写。
- 单元测试：duty 到 edge time 映射、edge 排序、0 duty、full duty、direction dead-time、emergency stop。
- 编译：`arduino-cli compile --fqbn arduino:avr:uno --warnings all .`
- 硬件阶段：示波器或逻辑分析仪确认 4 kHz PWM、D3/D5/D9/D10 波形、10 ms control tick。

## References

- 技术规范：`docs/line-follower-plan-and-spec.md`
- ArduinoCore-avr `wiring.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c
- ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h
- Arduino Port Manipulation：https://docs.arduino.cc/hacking/software/PortManipulation/
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
