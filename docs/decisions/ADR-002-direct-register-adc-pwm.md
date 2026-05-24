# ADR-002: 直接寄存器实现 ADC 与 PWM

## Status

Accepted; PWM timer policy superseded by ADR-003

## Date

2026-05-24

## Context

ADR-001 已决定控制热路径使用 AVR 端口寄存器，避免 `digitalRead()` 和 `digitalWrite()`。进一步核对 ArduinoCore-avr 后可以确认：

- `analogRead()` 在 AVR 上会做通用 pin 到 ADC channel 的转换、配置 `ADMUX`、启动转换并等待完成。
- `analogWrite()` 在 AVR 上会做通用 pin 到 timer 的映射，0/255 极值还会走数字输出路径。
- 本项目只需要固定的 A0/A1 ADC 和 D3/D5/D9/D10 PWM，不需要 Arduino API 的通用映射能力。
- `Car_head.h` 中 A6/A7 对应 ADC6/ADC7 硬件事实，但不是普通数字 I/O；首版循迹不使用它们。

## Decision

生产代码不调用 `analogRead()` 或 `analogWrite()`。

ADC：

- 新增 `AdcDriver`，只负责 ADC0/ADC1。
- 使用 `ADMUX` 选择 AVcc reference 和 channel。
- 使用 `ADCSRA` 启用 ADC、设置 prescaler、启动转换。
- 按 ATmega328P 数据手册顺序先读 `ADCL` 再读 `ADCH`。
- 模拟模式下才设置 `DIDR0` 的 ADC0/ADC1 数字输入禁用位；数字传感器模式保持数字输入可用。
- 默认同步有界采样；若未来控制频率提高或模拟读数成为主路径，再评估 ADC interrupt 或跨 tick pipeline。

PWM：

- 本 ADR 保留“生产代码不调用 `analogWrite()`”的结论。
- ADR-003 取代本 ADR 早期的硬件 PWM 思路：D3/D5/D9/D10 不再按各自 OCnx 硬件 PWM 输出，而是全部由 Timer1 CTC 软件 PWM 驱动。

## Rationale

Arduino 官方 API 适合通用 sketch，但本项目的硬件引脚固定，且用户明确要求在可以直接操作寄存器时避免高层 I/O API。直接寄存器方案减少 pin map 查表、分支和极值回退的不透明性，也让 Timer/ADC 副作用集中在少数可审计文件。

保留 Timer0/Timer2 的 Arduino core 初始化仍然是约束。Timer1 例外：根据 ADR-003，项目专用该开发板，允许手写 Timer1 CTC，把电机 PWM 和控制 tick 统一到 Timer1。

## Alternatives Considered

### 继续用 `analogRead()`

优点：

- 简洁，语义直接。
- Arduino 官方文档覆盖充分。

缺点：

- 通用 pin 转换和阻塞等待不可定制。
- 难以明确处理 ADC digital input buffer、通道切换丢弃样本和超时。
- 与直接寄存器的项目基调不一致。

结论：拒绝，改用 `AdcDriver`。

### 继续用 `analogWrite()`

优点：

- 简洁，保留 Arduino 0-255 语义。

缺点：

- 0/255 极值会走数字输出逻辑，行为不集中在 PWM 模块中。
- 难以显式审计每个电机输入对应的 COM/OCR/PORT 状态。
- 与 L9110S-MS 安全停转策略不够显式。

结论：拒绝，改用 ADR-003 的 `Timer1MotorPwm`。

### 重配 Timer0/Timer1/Timer2 统一硬件 PWM 频率

优点：

- 可以统一四路 PWM 频率。

缺点：

- 可能破坏 Arduino core 时间函数或未来库兼容性。
- 当前没有硬件，不值得提前引入风险。

结论：拒绝。ADR-003 只允许手写 Timer1，不允许改 Timer0/Timer2。

## Consequences

正面影响：

- 生产 I/O 路径完全显式，便于审计和测试。
- ADC/PWM 行为不依赖 Arduino API 的通用 pin map。
- 0 duty 和滑行停转策略明确，不依赖 `analogWrite()` 极值行为。

代价：

- 代码绑定 ATmega328P/ArduinoCore-avr standard pin map。
- 模拟模式需要实现和测试 ADC timeout、reference、DIDR0、通道切换策略。
- 若未来更换板卡，必须重写 `BoardProfile`、`AdcDriver` 和 `Timer1MotorPwm`。

## Verification

- 编译：`arduino-cli compile --fqbn arduino:avr:uno --warnings all .`
- 静态搜索：生产代码不出现 `analogRead(`、`analogWrite(`、`digitalRead(`、`digitalWrite(`。
- 单元测试：ADC channel 选择和 10-bit 组合逻辑；PWM 细节见 ADR-003。
- 硬件阶段：A0/A1 读数与临时诊断 sketch 对照；D3/D5/D9/D10 点动确认方向和温升。

## References

- ArduinoCore-avr `wiring_analog.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c
- ArduinoCore-avr `wiring.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c
- ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h
- Arduino Port Manipulation：https://docs.arduino.cc/hacking/software/PortManipulation/
- Arduino analogRead：https://docs.arduino.cc/language-reference/en/functions/analog-io/analogRead/
- Arduino analogWrite：https://docs.arduino.cc/language-reference/en/functions/analog-io/analogWrite/
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
