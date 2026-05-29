# ADR-001: 黑线循迹小车首版架构

## Status

Accepted; refined by ADR-002, ADR-003, ADR-004, and ADR-005

## Date

2026-05-24

## Context

项目目标是在 Arduino UNO 兼容、ATmega328P-AU 教学小车板上实现黑线循迹。当前没有硬件，不能上传或现场调参。

约束：

- `Pins.h` 是唯一接线事实来源；功能、Arduino 引脚、AVR 端口位和 ADC 通道不能漂移。
- 目标优先按 `arduino:avr:uno` 编译；当前 ArduinoCore-avr standard variant 定义 A6/A7 常量，但 A6/A7 不是普通数字 I/O。
- 电机控制脚是 D3/D5/D9/D10，全部具备 PWM 能力。
- 左右循迹输出是 A1/A0，使能是 D2/A5。
- 循迹传感器型号、黑线电平、使能有效电平、电机型号、电池规格未知。
- 用户要求实现基调为现代、显式、干净、节约资源、高性能，并倾向可直接操作寄存器时避免 `digitalWrite()`。

## Decision

首版采用低资源分层架构，并把控制热路径固定到 AVR 寄存器实现：

- `BoardProfile.h` 约束 UNO/ATmega328P-AU 编译目标，并把 A6/A7 明确为 ADC-only 硬件事实；首版不使用 A6/A7。
- `Pins.h` 直接承接原接线事实，向其它模块提供语义化引脚、端口位和 ADC 通道别名。
- `FastIo` 直接操作 DDRx/PORTx/PINx，用于传感器读取和使能控制。
- `Timer1MotorPwm` 手写初始化 Timer1 CTC，用 Timer1 软件 PWM 驱动 D3/D5/D9/D10，并产生控制 tick。
- `AdcDriver` 直接操作 `ADMUX`、`ADCSRA`、`ADCL/ADCH`，生产代码不调用 `analogRead()`。
- `MotorDriver` 封装 L9110S-MS 双输入驱动，外部只传左右有符号速度。
- `LineSensors` 封装 EN、OUT、黑线极性和毛刺过滤。
- `LineEstimator` 将双传感器状态转换为离散误差，并显式标记 ambiguous/intersection/lost。
- `PidController` 使用整数定点 PID。
- `RobotController` 负责消费 Timer1 tick、状态机、差速混控、失线策略和可选低频调试。
- 主循环只消费 `Timer1MotorPwm::takeControlTicks()`，不依赖 `micros()` 驱动控制闭环。

Arduino API 的边界：

- 允许 `micros()` / `millis()` 用于非控制路径的低频调试时间戳。
- 允许 `Serial` 作为编译期开关控制的低频调试。
- 控制路径不使用 `digitalRead()` / `digitalWrite()` / `analogRead()` / `analogWrite()`。
- Timer0/Timer2 不写；Timer1 是唯一手写初始化的 timer。

本 ADR 初始版本排除蓝牙、超声波、摄像头、显示屏、蜂鸣器和其它非循迹核心模块。后续 ADR-004 在固定 D12/D13 接线上加入非阻塞超声波测距；ADR-005 将避障动作定义为开环右转。

## Rationale

项目引脚固定且目标是 AVR UNO 兼容板，传感器和电机的热路径都能用固定端口/寄存器表达。相比每个 tick 走 Arduino 通用 pin API，固定寄存器方案更小、更快，也更容易审计定时器副作用。

同时，完全重配 Timer0/1/2 会增加风险：Timer0 支撑 Arduino 时间函数。根据 ADR-003，首版只手写 Timer1，且不碰 Timer0/Timer2。D3/D5/D9/D10 都由 Timer1 软件 PWM 输出，而不是使用分散在 Timer0/Timer1/Timer2 上的硬件 PWM。

两数字循迹传感器的信息量很低。把传感器、线位估计、PID 和电机层拆开，可以让后续替换为模拟读数或更多传感器时不重写全部控制核心。硬件未知项集中在 `RobotConfig`，避免把猜测散落在控制逻辑里。

## Alternatives Considered

### 只使用 Arduino API

优点：

- 代码更接近入门示例。
- 跨板移植更容易。

缺点：

- `digitalRead()` / `digitalWrite()` 是通用路径，热循环中没有必要。
- 难以显式审计端口、位和定时器通道。
- 不符合本项目对高性能、节约资源和直接寄存器的要求。

结论：拒绝作为首选，仅保留为回退思路。

### 完全手写 Timer 初始化

优点：

- 可以完全控制 PWM 频率和模式。
- 电机左右频率可统一设计。

缺点：

- 容易破坏 Arduino core 的时间函数，尤其是 Timer0。
- 当前没有硬件，无法证明改变 PWM 频率带来实车收益。
- 增加调试复杂度。

结论：首版拒绝。只写 OCR 和 compare output，不改 WGM/预分频。

### 使用浮点 PID

优点：

- 调参表达直观。

缺点：

- AVR 上浮点通常带来额外 Flash 和运行时开销。
- 双数字传感器误差离散，浮点收益有限。

结论：拒绝。使用 `int32_t` 中间值和 Q8 定点增益。

### 引入第三方 PID 或电机库

优点：

- 少写部分代码。

缺点：

- 依赖版本和实现策略不可控。
- 难以保证寄存器、定时器、内存和失线策略符合本项目约束。

结论：拒绝。首版自写小型、可审计模块。

### 同时加入超声波避障、蓝牙或显示

优点：

- 演示功能更多。

缺点：

- 引脚已经存在复用。
- 增加供电、串口、定时器和调试复杂度。
- 会稀释循迹闭环的核心目标。

结论：拒绝。后续作为独立增量重新评估。

## Consequences

正面影响：

- 控制热路径短、显式、可审计。
- ADC、PWM 和端口副作用集中在少数文件。
- 硬件未知项集中配置，便于首次上车校准。
- 无第三方依赖，适合 2 KB SRAM 的 ATmega328P。

代价：

- AVR 寄存器路径降低跨板可移植性。
- 文件数量多于单文件 sketch。
- 对 ArduinoCore-avr standard variant 和 ATmega328P 端口映射有明确绑定。
- 两数字传感器的 PID 效果仍受输入分辨率和安装几何限制。

## Verification

- 工具链可用时编译：`arduino-cli compile --fqbn arduino:avr:uno --warnings all .`
- 静态搜索：生产代码不出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`、`String`、动态分配。
- 逻辑测试：PID、ramp、线位映射、状态机用 host C++ 测试覆盖。
- 硬件阶段：按技术规范完成传感器极性、电机方向、最低 PWM、温升和电池压降校准。

## References

- 技术规范：`docs/line-follower-plan-and-spec.md`
- ADR-002：`docs/decisions/ADR-002-direct-register-adc-pwm.md`
- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
- ADR-004：`docs/decisions/ADR-004-ultrasonic-obstacle-avoidance.md`
- ADR-005：`docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md`
- Arduino UNO Rev3：https://docs.arduino.cc/hardware/uno-rev3/
- ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h
- ArduinoCore-avr wiring_analog：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c
- Arduino analogWrite：https://docs.arduino.cc/language-reference/en/functions/analog-io/analogWrite/
- Arduino micros：https://docs.arduino.cc/language-reference/en/functions/time/micros/
- Arduino CLI compile：https://docs.arduino.cc/arduino-cli/commands/arduino-cli_compile/
- Microchip ATmega328P：https://www.microchip.com/en-us/product/ATMEGA328P
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- L9110S-MS 供应商页：https://www.lcsc.com/product-detail/Motor-Driver-ICs_MSKSEMI-L9110S-MS_C19272815.html
