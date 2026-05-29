# ADR-004: 使用 PCINT 和 Timer1 时间戳实现超声波避障

## Status

Accepted; obstacle response refined by ADR-005

## Date

2026-05-29

## Context

用户要求在当前固定硬件上加入 HR-SR04/HC-SR04 类超声波测距避障，并继续参考 `Car_head.h`。该头文件给出的可用接线是：

- ECHO: D12。
- TRIG: D13。
- 蓝牙 RX 也在 D13，因此超声波和蓝牙不能同时使用。

现有 ADR 约束仍然成立：

- `Pins.h` 是唯一接线事实源。
- 生产控制路径不用 `digitalRead()`、`digitalWrite()`、`analogRead()`、`analogWrite()`。
- Timer1 是唯一手写初始化的 timer；Timer0 和 Timer2 不写。
- Timer1 已承担 4 kHz 电机软件 PWM 和 10 ms 控制 tick。
- ISR 中不做 ADC、PID、Serial、动态分配、长循环或距离换算。

资料核对：

- Arduino UNO 的 `attachInterrupt()` 不覆盖 D12；D12 只能通过普通 GPIO 或 ATmega328P pin-change interrupt 处理。
- Arduino `pulseIn()` 会等待脉冲，最长等待 timeout；不适合放进本项目 10 ms 控制循环。
- HC-SR04 资料给出 10 us TRIG、Echo 脉宽测距、2-400 cm 标称量程、约 38 ms 远端回波/超时和 60 ms 级测量周期建议。

## Decision

加入 `UltrasonicRangeSensor` 模块：

- D13/PB5 作为 TRIG 输出，直接写 `PORTB5`。
- D12/PB4/PCINT4 作为 ECHO 输入，启用 `PCIE0` 与 `PCMSK0.PCINT4`。
- Echo 上升沿和下降沿在 `PCINT0_vect` 中捕获。
- 时间戳来自 `Timer1MotorPwm::captureTimeTicksFromIsr()`，使用当前 Timer1 CTC 时基的 0.5 us 分辨率。
- `Timer1MotorPwm` 新增只读时间戳 API；不改变 PWM 频率、prescaler、control tick 或 edge 调度策略。
- 主循环非阻塞地产生 TRIG 脉冲、处理 Echo timeout、换算距离并维护避障 latch。
- 默认 60 ms 发起一次测距，Echo timeout 38 ms，连续 2 次距离小于等于 200 mm 进入 `ObstacleStop`。ADR-005 后，解除方式不再是等待连续清空，而是短暂停车后执行开环右转避障。

`RobotController` 新增 `ObstacleStop` 状态。避障触发时立即 `motors_.stopNow()`，清空 PID 和失线计数。ADR-005 进一步增加 `ObstacleTurnRight`，避障动作改为固定右转后重新走传感器稳定阶段。

## Rationale

`pulseIn()` 的阻塞模型会把最坏 38 ms Echo 等待带进主循环，超过当前 10 ms control tick 预算，且会让失线、电机 ramp 和避障响应之间相互拖延。

D12 不是 UNO 外部中断脚，不能用 `attachInterrupt()` 得到清晰的官方支持路径。ATmega328P 的 pin-change interrupt 正好覆盖 PB4/PCINT4；PCINT ISR 只捕获边沿和 Timer1 时间戳，工作量很小，符合现有 ISR 预算。

不新增 Timer0/Timer2，避免破坏现有 ADR。复用 Timer1 只读时间戳比新增定时器更小，但要把 Timer1 寄存器读取封装在 `Timer1MotorPwm`，避免控制层或超声波模块自行理解 Timer1 细节。

## Alternatives Considered

### 使用 `pulseIn()`

优点：

- 代码短，常见 Arduino 示例容易理解。

缺点：

- 最坏等待 Echo timeout，直接阻塞控制循环。
- 与本项目控制路径避免高层阻塞 I/O 的方向冲突。

结论：拒绝。

### 使用 `attachInterrupt()`

优点：

- Arduino 官方 API，语义清楚。

缺点：

- UNO 上 D12 不是 `attachInterrupt()` 支持脚。
- 改线到 D2/D3 会冲突现有循迹 EN 或电机输入，且修改 `Pins.h` 功能映射需要重新硬件确认。

结论：拒绝，使用 PCINT4。

### 新增 Timer2 或改用 Timer0 `micros()`

优点：

- 可以独立测距时基，代码容易和 Arduino 示例对齐。

缺点：

- 违反 Timer0/Timer2 边界。
- `micros()` 依赖 Timer0，本项目不把 Timer0 放进控制闭环。

结论：拒绝。

### 同时支持蓝牙和超声波

优点：

- 功能更多。

缺点：

- `Car_head.h` 中蓝牙 RX 与超声波 TRIG 同在 D13。
- SoftwareSerial 还会增加实时干扰和调试复杂度。

结论：拒绝；本次只启用超声波。

## Consequences

正面影响：

- 避障测距不阻塞 10 ms 控制循环。
- 不新增 timer，不写 Timer0/Timer2。
- D12/D13 接线集中在 `Pins.h`，与 `Car_head.h` 对齐。
- 超声波异常或远距离 timeout 不会永久卡住电机控制。

代价：

- `PCINT0_vect` 被本项目占用；未来若 D8-D13 其它 pin-change 功能加入，需要统一分发。
- Timer1 新增只读时间戳职责，后续修改 PWM TOP/prescaler 必须同步测距换算。
- D13 与蓝牙 RX 复用，启用避障时不能同时使用蓝牙。
- HC-SR04/HR-SR04 实物变体仍需硬件验证，尤其是供电、电平、近场和安装角度。

## Verification

- 工具链可用时编译：`arduino-cli compile --fqbn arduino:avr:uno --warnings all .`
- 静态搜索：生产代码不出现 `pulseIn(`、`delay(`、`delayMicroseconds(`。
- 静态搜索：项目代码不写 Timer0/Timer2 寄存器。
- 静态搜索：Timer1 寄存器写入仍集中在 `Timer1MotorPwm.cpp`。
- 硬件阶段：只接超声波模块，使用 20 cm、50 cm、100 cm 静态障碍物确认读数趋势和 200 mm 避障 latch，再接电机电源。

## References

- 技术规范：`docs/line-follower-plan-and-spec.md`
- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
- Arduino UNO Rev3：https://docs.arduino.cc/hardware/uno-rev3/
- Arduino `attachInterrupt()`：https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
- Arduino `pulseIn()`：https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/
- ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- SparkFun HC-SR04 datasheet：https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
