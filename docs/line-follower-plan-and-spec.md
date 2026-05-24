# 黑线循迹小车计划与技术规范

状态：Draft  
日期：2026-05-24  
范围：Arduino UNO 兼容、ATmega328P-AU 教学小车板的源码实现计划、接线规范、验证规范；当前没有硬件，不上传、不实跑。

## 1. 本轮架构结论

本项目专门针对当前教学小车板开发，不追求跨板可移植性。`Car_head.h` 中“符号 -> 引脚 -> 功能”的对应关系保持不变；实现层可以完全针对 ATmega328P/ArduinoCore-avr standard variant 优化。

新的硬约束：

- 手写 Timer 初始化只允许针对 Timer1。
- Timer0 保持 Arduino core 原状，避免破坏 `millis()`、`micros()`、`delay()` 以及 core 对 Timer0 的内部维护。
- Timer2 不碰，避免引入第二套定时器副作用。
- 生产控制路径不调用 `digitalRead()`、`digitalWrite()`、`analogRead()`、`analogWrite()`。
- 电机四路输入 D3/D5/D9/D10 全部由 Timer1 驱动的软件 PWM 输出；不使用 Timer0/Timer2 硬件 PWM，也不使用 Timer1 的 OC1A/OC1B 硬件 PWM。
- Timer1 同时提供电机 PWM 周期和 10 ms 控制 tick；主循环只消费 Timer1 ISR 置位的 tick 标志。
- ADC 生产路径直接操作 `ADMUX`、`ADCSRA`、`ADCL/ADCH`；只读 A0/A1，A6/A7 保留为 ADC-only 硬件事实但首版不用。

首版成功定义：

- 安装 `arduino:avr` core 后能用 `arduino-cli compile --fqbn arduino:avr:uno --warnings all .` 编译。
- `Car_head.h` 不被修改，且只有 `Pins.h` 直接依赖它。
- Timer1 CTC 初始化明确、集中、可审计；Timer0/Timer2 寄存器不被项目代码写入。
- 电机输出有方向抽象、软启动/斜率限制、死区、失线停车、方向切换低电平保护。
- 传感器黑线电平、EN 有效电平、中心模式、ADC 阈值、电机极性全部配置化。
- 静态搜索能证明生产代码没有 Arduino 高层 I/O API、动态分配、`String` 和控制路径阻塞。

## 2. 官方资料与可信度

| 来源 | 用途 | 可信度 |
|---|---|---|
| Arduino UNO Rev3：https://docs.arduino.cc/hardware/uno-rev3/ | UNO 兼容目标、ATmega328P、16 MHz、I/O、PWM、SRAM/Flash、I/O 电流 | Arduino 官方 |
| ArduinoCore-avr `wiring.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c | Arduino core 初始化 Timer0/Timer1/Timer2、ADC prescaler；确认 Timer0 与时间函数关系 | Arduino 官方源码 |
| ArduinoCore-avr `wiring_analog.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c | `analogRead()`、`analogWrite()` 的 AVR 实现；确认为何生产代码不用高层 API | Arduino 官方源码 |
| ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h | A0-A7 常量、D0-D13、PWM pin、Timer channel 映射 | Arduino 官方源码 |
| Arduino Port Manipulation：https://docs.arduino.cc/hacking/software/PortManipulation/ | 直接端口访问的速度收益、可移植性代价、DDR/PORT/PIN 模型 | Arduino 官方 |
| Arduino `micros()` / `millis()`：https://docs.arduino.cc/language-reference/en/functions/time/micros/ 、https://docs.arduino.cc/language-reference/en/functions/time/millis/ | 说明 Timer0 时间函数保留为非控制调试用途 | Arduino 官方 |
| Arduino CLI compile/core install/sketch spec：https://docs.arduino.cc/arduino-cli/commands/arduino-cli_compile/ 、https://docs.arduino.cc/arduino-cli/commands/arduino-cli_core_install/ 、https://docs.arduino.cc/arduino-cli/sketch-specification/ | 编译命令、core 安装、sketch 命名 | Arduino 官方 |
| Microchip ATmega328P：https://www.microchip.com/en-us/product/ATMEGA328P | MCU 官方入口与数据手册入口 | 芯片厂商官方 |
| ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf | Timer1 CTC/Fast PWM、COM/OCR、ADC、PORT、I/O 电气限制 | 芯片厂商官方 |
| LCSC MSKSEMI L9110S-MS：https://www.lcsc.com/product-detail/Motor-Driver-ICs_MSKSEMI-L9110S-MS_C19272815.html | L9110S-MS 料号、封装、2.5-12 V、1.2 A continuous、2.0 A peak 等分销页面参数 | 授权分销/供应商资料；低于厂商官网 |
| JLCPCB L9110S-MS：https://jlcpcb.com/partdetail/MSKSEMI-L9110SMS/C19272815 | 同一 LCSC 料号的装配供应链参数 | 授权制造/供应链资料；低于厂商官网 |

说明：没有找到可直接核验的 MSKSEMI 官网原始 PDF。文档只能把 LCSC/JLCPCB 参数视作“供应链可核验参数”，不能把它们当作本教学板 PCB 的连续散热能力。电机型号、电池规格、板卡电源路径仍必须硬件阶段确认。

## 3. 已确认参数与未知项

### 3.1 已确认

| 项目 | 已确认事实 | 固件约束 |
|---|---|---|
| MCU/板卡目标 | ATmega328P、16 MHz、5 V 逻辑、2 KB SRAM、32 KB Flash、1 KB EEPROM | 代码低资源、无堆、无第三方库 |
| I/O 电流 | Arduino UNO 官方页标注每个 I/O 脚 DC 电流 20 mA | MCU 引脚只驱动逻辑输入/EN，不给电机或模块供电 |
| Timer0 | ArduinoCore-avr 用 Timer0 overflow 维护 `millis()`/`micros()` | 项目代码不写 TCCR0A/B、OCR0A/B、TIMSK0 |
| Timer1 | 16-bit timer，有 CTC、OCR1A/OCR1B、prescaler 1/8/64/256/1024 | 项目唯一手写初始化的 timer |
| Timer2 | ArduinoCore-avr 初始化为 PWM 可用 timer | 项目不用 Timer2，D3 不使用 OC2B |
| 引脚常量 | standard variant 定义 A6/A7 常量，但 `NUM_ANALOG_INPUTS` 为 6，A6/A7 不是普通数字 I/O | A6/A7 只作为 ADC6/ADC7 事实保留，首版不用 |
| L9110S-MS | LCSC 页面标注 SOP-8、2.5-12 V、1.2 A continuous、2.0 A peak | 默认低速、限幅、斜率限制；不把峰值当连续能力 |

### 3.2 未确认

| 未确认项 | 风险 | 处理 |
|---|---|---|
| 循迹传感器型号、供电、电平、EN 极性 | 输入反相或模块未启用 | `SensorBlackLevel`、`SensorEnableActiveLevel` 配置化 |
| 两个传感器安装几何 | 双白/双黑居中不同 | `CenterMode` 配置化 |
| 电机额定电压、堵转电流、减速比 | 驱动过热、电池压降、复位 | 首次点动低 PWM，硬件阶段测温/测压 |
| 电池盒节数和化学类型 | 过压/欠压/供电能力不足 | 固件不假设；接线前实测 |
| 教学板 USB 与电池电源路径 | 反灌或电机干扰上传 | 没有说明时，上传/调试关闭电机电源 |
| L9110S-MS 在该 PCB 上的连续散热能力 | 芯片过热 | 默认 `maxPwm` 保守，记录温升后再提高 |

## 4. 引脚、端口与 Timer 审计

`Car_head.h` 功能对应关系保持不变：

| 功能 | 符号 | Arduino 引脚 | AVR 端口/位 | 原硬件能力 | 首版使用 |
|---|---:|---:|---|---|---|
| 左电机 IB | `IB_LEFT_pin` | D3 | PD3 | OC2B | Timer1 软件 PWM 输出 |
| 左电机 IA | `IA_LEFT_pin` | D5 | PD5 | OC0B | Timer1 软件 PWM 输出 |
| 右电机 IB | `IB_RIGHT_pin` | D9 | PB1 | OC1A | Timer1 软件 PWM 输出，OC1A 断开 |
| 右电机 IA | `IA_RIGHT_pin` | D10 | PB2 | OC1B | Timer1 软件 PWM 输出，OC1B 断开 |
| 左循迹 EN | `L_SENSOR_EN_pin` | D2 | PD2 | 数字 I/O | 直接写 PORTD2 |
| 右循迹 EN | `R_SENSOR_EN_pin` | A5 | PC5 | 数字 I/O/ADC5 | 直接写 PORTC5 |
| 左循迹 OUT | `L_SENSOR_OUT_pin` | A1 | PC1/ADC1 | 数字 I/O/ADC1 | 数字读 PINC1 或直接 ADC1 |
| 右循迹 OUT | `R_SENSOR_OUT_pin` | A0 | PC0/ADC0 | 数字 I/O/ADC0 | 数字读 PINC0 或直接 ADC0 |
| BADGE | `BADGE_pin` | A7 | ADC7 only | ADC-only | 首版不用 |
| 震动 | `VIBRATION_pin` | A6 | ADC6 only | ADC-only | 首版不用 |
| LED2/蓝牙 tx | `LED2_pin`/`tx_pin` | D11 | PB3/OC2A | 共享 | 首版不用 |
| 超声波 TRIG/蓝牙 rx | `TRIG_pin`/`rx_pin` | D13 | PB5 | 共享 | 首版不用 |

结论：

- D3/D5/D9/D10 的原硬件 PWM 分属 Timer2、Timer0、Timer1。为了“尽可能都用 Timer1”且不碰 Timer0/Timer2，首版不用任何硬件 PWM 通道。
- Timer1 的 OC1A/OC1B 输出断开，D9/D10 当普通 PB1/PB2 端口由 Timer1 ISR 软件输出。
- D3/D5 也当普通 PD3/PD5 端口由同一 Timer1 ISR 软件输出。

## 5. Timer1 专用时基设计

### 5.1 模式选择

使用 Timer1 CTC mode，`OCR1A` 作为 PWM 周期 TOP，`OCR1B` 作为本周期下一次下降沿事件。

初始化目标：

```text
TCCR1A = 0                         # OC1A/OC1B disconnected
TCCR1B = 0
TCNT1  = 0
OCR1A  = timer1PwmTop              # default 499
OCR1B  = 0
TIFR1  = OCF1A | OCF1B | TOV1      # clear stale flags
TIMSK1 = OCIE1A                    # period ISR; OCIE1B only when edge queue active
TCCR1B = WGM12 | CS11              # CTC, prescaler 8
```

在 16 MHz / prescaler 8 下：

| 参数 | 值 |
|---|---:|
| Timer1 tick | 0.5 us |
| `timer1PwmTop` | 499 |
| PWM period | `(499 + 1) * 0.5 us = 250 us` |
| PWM carrier | 4 kHz |
| 控制周期 | 40 个 PWM period = 10 ms |

选择 4 kHz 的理由：

- ISR 最坏情况约每 PWM 周期 1 次 period ISR + 4 次 edge ISR，即约 20k ISR/s，上限可控。
- 250 us 周期给 500 个 timer count，映射 8-bit PWM 到 0..499 有足够分辨率。
- 频率高于机械响应，且比 8 kHz/16 kHz 方案更节省 CPU。
- 若实测电机噪声不可接受，再评估 8 kHz；这需要重新预算 ISR 占用。

### 5.2 软件 PWM 调度

主循环提交电机输出时，`Timer1MotorPwm::submit()` 先把四路 duty 转成：

- 周期起始置高 mask。
- 最多 4 个已排序的 falling edge event。
- full-on mask 和 low mask。

转换、映射和排序都在主循环完成，不在 ISR 中完成。

每个 PWM 周期开始：

1. ISR 在周期边界 latch 已提交的 shadow event buffer。
2. 所有 duty 为 0 的输入保持低。
3. 所有 duty 为满量程的输入直接保持高；首版常规配置不使用 255。
4. 其它输入在周期开始置高。
5. 设置 `OCR1B` 为第一个 edge；启用 `OCIE1B`。
6. 每 40 个 PWM 周期置位 `controlTickDue`，主循环执行 PID/状态机。

`OCR1B` compare ISR：

1. 清除当前 edge time 对应的输出位。
2. 调度下一个 edge time。
3. 如果本周期没有更多 edge，禁用 `OCIE1B`。

约束：

- ISR 中只做端口置位/清位、edge 调度、计数和 flag；禁止 ADC、PID、Serial、除法、排序、大循环。
- 主循环更新 duty 使用双缓冲；多字节共享数据用 `cli()`/恢复 `SREG` 保护，或只在关中断窗口做短复制。
- 方向切换必须先进入低电平空档期，至少 1-2 个 PWM 周期后再启用反向输入，避免 L9110S 两输入短时间冲突。
- emergency stop 必须能在关中断短区间内把四路输出 mask 清零，并直接拉低 PORTD3/PORTD5/PORTB1/PORTB2。

### 5.3 Timer 资源边界

允许：

- Timer1 CTC、`OCR1A`、`OCR1B`、`OCIE1A`、`OCIE1B`。
- Timer0 时间函数只作可选调试时间戳，不驱动控制逻辑。

禁止：

- 写 TCCR0A/B、OCR0A/B、TIMSK0。
- 写 TCCR2A/B、OCR2A/B、TIMSK2。
- 使用 `analogWrite()`、Servo 库、Tone 或任何占用 Timer1 的库。
- 在生产控制路径依赖 `micros()` 调度。

## 6. ADC 与传感器输入

默认首版使用数字传感器模式：

- OUT 读数：一次读取 `PINC`，取 PC1/PC0。
- EN 控制：直接写 PORTD2/PORTC5。
- 毛刺过滤：跨 PWM/control tick 的 3 样本多数表决，不在同一 tick 忙等。

模拟模式保留但不调用 `analogRead()`：

```text
ADMUX  = REFS0 | channel
ADCSRA = ADEN | ADPS2 | ADPS1 | ADPS0
ADCSRA |= ADSC
等待 ADSC 清零，带 timeout
先读 ADCL，再读 ADCH
value = (ADCH << 8) | ADCL
```

约束：

- 16 MHz 下 ADC prescaler 128，ADC clock 约 125 kHz。
- 使用 AVcc reference；外部 AREF 需要新 ADR 和接线说明。
- 模拟模式才设置 `DIDR0` 的 ADC0D/ADC1D；数字模式不能关数字输入缓冲。
- ADC 不在 Timer1 ISR 中执行，避免打断 PWM edge 时序。
- ADC 阈值、滞回、黑白校准值来自硬件实测。

## 7. L9110S-MS 电机驱动策略

供应链可核验参数：

- 料号：MSKSEMI L9110S-MS。
- 封装：SOP-8。
- LCSC key attributes：2.5-12 V、maximum continuous current 1.2 A、peak current 2.0 A。

固件保守策略：

- 不把 1.2 A 当作本 PCB 的持续运行保证；散热和电机堵转电流必须硬件阶段确认。
- 默认 `basePwm` 低、`maxPwm` 保守、`rampStepPerControlTick` 小。
- `speed = 0` 时两个输入都低，滑行停转。
- 不使用两输入同时高的刹车模式作为常规控制。
- 每侧任一时刻最多一个输入参与 PWM。
- 方向切换先低电平空档，再启用新方向。

推荐输入策略：

| 目标 | IA | IB |
|---|---|---|
| 停止/滑行 | LOW | LOW |
| 方向 A | PWM | LOW |
| 方向 B | LOW | PWM |
| 刹车 | HIGH | HIGH，首版禁用 |

默认前进输入先按 IB：左 D3、右 D9。硬件点动后通过 `invertLeftMotor`、`invertRightMotor` 或 `forwardInput` 配置修正。

## 8. 控制算法与状态机

### 8.1 控制 tick

Timer1 每 40 个 PWM 周期置位一次 `controlTickDue`。主循环看到 flag 后：

1. 清 flag。
2. 读取传感器。
3. 更新线位估计。
4. 更新状态机。
5. 计算 PID。
6. 混控左右目标速度。
7. 应用 trim、限幅、ramp、方向空档。
8. 提交下一周期 PWM 影子缓冲。

主循环不得补跑历史 tick；如果错过 tick，只记录 missed counter，下一轮使用最新状态。

### 8.2 线位估计

| CenterMode | 左 | 右 | 解释 | 误差 |
|---|---|---|---|---:|
| `BetweenSensors` | 白 | 白 | 可能居中，也可能白底丢线 | 0，标记 ambiguous |
| `BetweenSensors` | 黑 | 白 | 线偏左 | -1000 |
| `BetweenSensors` | 白 | 黑 | 线偏右 | +1000 |
| `BetweenSensors` | 黑 | 黑 | 宽线/交叉/传感器过近 | 0，标记 intersectionLike |
| `OnLine` | 黑 | 黑 | 居中 | 0 |
| `OnLine` | 黑 | 白 | 偏左 | -1000 |
| `OnLine` | 白 | 黑 | 偏右 | +1000 |
| `OnLine` | 白 | 白 | 明确失线候选 | invalid |

双数字传感器无法凭单帧读数解决所有歧义；失线策略必须结合历史误差、持续时间和速度限制。

### 8.3 PID

整数 Q8 位置式 PID：

```text
integral = clamp(integral + error, -integralLimit, +integralLimit)
derivative = error - previousError
raw = kp * error + ki * integral + kd * derivative
correction = clamp(raw / 256, -maxCorrection, +maxCorrection)
```

规则：

- 中间值 `int32_t`。
- `ki` 初始 0。
- 输出饱和时冻结或回退积分。
- 失线、停车、模式切换时 reset/freeze 积分。

### 8.4 状态机

| 状态 | 行为 |
|---|---|
| `Boot` | 端口安全初始化，Timer1 未启动前四路电机低 |
| `TimerReady` | Timer1 CTC/PWM 启动，PWM shadow 全 0 |
| `SensorSettle` | EN 生效后等待传感器稳定 |
| `FollowLine` | PID 差速循迹 |
| `LineLost` | 冻结积分，按最后误差低速搜索；超时停车 |
| `Stopped` | 四路电机输入低，等待复位或未来启动输入 |

当前 `Car_head.h` 没有按键引脚，首版不设计按键启动。

## 9. 代码结构

```text
.
├── line-follower.ino
├── Car_head.h
├── BoardProfile.h              # ATmega328P/UNO 编译期断言，A6/A7 ADC-only
├── Pins.h                      # 唯一包含 Car_head.h 的项目头
├── FastIo.h                    # 固定端口位操作
├── Timer1MotorPwm.h/.cpp       # Timer1 CTC、软件 PWM、control tick flag
├── AdcDriver.h/.cpp            # ADC0/ADC1 直接寄存器读取
├── RobotConfig.h               # 所有配置
├── MotorDriver.h/.cpp          # 有符号速度、方向、ramp、dead-time
├── LineSensors.h/.cpp          # EN/OUT、数字/ADC、极性、滤波
├── LineEstimator.h/.cpp
├── PidController.h/.cpp
├── RobotController.h/.cpp
└── docs/
    ├── line-follower-plan-and-spec.md
    └── decisions/
        ├── ADR-001-line-follower-architecture.md
        ├── ADR-002-direct-register-adc-pwm.md
        └── ADR-003-timer1-motor-timebase.md
```

模块边界：

- 只有 `Timer1MotorPwm` 写 Timer1 寄存器和电机输出 PORT 位。
- 只有 `AdcDriver` 写 ADC 寄存器。
- 只有 `FastIo` 写传感器 EN 和读取 PINC。
- 控制层不直接碰硬件寄存器。
- 所有共享 ISR 数据都由 `Timer1MotorPwm` 提供明确的 `submit()` / `emergencyStop()` 接口。

## 10. 接线规范

### 10.1 总原则

1. 断电接线，USB 和电池都先断开。
2. 电机供电不从 Arduino 5 V 或电脑 USB 取电。
3. MCU GND、电机驱动 GND、传感器 GND 必须共地。
4. 电池极性、驱动 VM/VCC、传感器 VCC/GND 必须查板卡说明或实测确认。
5. 第一次电机测试轮子离地，PWM 从低值点动。
6. 上传/串口调试时，如无板卡电源路径说明，关闭电机电源。

### 10.2 集成教学板检查

| 项 | 要求 |
|---|---|
| 左电机 | 接板上左电机输出端，不接 D3/D5 信号脚 |
| 右电机 | 接板上右电机输出端，不接 D9/D10 信号脚 |
| 左循迹 | 板载或外接到 EN=D2、OUT=A1 |
| 右循迹 | 板载或外接到 EN=A5、OUT=A0 |
| 电池盒 | 接板上电源输入，先确认极性和允许电压 |
| USB-C | 编译上传和低频串口调试；电机调试时轮子离地 |

### 10.3 外接 L9110S-MS 时

| 驱动侧 | 板侧 |
|---|---|
| 左 IA | D5 / `IA_LEFT_pin` |
| 左 IB | D3 / `IB_LEFT_pin` |
| 左 OA/OB | 左电机 |
| 右 IA | D10 / `IA_RIGHT_pin` |
| 右 IB | D9 / `IB_RIGHT_pin` |
| 右 OA/OB | 右电机 |
| VM/VCC | 按 L9110S-MS/模块/教学板说明接电机电源 |
| GND | 与 Arduino GND、电池负极共地 |

### 10.4 外接循迹传感器时

| 传感器侧 | 板侧 |
|---|---|
| 左 OUT | A1 / ADC1 / PC1 |
| 右 OUT | A0 / ADC0 / PC0 |
| 左 EN | D2 / PD2 |
| 右 EN | A5 / PC5 |
| VCC | 按模块资料；不能假设都 5 V tolerant |
| GND | Arduino GND |

## 11. 实现任务

### Phase 1: 编译骨架

- [ ] 新增 `line-follower.ino`、`BoardProfile.h`、`Pins.h`。
  - Acceptance：`Pins.h` 唯一包含 `Car_head.h`；编译期断言 ATmega328P、16 MHz、UNO pin map、A6/A7 ADC-only。
  - Verification：`arduino-cli compile --fqbn arduino:avr:uno --warnings all .`

- [ ] 新增 `RobotConfig.h`。
  - Acceptance：Timer1、PWM、PID、传感器极性、电机极性全部集中配置。
  - Verification：`static_assert` 检查 `timer1PwmTop`、控制周期、PWM 范围。

### Phase 2: Timer1 与底层 I/O

- [ ] 实现 `Timer1MotorPwm`。
  - Acceptance：只写 Timer1；CTC 4 kHz 软件 PWM；40 周期产生控制 tick；四路输出初始低。
  - Verification：静态搜索不写 Timer0/Timer2；host 测试 duty 映射、edge 排序、0/255、emergency stop。

- [ ] 实现 `FastIo` 与 `AdcDriver`。
  - Acceptance：传感器数字模式读 `PINC`；模拟模式直接 ADC0/ADC1；EN 直接 PORT 输出。
  - Verification：不调用 `digitalRead()`、`digitalWrite()`、`analogRead()`。

Checkpoint：底层硬件路径可编译，电机默认安全低电平。

### Phase 3: 电机与传感器抽象

- [ ] 实现 `MotorDriver`。
  - Acceptance：有符号速度、限幅、ramp、方向反转、方向切换 dead-time、失控停车。
  - Verification：host 测试 clamp/ramp/dead-time/submit mask。

- [ ] 实现 `LineSensors`。
  - Acceptance：数字/ADC 两种模式、黑线极性、EN 极性、3 样本多数表决、ADC 阈值/滞回。
  - Verification：host 测试极性和滤波。

### Phase 4: 控制闭环

- [ ] 实现 `LineEstimator`。
  - Acceptance：支持 `BetweenSensors` 与 `OnLine` 映射，输出 valid/ambiguous/intersectionLike。
  - Verification：覆盖所有左右黑白组合。

- [ ] 实现 `PidController` 与 `RobotController`。
  - Acceptance：Timer1 tick 驱动 10 ms 控制循环，整数 PID，状态机，失线搜索/停车。
  - Verification：host 测试 PID、状态迁移、missed tick。

Checkpoint：完整首版可编译；当前阶段不上传。

### Phase 5: 硬件校准

- [ ] 测 A0/A1 黑白电平与 EN 极性。
- [ ] 点动左右电机，确认方向和最低启动 PWM。
- [ ] 观察低速连续运行温升和电池压降。
- [ ] 调 P，再调 D，最后决定是否需要 I。
- [ ] 记录最终配置到文档。

## 12. 验证策略

本机当前状态：

| 命令 | 结果 |
|---|---|
| `arduino-cli version` | `arduino-cli Version: 1.5.0 Commit: Homebrew Date: 2026-05-19T10:00:29Z` |
| `arduino-cli core list` | `No platforms installed.` |

准备：

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
```

编译：

```sh
arduino-cli compile --fqbn arduino:avr:uno --warnings all .
```

静态检查：

- 生产代码不得出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`。
- 生产代码不得写 Timer0/Timer2 寄存器。
- ISR 中不得出现 `Serial`、ADC、PID、动态分配、除法或长循环。
- `Car_head.h` 不修改。

硬件阶段：

- 先传感器，后电机。
- 先单轮点动，后双轮低速。
- 先低 `maxPwm`，记录温升后再提高。
- 上传/调试和电机供电的组合必须按板卡说明；没有说明则电机电源关闭。

## 13. 边界

Always：

- Timer1 是唯一手写初始化的 timer。
- Timer0/Timer2 不写。
- 四路电机 PWM 都由 Timer1 软件 PWM 输出。
- 控制 tick 来自 Timer1，不依赖 `micros()`。
- 硬件未知项配置化。

Ask first：

- 修改 `Car_head.h` 的功能/引脚对应关系。
- 使用 Timer0 或 Timer2。
- 加第三方库。
- 启用 Servo/Tone/SoftwareSerial 等会占用 timer 或实时资源的功能。
- 把上传/通电/实跑作为验收。

Never：

- 从 Arduino 5 V 或 USB 给电机供电。
- 把 L9110S-MS 峰值电流当连续能力。
- 把 A6/A7 当普通数字 I/O。
- 在 ISR 中做串口输出、ADC 阻塞采样或 PID。
- 为了解决编译问题删除 `Car_head.h` 中的既有接线事实。

## 14. 开放问题

1. 循迹传感器型号、供电、电平、EN 极性是什么？
2. 传感器安装几何是 `BetweenSensors` 还是 `OnLine`？
3. 小车电机额定电压、空载电流、堵转电流是多少？
4. 电池盒电压和放电能力是多少？
5. 教学板 USB 与电池同时连接的电源路径是否有说明？
6. 4 kHz 软件 PWM 的电机噪声和低速扭矩是否可接受？如不可接受，再评估 8 kHz 与 ISR 预算。
