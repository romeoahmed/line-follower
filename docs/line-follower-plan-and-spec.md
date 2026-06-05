# 黑线循迹小车计划与技术规范

状态：Draft
日期：2026-05-29
范围：Arduino UNO 兼容、ATmega328P-AU 教学小车板的源码实现计划、接线规范、验证规范；当前没有硬件，不上传、不实跑。

## 1. 本轮架构结论

本项目专门针对当前教学小车板开发，不追求跨板可移植性。`Pins.h` 是唯一接线事实源，保存“功能 -> Arduino 引脚 -> AVR 端口/位/ADC 通道”的对应关系；实现层可以完全针对 ATmega328P/ArduinoCore-avr standard variant 优化。

新的硬约束：

- 手写 Timer 初始化只允许针对 Timer1。
- Timer0 保持 Arduino core 原状，避免破坏 `millis()`、`micros()`、`delay()` 以及 core 对 Timer0 的内部维护。
- Timer2 不碰，避免引入第二套定时器副作用。
- 生产控制路径不调用 `digitalRead()`、`digitalWrite()`、`analogRead()`、`analogWrite()`。
- 电机四路输入 D3/D5/D9/D10 全部由 Timer1 驱动的软件 PWM 输出；不使用 Timer0/Timer2 硬件 PWM，也不使用 Timer1 的 OC1A/OC1B 硬件 PWM。
- Timer1 同时提供电机 PWM 周期和 10 ms 控制 tick；主循环只消费 Timer1 ISR 置位的 tick 标志。
- ADC 生产路径直接操作 `ADMUX`、`ADCSRA`、`ADCL/ADCH`；只读 A0/A1，A6/A7 保留为 ADC-only 硬件事实但首版不用。

当前成功定义：

- 工具链可用时应能用 `arduino-cli compile --fqbn arduino:avr:uno --warnings all .` 编译；当前环境没有 `arduino-cli`，本轮验收只做静态审计。
- 旧接线头文件已删除；`Pins.h` 直接承接原引脚功能映射，且其它模块不散落裸引脚号。
- Timer1 CTC 初始化明确、集中、可审计；Timer0/Timer2 寄存器不被项目代码写入。
- 电机输出有方向抽象、教师兼容高侧刹车/反相 PWM 驱动模式、软启动/斜率限制、可选最低有效 PWM、左右补偿、失线停车、方向切换低电平保护。
- 传感器黑线电平、EN 有效电平、中心模式、ADC 阈值、电机极性、分层速度、PID profile、开环右转避障参数全部配置化。
- 静态搜索能证明生产代码没有 Arduino 高层 I/O API、动态分配、`String` 和控制路径阻塞。

## 2. 官方资料与可信度

| 来源 | 用途 | 可信度 |
|---|---|---|
| Arduino UNO Rev3：https://docs.arduino.cc/hardware/uno-rev3/ | UNO 兼容目标、ATmega328P、16 MHz、I/O、PWM、SRAM/Flash、I/O 电流 | Arduino 官方 |
| ArduinoCore-avr `wiring.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c | Arduino core 初始化 Timer0/Timer1/Timer2、ADC prescaler；确认 Timer0 与时间函数关系 | Arduino 官方源码 |
| ArduinoCore-avr `wiring_analog.c`：https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring_analog.c | `analogRead()`、`analogWrite()` 的 AVR 实现；确认为何生产代码不用高层 API | Arduino 官方源码 |
| ArduinoCore-avr standard pins：https://github.com/arduino/ArduinoCore-avr/blob/master/variants/standard/pins_arduino.h | A0-A7 常量、D0-D13、PWM pin、Timer channel 映射 | Arduino 官方源码 |
| Arduino Port Manipulation：https://docs.arduino.cc/hacking/software/PortManipulation/ | 直接端口访问的速度收益、可移植性代价、DDR/PORT/PIN 模型 | Arduino 官方 |
| Arduino `pulseIn()`：https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/ | 确认 `pulseIn()` 会等待脉冲，避障控制路径不使用它 | Arduino 官方 |
| Arduino `attachInterrupt()`：https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/ | UNO 外部中断脚限制；D12 ECHO 不能用 `attachInterrupt()` | Arduino 官方 |
| Arduino `micros()` / `millis()`：https://docs.arduino.cc/language-reference/en/functions/time/micros/ 、https://docs.arduino.cc/language-reference/en/functions/time/millis/ | 说明 Timer0 时间函数保留为非控制调试用途 | Arduino 官方 |
| Arduino CLI compile/core install/sketch spec：https://docs.arduino.cc/arduino-cli/commands/arduino-cli_compile/ 、https://docs.arduino.cc/arduino-cli/commands/arduino-cli_core_install/ 、https://docs.arduino.cc/arduino-cli/sketch-specification/ | 编译命令、core 安装、sketch 命名 | Arduino 官方 |
| Microchip ATmega328P：https://www.microchip.com/en-us/product/ATMEGA328P | MCU 官方入口与数据手册入口 | 芯片厂商官方 |
| ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf | Timer1 CTC/Fast PWM、COM/OCR、ADC、PORT、I/O 电气限制 | 芯片厂商官方 |
| SparkFun HC-SR04 datasheet：https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf | HC-SR04 5 V 供电、10 us TRIG、Echo 宽度、2-400 cm 量程、约 38 ms 超时、测量周期建议 | 供应商一手资料；高于泛博客，低于原厂芯片数据手册 |
| ISO C++ Core Guidelines：https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines | C++ 接口、类型、安全转换和资源管理最佳实践参考 | ISO C++ 社区官方指南 |
| LCSC MSKSEMI L9110S-MS：https://www.lcsc.com/product-detail/Motor-Driver-ICs_MSKSEMI-L9110S-MS_C19272815.html | L9110S-MS 料号、封装、2.5-12 V、1.2 A continuous、2.0 A peak 等分销页面参数 | 授权分销/供应商资料；低于厂商官网 |
| JLCPCB L9110S-MS：https://jlcpcb.com/partdetail/MSKSEMI-L9110SMS/C19272815 | 同一 LCSC 料号的装配供应链参数 | 授权制造/供应链资料；低于厂商官网 |

说明：没有找到可直接核验的 MSKSEMI 官网原始 PDF。文档只能把 LCSC/JLCPCB 参数视作“供应链可核验参数”，不能把它们当作本教学板 PCB 的连续散热能力。用户写的是 HR-SR04；当前可核验资料主要是 HC-SR04/SR04 模块族资料，本设计按同类 5 V 超声波测距模块处理，实际模块丝印和电气参数仍必须硬件阶段确认。电机型号、电池规格、板卡电源路径仍必须硬件阶段确认。

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
| HC-SR04/SR04 超声波 | SparkFun datasheet 标注 5 V、约 15 mA、2-400 cm、约 15 度测量角、10 us TRIG、Echo 输出与距离成比例 | 默认 60 ms 周期、38 ms Echo 超时、200 mm 避障阈值、连续样本确认后开环右转 |

### 3.2 未确认

| 未确认项 | 风险 | 处理 |
|---|---|---|
| 循迹传感器型号、供电、电平、EN 极性 | 输入反相或模块未启用 | `SensorBlackLevel`、`SensorEnableActiveLevel` 配置化 |
| 两个传感器安装几何 | 双白/双黑居中不同 | `CenterMode` 配置化 |
| 轮距、轮径、减速比和地面摩擦 | 无编码器/陀螺仪时不能闭环保证 90° | `kObstacleRightTurnControlTicks` 只能通过实车标定逼近 90° |
| 电机额定电压、堵转电流、减速比 | 驱动过热、电池压降、复位 | 首次点动低 PWM，硬件阶段测温/测压 |
| 电池盒节数和化学类型 | 过压/欠压/供电能力不足 | 固件不假设；接线前实测 |
| 教学板 USB 与电池电源路径 | 反灌或电机干扰上传 | 没有说明时，上传/调试关闭电机电源 |
| L9110S-MS 在该 PCB 上的连续散热能力 | 芯片过热 | 默认 `maxPwm` 保守，记录温升后再提高 |
| 实物超声波模块是否真为 HC-SR04 兼容 | 命名 HR-SR04 可能是供应商丝印或变体 | 先按 D12/D13 低速静态测距验证，再让车轮离地联调 |

## 4. 引脚、端口与 Timer 审计

`Pins.h` 中的功能对应关系：

| 功能 | 符号 | Arduino 引脚 | AVR 端口/位 | 原硬件能力 | 首版使用 |
|---|---:|---:|---|---|---|
| 左电机 IB | `kLeftMotorIbPin` | D3 | PD3 | OC2B | Timer1 软件 PWM 输出 |
| 左电机 IA | `kLeftMotorIaPin` | D5 | PD5 | OC0B | Timer1 软件 PWM 输出 |
| 右电机 IB | `kRightMotorIbPin` | D9 | PB1 | OC1A | Timer1 软件 PWM 输出，OC1A 断开 |
| 右电机 IA | `kRightMotorIaPin` | D10 | PB2 | OC1B | Timer1 软件 PWM 输出，OC1B 断开 |
| 左循迹 EN | `kLeftSensorEnablePin` | D2 | PD2 | 数字 I/O | 直接写 PORTD2 |
| 右循迹 EN | `kRightSensorEnablePin` | A5 | PC5 | 数字 I/O/ADC5 | 直接写 PORTC5 |
| 左循迹 OUT | `kLeftSensorOutPin` | A1 | PC1/ADC1 | 数字 I/O/ADC1 | 数字读 PINC1 或直接 ADC1 |
| 右循迹 OUT | `kRightSensorOutPin` | A0 | PC0/ADC0 | 数字 I/O/ADC0 | 数字读 PINC0 或直接 ADC0 |
| 超声波 ECHO | `kUltrasonicEchoPin` | D12 | PB4/PCINT4 | 数字 I/O / pin-change interrupt | PCINT0 ISR 捕获回波边沿 |
| 超声波 TRIG | `kUltrasonicTriggerPin` | D13 | PB5/SCK/LED_BUILTIN | 数字 I/O | 直接写 PORTB5，和蓝牙 RX 复用 |
| BADGE | 已排除 | A7 | ADC7 only | ADC-only | 首版不用 |
| 震动 | 已排除 | A6 | ADC6 only | ADC-only | 首版不用 |
| LED2/蓝牙 tx | 已排除 | D11 | PB3/OC2A | 共享 | 首版不用 |
| 蓝牙 rx | 已排除 | D13 | PB5 | 与超声波 TRIG 共享 | 加入超声波后不同时使用蓝牙 |

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

## 7. 超声波避障输入

按 `Car_head.h` 保留的接线加入 HC-SR04/HR-SR04 类超声波模块：

| 模块侧 | 板侧 |
|---|---|
| ECHO | D12 / PB4 / PCINT4 |
| TRIG | D13 / PB5 |
| VCC | 按模块资料，HC-SR04 资料为 5 V |
| GND | Arduino GND，必须与电机电源负极共地 |

参数按可核验 HC-SR04 资料保守配置：

- TRIG 高电平至少 10 us。
- Echo 脉宽与距离成比例，常用换算为 `distance_cm = echo_us / 58`。
- 标称测距范围 2-400 cm；近于 2 cm 的读数只作为“极近障碍”保守处理，不作为精确距离。
- Echo 超时按 38 ms 处理；测量周期默认 60 ms，避免连续触发回波串扰。

实现约束：

- 不使用 `pulseIn()`，避免主循环最长阻塞到 Echo timeout。
- D12 不是 UNO `attachInterrupt()` 支持的外部中断脚，因此直接使用 ATmega328P PCINT0/PB4 捕获 Echo 上升沿和下降沿。
- 时间戳来自 `Timer1MotorPwm::captureTimeTicks()`，分辨率 0.5 us；不新增 Timer0/Timer2 用途。
- PCINT ISR 只读 PINB、捕获 Timer1 时间戳和设置 pending flag；距离换算、确认计数和状态机都在主循环完成。
- 默认连续 2 次测得距离小于等于 200 mm 才进入 `ObstacleStop`；短暂停车后进入 `ObstacleTurnRight`，以低速开环右转约 90°，随后重新进入 `SensorSettle`。

## 8. L9110S-MS 电机驱动策略

供应链可核验参数：

- 料号：MSKSEMI L9110S-MS。
- 封装：SOP-8。
- LCSC key attributes：2.5-12 V、maximum continuous current 1.2 A、peak current 2.0 A。

固件保守策略：

- 不把 1.2 A 当作本 PCB 的持续运行保证；散热和电机堵转电流必须硬件阶段确认。
- 默认直线、弯道、保守和寻线速度分层，`maxPwm` 仍低于满量程，`rampStepPerControlTick` 保守但避免过长时间停在极低占空比。
- 左右电机 trim 和最低有效 PWM 默认关闭，只在实测两侧速度差或启动死区后启用。
- `speed = 0` 时两个输入都低，滑行停转。
- 默认 `kBrakeHighSideInversePwm` 模式接近教师参考代码：方向输入整周期 HIGH，另一输入输出 `255-duty` 反相 PWM；反相输入为 LOW 的时间就是有效驱动占空比，反相输入为 HIGH 时进入 L9110S 双高刹车态。
- 保留 `kCoastLowSidePwm` 作为回退配置：只给方向输入输出 PWM，另一输入保持 LOW。
- 方向切换先低电平空档，再启用新方向。

默认输入策略：

| 目标 | 方向输入 | 另一输入 |
|---|---|---|
| 停止/滑行 | LOW | LOW |
| 方向 A/B 有效驱动段 | HIGH | LOW |
| 方向 A/B 刹车段 | HIGH | HIGH |

默认前进输入按教师参考代码的隐含规则：左轮前进使用 IB/D3，右轮前进使用 IA/D10。硬件点动后通过 `invertLeftMotor`、`invertRightMotor`、`forwardInput` 或 `kMotorDriveMode` 配置修正。

## 9. 控制算法与状态机

ADR-012 之后，项目顶层行为是「默认直行 + 遇黑左转」，**不是**「黑线循迹」。本节
按新行为描述；旧的「分层 profile + PID + 失线搜索」机制已经从代码与配置中删除，
历史动机仍记录在 ADR-005 / ADR-007 / ADR-008 / ADR-011。

### 9.1 控制 tick

Timer1 每 40 个 PWM 周期置位一次 `controlTickDue`。`RobotController::poll()` 看到
flag 后：

1. 喂狗 (`wdt_reset()`)——tick-gated 才能抓 Timer1 ISR 静默失效。
2. 按状态分派：
   - `kStopped` / `kFault`：永久停车，早返回。
   - `kObstacleStop` / `kAvoidanceTurnRight` / `kEncounterTurnLeft`：进行中的
     机动不可被打断，直接跑该状态的 handler。
   - 否则，若超声波 latch 生效 → `transitionTo(kObstacleStop)`。
   - `kSensorSettle`：跑采样让多数表决预热满窗口，电机锁 0，到点转 `kGoStraight`。
   - `kGoStraight`：读传感器 → 估计 → 见黑去抖 → 触发 `kEncounterTurnLeft`，否则
     双轮 `kMotorCruisePwm` 直行。
3. 不补跑历史 tick；错过的 tick 只记到 `missedControlTicks_`（saturating，
   16-bit）。

### 9.2 线位估计

`LineEstimator` 把双传感器布尔值映射到 6 态互斥 `LineState` enum + 离散误差。
新行为下控制层**只消费 state**，不消费 error 数值（仍保留 `kLineErrorUnit` 让
LineEstimator 与未来连续误差源兼容）。

| CenterMode | 左 | 右 | LineState | 控制层判定（ADR-012 Rev 1） |
|---|---|---|---|---|
| `BetweenSensors` | 白 | 白 | `kAmbiguous` | 没见到黑 → 直行 |
| `BetweenSensors` | 黑 | 白 | `kOffsetLeft` | **单边偏黑 → 不触发**，按"没见到黑"处理 |
| `BetweenSensors` | 白 | 黑 | `kOffsetRight` | **单边偏黑 → 不触发**，按"没见到黑"处理 |
| `BetweenSensors` | 黑 | 黑 | `kIntersection` | **双黑 → 进入触发去抖窗口** |
| `OnLine` | 黑 | 黑 | `kCentered` | **双黑 → 进入触发去抖窗口** |
| `OnLine` | 黑 | 白 | `kOffsetLeft` | 单边偏黑 → 不触发 |
| `OnLine` | 白 | 黑 | `kOffsetRight` | 单边偏黑 → 不触发 |
| `OnLine` | 白 | 白 | `kInvalid` | 采样无效 → 直行（默认安全） |

ADR-012 初版把"任一传感器见黑"算作遇黑；Rev 1（同 ADR）严苛收紧到"两个
传感器都见黑"——挡掉细黑线 / 反光 / 斑点 / 边缘斜扫造成的单边偏黑误触发。

ADC 采样失败 → `kInvalid` 一律视为「没见到黑」，继续直行；这是默认安全策略，避免
传感器异常时车在路上自旋。

### 9.3 触发去抖

`runGoStraight` 维护一个连续"双黑"的 tick 计数（复用 `stateTicks_`）：

- 两个传感器同时见黑（`kIntersection` / `kCentered`）：`++stateTicks_`，达到
  `kEncounterConfirmTicks` 即 `transitionTo(kEncounterTurnLeft)`。
- 任何其它读数（单边偏黑、双白、采样无效）：`stateTicks_ = 0`，序列重置。

默认 `kEncounterConfirmTicks = 5`（50 ms 触发延迟）。配合「双黑」严苛触发，
单一帧的反光 / 单边斑点 / 边缘斜扫都不会让计数进展。窗口设为 1 关闭去抖
（不推荐——单帧噪声足以触发）。

### 9.4 开环转向

两个转向 handler 共用同款骨架：

- `kEncounterTurnLeft`：双轮 `(-kEncounterTurnLeftPwm, +kEncounterTurnLeftPwm)`，
  跑 `kEncounterTurnLeftControlTicks` 个 tick，然后 `motors_.stopNow()` + 回
  `kSensorSettle`。**不调** `restartAfterManeuver()`（入口不是 obstacle latch，
  且测距持续运行能在左转期间响应突发障碍）。
- `kAvoidanceTurnRight`：双轮 `(+kObstacleRightTurnPwm, -kObstacleRightTurnPwm)`，
  跑 `kObstacleRightTurnControlTicks` 个 tick，然后 `motors_.stopNow()` +
  `ultrasonic_.restartAfterManeuver()` + 回 `kSensorSettle`。

没有轮速编码器、IMU 或舵机扫描时，固件无法从几何上闭环证明转向角度。当前做法
是把两个转向定义为可标定开环动作：实车用低速原地转向测出接近期望角度的时间，
再写入对应常量。

### 9.5 状态机

| 状态 | 行为 |
|---|---|
| `kSensorSettle` | EN 生效后等传感器多数表决窗口跑满；电机锁 0 |
| `kGoStraight` | 双轮 `kMotorCruisePwm` 匀速直行；两个传感器**同时**连续 `kEncounterConfirmTicks` 见黑（`kIntersection`）触发左转 |
| `kEncounterTurnLeft` | L 反转 / R 正转，按 `kEncounterTurnLeftControlTicks` 开环计时；不可被打断 |
| `kObstacleStop` | 超声波连续确认近距离障碍后立即拉低电机，并保持短暂停车 |
| `kAvoidanceTurnRight` | L 正转 / R 反转，按 `kObstacleRightTurnControlTicks` 开环右转；不可被打断 |
| `kStopped` | 四路电机输入低，永久停车 |
| `kFault` | boot 时 MCUSR.WDRF 触发；关 WDT + 电机断电；MCU 仍跑 loop() 便于调试 |

所有 `state_` 写入仍只走 `transitionTo()`；`stateTicks_` 由它统一清零。
当前接线表没有按键引脚，不设计按键启动。

## 10. 代码结构

```text
.
├── line-follower.ino
├── BoardProfile.h              # ATmega328P/UNO 编译期断言，A6/A7 ADC-only
├── Pins.h                      # 唯一接线事实源
├── FastIo.h                    # 固定端口位操作
├── Timer1MotorPwm.h/.cpp       # Timer1 CTC、软件 PWM、control tick flag
├── AdcDriver.h/.cpp            # ADC0/ADC1 直接寄存器读取
├── RobotConfig.h               # 所有速度、行为、避障、传感器和电机配置
├── MotorDriver.h/.cpp          # 有符号速度、方向、trim、ramp、dead-time
├── LineSensors.h/.cpp          # EN/OUT、数字/ADC、极性、滤波
├── LineEstimator.h/.cpp
├── UltrasonicRangeSensor.h/.cpp # PCINT Echo 捕获、Timer1 时间戳、避障 latch
├── RobotController.h/.cpp      # 直行 + 遇黑左转 + 避障右转（ADR-012）
└── docs/
    ├── line-follower-plan-and-spec.md
    └── decisions/
        ├── ADR-001-line-follower-architecture.md
        ├── ADR-002-direct-register-adc-pwm.md
        ├── ADR-003-timer1-motor-timebase.md
        ├── ADR-004-ultrasonic-obstacle-avoidance.md
        └── ADR-005-layered-control-and-right-turn-obstacle.md
```

模块边界：

- 只有 `Timer1MotorPwm` 写 Timer1 寄存器和电机输出 PORT 位。
- 只有 `AdcDriver` 写 ADC 寄存器。
- 只有 `FastIo` 写传感器 EN 和读取 PINC。
- 只有 `UltrasonicRangeSensor` 写超声波 TRIG/ECHO DDR/PORT 和 PCINT 寄存器。
- 控制层不直接碰硬件寄存器。
- 所有 Timer1 共享 ISR 数据都由 `Timer1MotorPwm` 提供明确的 `submit()` / `emergencyStop()` / `captureTimeTicks()` 接口。

## 11. 接线规范

### 11.1 总原则

1. 断电接线，USB 和电池都先断开。
2. 电机供电不从 Arduino 5 V 或电脑 USB 取电。
3. MCU GND、电机驱动 GND、传感器 GND 必须共地。
4. 电池极性、驱动 VM/VCC、传感器 VCC/GND 必须查板卡说明或实测确认。
5. 第一次电机测试轮子离地，PWM 从低值点动。
6. 上传/串口调试时，如无板卡电源路径说明，关闭电机电源。

### 11.2 集成教学板检查

| 项 | 要求 |
|---|---|
| 左电机 | 接板上左电机输出端，不接 D3/D5 信号脚 |
| 右电机 | 接板上右电机输出端，不接 D9/D10 信号脚 |
| 左循迹 | 板载或外接到 EN=D2、OUT=A1 |
| 右循迹 | 板载或外接到 EN=A5、OUT=A0 |
| 超声波 | 外接到 TRIG=D13、ECHO=D12；D13 与蓝牙 RX 复用，不同时接蓝牙 |
| 电池盒 | 接板上电源输入，先确认极性和允许电压 |
| USB-C | 编译上传和低频串口调试；电机调试时轮子离地 |

### 11.3 外接 L9110S-MS 时

| 驱动侧 | 板侧 |
|---|---|
| 左 IA | D5 / `kLeftMotorIaPin` |
| 左 IB | D3 / `kLeftMotorIbPin` |
| 左 OA/OB | 左电机 |
| 右 IA | D10 / `kRightMotorIaPin` |
| 右 IB | D9 / `kRightMotorIbPin` |
| 右 OA/OB | 右电机 |
| VM/VCC | 按 L9110S-MS/模块/教学板说明接电机电源 |
| GND | 与 Arduino GND、电池负极共地 |

### 11.4 外接循迹传感器时

| 传感器侧 | 板侧 |
|---|---|
| 左 OUT | A1 / ADC1 / PC1 |
| 右 OUT | A0 / ADC0 / PC0 |
| 左 EN | D2 / PD2 |
| 右 EN | A5 / PC5 |
| VCC | 按模块资料；不能假设都 5 V tolerant |
| GND | Arduino GND |

### 11.5 外接超声波模块时

| 超声波侧 | 板侧 |
|---|---|
| TRIG | D13 / PB5 / `kUltrasonicTriggerPin` |
| ECHO | D12 / PB4 / PCINT4 / `kUltrasonicEchoPin` |
| VCC | 按模块资料；HC-SR04 资料为 5 V |
| GND | Arduino GND，与电机电源负极共地 |

注意：

- D13 在 `Car_head.h` 中同时也是蓝牙 `rx_pin`，启用超声波避障时不接蓝牙。
- D13 也是 UNO 板载 LED/SCK；本项目不使用 SPI，板载 LED 负载不作为状态指示使用。
- 第一次验证只接 USB 和超声波模块，不接电机电源，先用静止障碍物确认距离趋势。

## 12. 实现任务

### Phase 1: 编译骨架

- [ ] 新增 `line-follower.ino`、`BoardProfile.h`、`Pins.h`。
  - Acceptance：`Pins.h` 直接保存功能引脚、端口位和 ADC 通道；编译期断言 ATmega328P、16 MHz、UNO pin map、A6/A7 ADC-only。
  - Verification：工具链可用时运行 `arduino-cli compile --fqbn arduino:avr:uno --warnings all .`

- [ ] 新增 `RobotConfig.h`。
  - Acceptance：Timer1、PWM、分层速度、PID profile、传感器极性、电机极性、超声波测距和开环右转避障参数全部集中配置。
  - Verification：`static_assert` 检查 `timer1PwmTop`、控制周期、PWM 范围。

### Phase 2: Timer1 与底层 I/O

- [ ] 实现 `Timer1MotorPwm`。
  - Acceptance：只写 Timer1；CTC 4 kHz 软件 PWM；40 周期产生控制 tick；四路输出初始低。
  - Verification：静态搜索不写 Timer0/Timer2；host 测试 duty 映射、edge 排序、0/255、emergency stop。

- [ ] 实现 `FastIo` 与 `AdcDriver`。
  - Acceptance：传感器数字模式读 `PINC`；模拟模式直接 ADC0/ADC1；EN 直接 PORT 输出。
  - Verification：不调用 `digitalRead()`、`digitalWrite()`、`analogRead()`。

- [ ] 实现 `UltrasonicRangeSensor`。
  - Acceptance：D13 非阻塞 TRIG；D12 PCINT 捕获 Echo；Timer1 时间戳换算距离；连续样本避障 latch；避障机动后可丢弃旧测距结果并重新开始测距周期。
  - Verification：不使用 `pulseIn()`、`delay()`、`delayMicroseconds()`，不写 Timer0/Timer2。

Checkpoint：底层硬件路径可编译，电机默认安全低电平。

### Phase 3: 电机与传感器抽象

- [ ] 实现 `MotorDriver`。
  - Acceptance：有符号速度、可配置驱动模式、限幅、左右 trim、最低有效 PWM、ramp、方向反转、方向切换 dead-time、失控停车。
  - Verification：host 测试 clamp/ramp/dead-time/submit mask。

- [ ] 实现 `LineSensors`。
  - Acceptance：数字/ADC 两种模式、黑线极性、EN 极性、3 样本多数表决、ADC 阈值/滞回。
  - Verification：host 测试极性和滤波。

### Phase 4: 控制闭环

- [ ] 实现 `LineEstimator`。
  - Acceptance：支持 `BetweenSensors` 与 `OnLine` 映射，输出 valid/ambiguous/intersectionLike。
  - Verification：覆盖所有左右黑白组合。

- [ ] 实现 `PidController` 与 `RobotController`。
  - Acceptance：Timer1 tick 驱动 10 ms 控制循环，分层速度，可变 Q8 PID，状态机，失线搜索/停车，超声波确认后开环右转约 90°。
  - Verification：host 测试 PID、状态迁移、missed tick。

Checkpoint：完整首版可编译；当前阶段不上传。

### Phase 5: 硬件校准

- [ ] 测 A0/A1 黑白电平与 EN 极性。
- [ ] 静态障碍物下验证 D12/D13 超声波距离趋势和 200 mm 避障阈值。
- [ ] 点动左右电机，确认方向和最低启动 PWM。
- [ ] 观察低速连续运行温升和电池压降。
- [ ] 标定低速原地右转 90° 所需 control ticks，更新 `kObstacleRightTurnControlTicks`。
- [ ] 根据直线偏航实测决定是否启用左右电机 trim。
- [ ] 调直线 P/D，再调弯道 P/D，最后决定是否需要 I。
- [ ] 记录最终配置到文档。

## 13. 验证策略

本机当前状态：

| 命令 | 结果 |
|---|---|
| `arduino-cli version` | 当前环境没有 `arduino-cli`，本轮不尝试编译 |

后续工具链可用时准备：

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
```

后续工具链可用时编译：

```sh
arduino-cli compile --fqbn arduino:avr:uno --warnings all .
```

静态检查：

- 生产代码不得出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`。
- 生产代码不得出现 `pulseIn(`、`delay(`、`delayMicroseconds(`。
- 生产代码不得写 Timer0/Timer2 寄存器。
- Timer1 寄存器写入应保持集中在 `Timer1MotorPwm.cpp`。
- ISR 中不得出现 `Serial`、ADC、PID、动态分配、除法或长循环。
- `Pins.h` 是唯一接线事实源，生产代码不直接写裸引脚号。
常见但本项目仍需避免的捷径：

- 不用 `attachInterrupt()` 处理 D12；UNO 外部中断脚不是 D12。
- 不用 `micros()` 驱动避障测距；Timer0 保持给 Arduino core 和非控制调试。

硬件阶段：

- 先传感器，后电机。
- 超声波先静态验证 20 cm、50 cm、100 cm 三点读数趋势，再接入电机电源。
- 先单轮点动，后双轮低速。
- 低速原地左转 / 右转都必须轮子离地先确认方向，再落地短时分别标定（左右不一定对称）。
- 先低 `maxPwm`，记录温升后再提高。
- 上传/调试和电机供电的组合必须按板卡说明；没有说明则电机电源关闭。

## 14. 边界

Always：

- Timer1 是唯一手写初始化的 timer。
- Timer0/Timer2 不写。
- 四路电机 PWM 都由 Timer1 软件 PWM 输出。
- 控制 tick 来自 Timer1，不依赖 `micros()`。
- 超声波 Echo 捕获只用 PCINT 和 Timer1 时间戳，不新增 timer。
- 避障右转与遇黑左转**都是**开环标定动作，不得写成“无需硬件校准的精确角度”。
- 硬件未知项配置化。
- Watchdog 在正常路径开启（120 ms）；boot 路径检测到上一次复位由 WDT 触发即
  永久进 `kFault`，关 WDT、电机断电。`wdt_reset()` 只在控制 tick 推进时调用，
  让 Timer1 COMPA ISR 静默失效也能被抓住（见 ADR-011）。
- 已开始的转向机动（左转或右转）不可被任何新事件打断；只有该状态自己的 tick
  计数器把它推回 `kSensorSettle`（见 ADR-012）。

Ask first：

- 修改 `Pins.h` 的功能/引脚对应关系。
- 使用 Timer0 或 Timer2。
- 加第三方库。
- 启用 Servo/Tone/SoftwareSerial 等会占用 timer 或实时资源的功能。
- 同时启用蓝牙和 D13 超声波 TRIG。
- 把上传/通电/实跑作为验收。

Never：

- 从 Arduino 5 V 或 USB 给电机供电。
- 把 L9110S-MS 峰值电流当连续能力。
- 把 A6/A7 当普通数字 I/O。
- 在 ISR 中做串口输出、ADC 阻塞采样或 PID。
- 用 `pulseIn()` 在控制路径等待超声波回波。
- 为了解决编译问题绕过 `Pins.h` 中的接线事实。

## 15. 开放问题

1. 循迹传感器型号、供电、电平、EN 极性是什么？
2. 传感器安装几何是 `BetweenSensors` 还是 `OnLine`？
3. 小车电机额定电压、空载电流、堵转电流是多少？
4. 电池盒电压和放电能力是多少？
5. 教学板 USB 与电池同时连接的电源路径是否有说明？
6. 4 kHz 软件 PWM 的电机噪声和低速扭矩是否可接受？如不可接受，再评估 8 kHz 与 ISR 预算。
7. 实物超声波模块丝印和资料是否确认为 HC-SR04 兼容？D13 板载 LED 负载是否影响 TRIG 边沿？
8. `kObstacleRightTurnPwm` 与 `kObstacleRightTurnControlTicks` 在当前电池电压、地面摩擦和轮胎状态下是否接近 90°？
9. `kEncounterTurnLeftControlTicks` 在当前条件下实际转出多少度？是否需要与右转分别标定？（ADR-012 起的新参数。）
10. 提速后（巡航 180 PWM、最大 220）L9110S 在该 PCB 上的连续散热与电池压降是否仍在安全包络？（ADR-012。）
