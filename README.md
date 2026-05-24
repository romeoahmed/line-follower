# 黑线循迹小车固件

这是一个面向 Arduino UNO 兼容、ATmega328P-AU 教学小车板的黑线循迹固件。项目目标不是通用 Arduino 示例，而是在固定接线、低资源 AVR 环境下实现可审计、低开销、默认安全的循迹控制。

当前仓库包含首版固件、技术规范和 ADR。当前环境没有连接硬件，本项目现阶段只做编译与静态验证；上传、点动和实跑需要在硬件确认后执行。

## 当前状态

- 目标板：Arduino UNO 兼容板，ATmega328P，16 MHz，ArduinoCore-avr standard variant。
- 电机驱动：按 L9110S-MS 双输入模型设计，固件默认保守限幅。
- 传感器：左右双循迹传感器，默认数字模式，A1/A0 读 OUT，D2/A5 控 EN。
- PWM 与控制 tick：Timer1 CTC 软件 PWM，默认 4 kHz；每 40 个 PWM 周期产生 10 ms 控制 tick。
- 生产控制路径：不使用 `digitalRead()`、`digitalWrite()`、`analogRead()`、`analogWrite()`、`String` 或动态分配。
- Timer 边界：只手写 Timer1；Timer0 和 Timer2 保持 Arduino core 原状。

## 快速开始

安装 Arduino CLI 和 AVR core：

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
```

编译：

```sh
arduino-cli compile --fqbn arduino:avr:uno --warnings all .
```

## 项目结构

| 路径 | 作用 |
|---|---|
| `line-follower.ino` | Arduino sketch 入口，只创建并轮询 `RobotController`。 |
| `BoardProfile.h` | 编译期约束 ATmega328P、16 MHz、UNO standard pin map。 |
| `Pins.h` | 唯一接线事实源，保存功能、Arduino 引脚、AVR 端口位和 ADC 通道。 |
| `RobotConfig.h` | Timer1、PWM、PID、传感器极性、电机极性、失线策略等配置。 |
| `FastIo.h` | 固定端口位的传感器 EN/OUT 操作。 |
| `AdcDriver.*` | ADC0/ADC1 直接寄存器读取，模拟模式使用。 |
| `Timer1MotorPwm.*` | Timer1 CTC 初始化、四路软件 PWM、10 ms control tick。 |
| `MotorDriver.*` | L9110S-MS 有符号速度、方向反转、限幅、斜率限制和方向空档。 |
| `LineSensors.*` | 数字/ADC 传感器采样、极性配置、3 样本多数表决。 |
| `LineEstimator.*` | 双传感器黑白状态到离散误差和失线标记的转换。 |
| `PidController.*` | 整数 Q8 定点 PID。 |
| `RobotController.*` | 主状态机、10 ms 控制循环、差速混控、失线搜索和停车。 |
| `docs/line-follower-plan-and-spec.md` | 详细技术规范、接线规范、验证策略和开放问题。 |
| `docs/decisions/` | 架构和定时器/寄存器决策记录。 |

## 接线事实

`Pins.h` 是唯一接线事实源。除非同时更新代码、文档和硬件验证计划，不要在其它文件中散落裸引脚号。

| 功能 | Arduino 引脚 | AVR 端口/位 | 首版用途 |
|---|---:|---|---|
| 左电机 IB | D3 | PD3 | Timer1 软件 PWM |
| 左电机 IA | D5 | PD5 | Timer1 软件 PWM |
| 右电机 IB | D9 | PB1 | Timer1 软件 PWM，OC1A 断开 |
| 右电机 IA | D10 | PB2 | Timer1 软件 PWM，OC1B 断开 |
| 左循迹 EN | D2 | PD2 | 直接写 PORTD2 |
| 右循迹 EN | A5 | PC5 | 直接写 PORTC5 |
| 左循迹 OUT | A1 | PC1/ADC1 | 数字读 PINC1 或 ADC1 |
| 右循迹 OUT | A0 | PC0/ADC0 | 数字读 PINC0 或 ADC0 |

A6/A7 只作为 ADC-only 硬件事实保留，首版不使用，不能当普通数字 I/O。

## 架构要点

控制循环由 Timer1 产生的 10 ms tick 驱动。`loop()` 只调用 `RobotController::poll()`，不会用 `micros()` 补跑历史 tick。若主循环错过多个 tick，只记录 missed counter，并使用最新状态继续控制，避免延迟累积。

Timer1 同时承担电机软件 PWM 和 control tick。`Timer1MotorPwm` 在主循环中把 0-255 duty 转换为周期起始高电平 mask 和最多四个 falling edge event；ISR 只做端口置位/清位、edge 调度和 tick 计数。

电机层只暴露左右有符号速度。`MotorDriver` 负责限幅、ramp、方向反转、同侧单输入 PWM 和方向切换低电平空档，避免 L9110S 两输入瞬间冲突。

传感器默认数字模式，跨控制 tick 做 3 样本多数表决；模拟模式保留 ADC0/ADC1 直接寄存器读取、阈值和滞回，但不调用 `analogRead()`。

## 配置入口

优先改 `RobotConfig.h` 中的配置，不要把参数写进控制逻辑：

- 电机：`kMotorBasePwm`、`kMotorSearchPwm`、`kMotorMaxPwm`、`kMotorRampStepPerControlTick`。
- 方向：`kInvertLeftMotor`、`kInvertRightMotor`、`kLeftForwardUsesIb`、`kRightForwardUsesIb`。
- 传感器：`kSensorMode`、`kSensorEnableActiveLevel`、`kSensorBlackLevel`、`kCenterMode`。
- ADC：`kAdcBlackThreshold`、`kAdcHysteresis`。
- PID：`kPidKpQ8`、`kPidKiQ8`、`kPidKdQ8`、`kPidIntegralLimit`、`kPidMaxCorrection`。
- 失线：`kAmbiguousCenterLimitTicks`、`kLineLostStopTicks`。

修改 `Pins.h` 属于硬件接线变更，需要先确认板卡原理图或实测结果，并同步更新文档和验证计划。

## 硬件安全

- 断电接线，USB 和电池都先断开。
- 电机供电不要从 Arduino 5 V 或电脑 USB 取电。
- MCU GND、电机驱动 GND、传感器 GND 和电池负极必须共地。
- 上传或串口调试时，如果没有板卡电源路径说明，关闭电机电源。
- 第一次电机测试时轮子离地，从低 PWM 单轮点动开始。
- 不要把 L9110S-MS 的峰值电流当连续能力，必须在硬件阶段记录温升和电池压降。
- 没有用户明确要求和硬件安全确认时，不上传、不实跑。

## 首次校准顺序

1. 只接传感器，确认 EN 有效电平、黑线电平和 A0/A1 左右对应关系。
2. 轮子离地，低 PWM 点动左电机和右电机，确认方向。
3. 根据实测结果调整 `kInvertLeftMotor`、`kInvertRightMotor`、`kLeftForwardUsesIb`、`kRightForwardUsesIb`。
4. 测最低启动 PWM、低速连续运行温升和电池压降，再决定是否提高 `kMotorMaxPwm`。
5. 先调 P，再调 D，最后决定是否需要 I；每次调整都记录硬件条件。

## 设计记录

- [技术规范](docs/line-follower-plan-and-spec.md)
- [ADR-001: 黑线循迹小车首版架构](docs/decisions/ADR-001-line-follower-architecture.md)
- [ADR-002: 直接寄存器实现 ADC 与 PWM](docs/decisions/ADR-002-direct-register-adc-pwm.md)
- [ADR-003: 使用 Timer1 作为电机 PWM 与控制 tick 专用时基](docs/decisions/ADR-003-timer1-motor-timebase.md)

## 许可证

见 [LICENSE](LICENSE)。
