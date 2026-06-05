# 遇黑左转小车固件

面向 **Arduino UNO 兼容、ATmega328P-AU 教学小车板** 的「默认直行、遇黑左转」教学小车固件。在固定接线、低资源 AVR 环境下做可审计、低开销、默认安全的行为控制。

不追求跨板可移植性：`Pins.h` 是接线事实源，代码直接依赖 ArduinoCore-avr standard variant 的 UNO 引脚表。

## 功能概述

- **行为**：默认双轮匀速直行；**两个传感器同时**连续 `kEncounterConfirmTicks` 个 tick 见黑（即整条黑线横在车前）才触发一次开环左转，转完回到直行（ADR-012 / Rev 1）。单边偏黑、反光、斑点不触发。
- **电机驱动**：L9110S-MS 双输入；默认高侧刹车 / 反相 PWM 模式，含 ramp、启动死区跳变、方向切换空档、左右补偿。
- **PWM 与控制 tick**：Timer1 CTC 软件 PWM @ 4 kHz；每 40 个 PWM 周期一次 10 ms 控制 tick。Timer0 / Timer2 完全不动。
- **传感器**：A0 / A1 双数字传感器，D2 / A5 控 EN；3 样本多数表决；保留 ADC 模式备用。
- **避障**：D13 TRIG、D12 ECHO（通过 ATmega328P PCINT4 捕获，非阻塞）；连续 2 帧 ≤ 200 mm 触发停车 → 开环右转。优先级最高，但已开始的左转/右转不可被打断。
- **失活保护**：120 ms watchdog + `.init3` 段 MCUSR 捕获；上一帧固件失活（ISR 死循环、UB）即永久进 `kFault`，关 WDT、电机断电（ADR-011）。

## 编译

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno --warnings all .
```

## 项目结构

| 路径 | 作用 |
|---|---|
| `line-follower.ino` | Sketch 入口；构造 `RobotController` 并轮询 `poll()`。 |
| `BoardProfile.h` | 编译期断言：ATmega328P、16 MHz、UNO standard variant。 |
| `BootStatus.*` | `.init3` 段捕获 MCUSR 镜像并 `wdt_disable()`；暴露复位原因给控制层（ADR-011）。 |
| `MathUtils.h` | `constexpr` 共享数学工具：`clampSigned`、`maxOf`、`minOf`。 |
| `AtomicGuard.h` | RAII：SREG save + `cli()` + restore；ISR 临界区只能通过它进入。 |
| `Pins.h` | 接线事实源：功能名 → Arduino 引脚 → AVR 端口位 → ADC 通道。 |
| `RobotConfig.h` | 所有可调参数：Timer1、PWM、分层速度、PID profile、传感器/电机极性、失线、避障。 |
| `FastIo.h` | 头文件库，传感器 EN/OUT 的直接端口操作。 |
| `AdcDriver.*` | 直接寄存器读 ADC0 / ADC1。 |
| `Timer1MotorPwm.*` | Timer1 CTC、四路软件 PWM、控制 tick、0.5 µs 时间戳。 |
| `MotorDriver.*` | 有符号速度、限幅、左右 trim、ramp、启动死区跳变、方向切换空档、驱动模式映射。 |
| `LineSensors.*` | 数字/模拟采样、极性、3 样本多数表决。 |
| `LineEstimator.*` | 双传感器布尔值 → `LineState` enum（互斥的 6 态）+ 离散误差。 |
| `UltrasonicRangeSensor.*` | 非阻塞 TRIG 状态机、PCINT0 ECHO 捕获、距离换算、避障 latch。 |
| `RobotController.*` | 主状态机；10 ms 控制循环；默认直行；遇黑去抖触发开环左转；避障停车与开环右转；watchdog + kFault 永久停车（ADR-011 / ADR-012）。 |
| `docs/line-follower-plan-and-spec.md` | 详细规范、接线、验证策略、开放问题。 |
| `docs/decisions/` | 架构决策记录（ADR-001 至 ADR-010）。 |

## 接线

| 功能 | Arduino 引脚 | AVR 端口位 | 备注 |
|---|---:|---|---|
| 左电机 IB | D3 | PD3 | Timer1 软件 PWM 输出 |
| 左电机 IA | D5 | PD5 | Timer1 软件 PWM 输出 |
| 右电机 IB | D9 | PB1 | Timer1 软件 PWM；OC1A 断开 |
| 右电机 IA | D10 | PB2 | Timer1 软件 PWM；OC1B 断开 |
| 左循迹 EN | D2 | PD2 | 直接写 PORTD2 |
| 右循迹 EN | A5 | PC5 | 直接写 PORTC5 |
| 左循迹 OUT | A1 | PC1 / ADC1 | 数字读 PINC1 或 ADC1 |
| 右循迹 OUT | A0 | PC0 / ADC0 | 数字读 PINC0 或 ADC0 |
| 超声波 ECHO | D12 | PB4 / PCINT4 | PCINT0_vect 捕获回波边沿 |
| 超声波 TRIG | D13 | PB5 | 与板载 LED / SCK / 蓝牙 RX 复用，不并用 |

A6 / A7 是 ADC-only 引脚，首版不使用，也不能当数字 I/O。

## 工作原理

### 时钟与控制循环

Timer1 在 16 MHz / prescaler 8 / TOP=499 下产生 4 kHz PWM（0.5 µs 分辨率，每个周期 250 µs）。COMPA ISR 每个周期开始置位 PWM 高电平、调度第一个下降沿、推进时基；每 40 个周期置位一次控制 tick。

`loop()` 只调用 `RobotController::poll()`。`poll()` 高频轮询超声波状态机，并在控制 tick 就绪时跑一次控制步。如果错过多个 tick，记录 `missedControlTicks_` 但只跑最新一帧——不补跑历史 tick，避免延迟累积。

### 默认直行 + 遇黑左转

控制状态机：

```
kSensorSettle  →  kGoStraight  ──见黑 N 帧──→  kEncounterTurnLeft  ──tick 到点──→ kSensorSettle
                       │
                       └──超声波连续确认障碍──→  kObstacleStop  →  kAvoidanceTurnRight  →  kSensorSettle
```

- 默认 `kGoStraight`：双轮 `kMotorCruisePwm` 匀速直行。
- **两个传感器**（A0 右 + A1 左）**同时**连续 `kEncounterConfirmTicks` 个 control tick 见黑（`LineState::kIntersection`）才触发开环左转（L 反转、R 正转），持续 `kEncounterTurnLeftControlTicks`，完了重回 settle 再到直行。**单边偏黑不触发**——严苛挡误识别（窄白底斑点 / 反光 / 边缘斜扫）。
- 超声波避障优先级最高，但**已开始的转向（左转或右转）不可被打断**——避免动作中途反悔。
- `kEncounterTurnLeft` 的 tick 数与现有 `kObstacleRightTurnControlTicks` 同款逻辑：**开环、按 tick 数计时，不是几何角度保证**。硬件阶段必须按实际目标转角重新标定（见 ADR-005 / ADR-012）。

### 电机驱动

默认模式（`kBrakeHighSideInversePwm`）：方向输入整周期 HIGH，另一输入输出 `255 - duty` 反相 PWM。

| 状态 | 方向输入 | 另一输入 |
|---|---|---|
| 停止 / 滑行 | LOW | LOW |
| 有效驱动段 | HIGH | LOW |
| 刹车段 | HIGH | HIGH |

**重要**：在这种模式下，**有效驱动占空比 = `duty / 255`**——`kMotor*Pwm = 128` 约 50% 驱动，不是 Arduino `analogWrite()` 那种"PWM 高电平就是驱动"的直觉。

`kMotorMinimumEffectivePwm`（默认 90）让 ramp 在跨越 0 时一次性跳到最小有效占空比，再继续按 step 上行，避开电机启动死区。

回退模式 `kCoastLowSidePwm`：只给方向输入输出 PWM、另一输入保持 LOW。刹车模式发热或噪声不可接受可切回。

### 超声波避障

D12 不是 UNO 外部中断脚，用 ATmega328P 的 PCINT4 / `PCINT0_vect` 捕获 ECHO 边沿，时间戳来自 Timer1（不引入 Timer0 或 Timer2 用途）。距离按 HC-SR04 datasheet 的 `distance_cm = echo_us / 58` 换算。

默认 60 ms 一次测距，38 ms ECHO 超时；连续 2 帧 ≤ 200 mm 触发：

```
ObstacleStop（短暂停车）
  → ObstacleTurnRight（开环按 kObstacleRightTurnControlTicks 维持低速反向差速）
  → SensorSettle
```

90° / 左转角度都是开环标定的结果，**不是几何保证**——电池电压、地面摩擦、轮胎都会让它漂移。

## 调参

调参从 `RobotConfig.h` 开始：

- **巡航**：`kMotorCruisePwm`（默认 180，约 71% 有效驱动）。
- **遇黑左转**：`kEncounterTurnLeftPwm`（默认 160）、`kEncounterTurnLeftControlTicks`（默认 55，**初值，需硬件标定**）、`kEncounterConfirmTicks`（默认 5 = 50 ms，连续 N tick **两个传感器都见黑**才触发；1 = 关闭去抖）。
- **电机包络**：`kMotorMaxPwm`（默认 220）、`kMotorDriveMode`、`kMotorRampStepPerControlTick`、`kMotorMinimumEffectivePwm`、`k{Left,Right}MotorTrimPermille`。trim 在 `MotorDriver` 内做左右差分补偿，独立的 clamp 守 trim 放大后的越限。
- **方向**：`kInvert{Left,Right}Motor`、`k{Left,Right}ForwardUsesIb`。
- **传感器**：`kSensorMode`、`kSensorEnableActiveLevel`、`kSensorBlackLevel`、`kCenterMode`、`kAdcBlackThreshold`、`kAdcHysteresis`、`kSensorSettleControlTicks`（必须 ≥ `LineSensors::kHistoryDepth`，已用 `static_assert` 守门）。
- **避障**：`kObstacleAvoidanceEnabled`、`kObstacleStopDistanceMm`、`kObstacleConfirmSamples`、`kObstacleClearSamples`、`kObstacleStopHoldControlTicks`、`kObstacleRightTurnPwm`、`kObstacleRightTurnControlTicks`、`kUltrasonicMeasurementIntervalMs`。

改 `Pins.h` 属于硬件接线变更，必须先核对原理图或实测结果，再同步本 README、`docs/line-follower-plan-and-spec.md` 和相关 ADR。

## 硬件安全

- 接线 / 改线前断电，USB 与电池都拔。
- **电机绝不从 Arduino 5 V 或电脑 USB 取电**。
- MCU、电机驱动、传感器、电池负极必须共地。
- 上传 / 串口调试时，若无板卡电源路径说明，先关电机电源。
- 第一次电机测试**轮子离地**，单轮、低 PWM 点动。
- L9110S-MS 的 1.2 A continuous / 2.0 A peak 是料号上限，不是本 PCB 的散热保证；硬件阶段记录温升和电池压降。

## 首次校准顺序

1. **只接传感器**：确认 EN 有效电平、黑线电平、A0 / A1 左右对应。
2. **只接超声波**：D13 TRIG、D12 ECHO、5 V / GND 共地；静止障碍物核对 20–400 cm 量程内读数趋势。
3. **轮子离地，低 PWM 单轮点动**：确认方向；必要时调 `kInvert{Left,Right}Motor`、`k{Left,Right}ForwardUsesIb`、`kMotorDriveMode`。
4. **测最低启动 PWM、温升与电池压降**：
   - 默认 `kMotorMinimumEffectivePwm = 90` 已开启 → 静止启动时直接跳到该占空比再 ramp，避开 8–90 段死区。
   - 电机在更低占空比已能起转：降低该值减小启动冲击。
   - 电机更迟才起转：优先提高基础速度而不是无脑放大 minimum，避免反向制动过冲。
   - 在 `kBrakeHighSideInversePwm` 模式下，`kMotor*Pwm = 128` ≈ 50% 驱动；`= 255` = 整周期驱动。
   - 本次（ADR-012）巡航 PWM 从 160 → 180，最大限幅从 200 → 220；首跑必须实测 L9110S 温度与电池压降。
5. **标定遇黑左转 + 避障右转**：两者都是开环按 tick 计时，必须分别在地面上标定到接近期望转角；标定方法相同（轮子离地确认方向 → 落地短时测量 → 写回常量）。
6. **去抖 trade-off**：默认 `kEncounterConfirmTicks = 5`（50 ms）已经偏严苛。若发现"压线才转"，先核对 A0 / A1 是否真能稳定双黑（窗口太短 vs 双边覆盖不足是两类问题）；确认双黑稳定后再把窗口调到 3–4 提高响应。窗口设为 1 关闭去抖（不推荐）。

## 设计记录

- [技术规范](docs/line-follower-plan-and-spec.md)
- [ADR-001：首版架构](docs/decisions/ADR-001-line-follower-architecture.md)
- [ADR-002：直接寄存器 ADC 与 PWM](docs/decisions/ADR-002-direct-register-adc-pwm.md)
- [ADR-003：Timer1 作为电机 PWM 与控制 tick 时基](docs/decisions/ADR-003-timer1-motor-timebase.md)
- [ADR-004：PCINT + Timer1 时间戳实现超声波避障](docs/decisions/ADR-004-ultrasonic-obstacle-avoidance.md)
- [ADR-005：分层循迹控制与开环 90° 右转避障](docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md)
- [ADR-006：教师兼容的 L9110S 电机驱动模型](docs/decisions/ADR-006-teacher-compatible-motor-drive-model.md)
- [ADR-007：启动死区跳变与默认 PWM 重新标定](docs/decisions/ADR-007-startup-deadband-and-default-speeds.md)
- [ADR-008：只用 PD 与启用双白超时](docs/decisions/ADR-008-pd-only-and-mandatory-ambiguous-timeout.md)
- [ADR-009：RobotController 状态机清理与模块解耦](docs/decisions/ADR-009-state-machine-cleanup.md)
- [ADR-010：AtomicGuard RAII、LineState enum 与 .cpp-only 工具](docs/decisions/ADR-010-raii-atomic-and-line-state-cleanup.md)
- [ADR-011：Watchdog、kFault、差分保持饱和混控、共享数学工具](docs/decisions/ADR-011-watchdog-fault-state-and-saturation-mix.md)
- [ADR-012：行为切换为「遇黑左转」+ PD 路径全量删除 + 适度提速](docs/decisions/ADR-012-encounter-turn-left-and-pd-removal.md)

## 许可证

见 [LICENSE](LICENSE)。
