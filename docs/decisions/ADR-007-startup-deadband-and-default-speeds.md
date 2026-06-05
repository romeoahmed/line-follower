# ADR-007: 启动死区跳变与默认 PWM 重新标定

## Status

Accepted; speed defaults superseded by ADR-012 (cruise/max PWM uplifted alongside behavior change)

## Date

2026-05-31

## Context

ADR-006 落实了教师兼容的 `kBrakeHighSideInversePwm` 驱动模型。在该模型下，方向输入整周期 HIGH，另一输入输出 `255-duty` 反相 PWM；L9110 双高输入 = 刹车态，单高单低输入 = 方向驱动态。结论是 **有效驱动时间分数 = `duty / 255`**，不是 Arduino `analogWrite()` 风格下"PWM 高电平就是驱动时间"的直觉。

实际反馈与静态审计发现：

1. 旧默认 `kMotorStraightPwm=92` / `kMotorCurvePwm=78` / `kMotorCautiousPwm=68` 只产生 27-36% 的实际驱动占空比。在 5-6 V 电池下，相当于电机两端平均电压 1.6-2.2 V，刚好落在教学小车减速直流电机的启动死区（典型 1.5-2.5 V 空载，带摩擦后更高）。结果是高摩擦地面上车不动、电机干响。
2. `kMotorMinimumEffectivePwm` 原本只作用于 `applyCompensation()` 的 target，但 `stepMotor → rampToward` 让 `current` 从 0 以 `kMotorRampStepPerControlTick=8` 慢慢爬升。即使把 minimum 设到 100，启动后头 ~12 个 control tick (≈120 ms) 内的 current 仍然落在 8..96 这种死区里，电机仍然干响不动。

## Decision

### 1. 重新标定默认 PWM

| 配置 | 旧值 | 新值 | 新值对应有效驱动占空比 |
|---|---:|---:|---:|
| `kMotorStraightPwm` | 92 | 160 | 63% |
| `kMotorCurvePwm` | 78 | 130 | 51% |
| `kMotorCautiousPwm` | 68 | 110 | 43% |
| `kMotorSearchPwm` | 68 | 140 | 55% |
| `kObstacleRightTurnPwm` | 72 | 140 | 55% |
| `kMotorMaxPwm` | 150 | 200 | 78% |
| `kMotorRampStepPerControlTick` | 8 | 12 | — |
| `kMotorMinimumEffectivePwm` | 0 | 90 | 35% |

新默认仍然显著低于满量程，硬件阶段必须实测电流、温升和电池压降后再决定是否上调。

### 2. 启动死区跳变

`MotorDriver::rampToward()` 在静止→运动方向跨越 0 时直接跳到 `kMotorMinimumEffectivePwm`（受 target 上限约束），随后才按 ramp step 继续上调。反向同理。`kMotorMinimumEffectivePwm = 0` 关闭此机制，与旧版行为兼容。

### 3. 控制路径清理

- `RobotController::poll()` 与 `runControlStep()` 之间的重复 `ultrasonic_.poll()` 调用合并为一次。
- `runControlStep()` 的 switch 移除已经在前置 if 中 `return` 的 `kObstacleStop` / `kObstacleTurnRight` / `kSensorSettle` 分支，去掉 `default`，让编译器能在新增状态时给出未处理警告；不可达状态走 `enterStopped()` 作为最低风险态。

### 4. 周边硬化

- `kUltrasonicTriggerPulseUs` 从 10 提到 12，吸收主循环轮询和 Timer1/PCINT ISR 抢占带来的最坏几 us 偏差。HC-SR04 datasheet 标注最小 10 us。
- `AdcDriver::read()` 通道校验改为显式枚举比较，不再依赖 `kLeftSensorAdcChannel = 1` 这个巧合上限。

## Rationale

PWM 默认值是这台教学小车从"理论可走"变成"在桌面/木地板能走"的关键变量。在没有硬件可实测之前，按 brake 模型下 50-65% 有效驱动占空比作为出厂默认，比 27-36% 的"看起来保守"更接近真实启动条件；同时仍然低于 78% 的 `kMotorMaxPwm`，给后续硬件标定留出向上空间。

启动死区跳变是消除"低占空比无效驱动"的根因修复。把它放在 `MotorDriver::rampToward()` 而不是控制层，是因为只有电机层知道当前 `current` 与 ramp 步长，控制层只输出有符号目标速度；保持职责边界与现有 ADR 一致。

ADR-006 已经允许 `kBrakeHighSideInversePwm` 在实测发热不可接受时退回 `kCoastLowSidePwm`。本次只调整数值和死区跳变，不改驱动模式选项，也不动 Timer1、PWM 频率或 control tick。

## Consequences

正面：

- 默认配置在常见教学桌面/木地板上更可能直接起步成功。
- 启动死区跳变让 ramp 阶段不再浪费 100 ms 在无效驱动占空比上。
- 控制路径少了一次冗余调用，状态机分支更可静态审计。
- TRIG 时长在 ISR 抖动下仍能满足 HC-SR04 datasheet 下限。

代价：

- 启动冲击变大，机械结构需要承受瞬时跳变到 minimum 的电流脉冲。如发现轮胎打滑或负载冲击不可接受，调低 `kMotorMinimumEffectivePwm` 或回到 0，并适当提高基础速度。
- 转向 PWM 提高后，旧的 `kObstacleRightTurnControlTicks = 55` 接近 90° 的标定值大概率不再有效，必须按 ADR-005 描述重新标定。
- 默认运行电流和温升均上升，硬件阶段必须重新观察 L9110S-MS 温度与电池压降。
- 反向制动从静止开始时也会跳到 `-minimum`；如果反向使用场景对冲击敏感（默认只在失线搜索原地旋转时出现），按需调低 minimum。

## Verification

- 静态搜索：生产代码不出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`、`pulseIn(`、`delay(`、`delayMicroseconds(`。
- 静态搜索：Timer0/Timer2 寄存器未被写入；Timer1 寄存器写入仍集中在 `Timer1MotorPwm.cpp`。
- 代码审计：`MotorDriver::rampToward()` 在 `current ≤ 0 && target > 0 && minimum > 0` 时一次性跳到 `min(target, minimum)`；反向对称。
- 代码审计：`runControlStep()` switch 覆盖所有 `RobotState` 枚举值；不可达状态进入 `enterStopped()`。
- 硬件阶段：
  1. 轮子离地，单轮点动，确认在新 `kMotorStraightPwm = 160` 下方向正确、电机噪声可接受。
  2. 低速连续运行 30 s，记录 L9110S-MS 温度与电池压降；若超出可接受范围，下调 `kMotorMaxPwm`。
  3. 落地后在硬质地面和摩擦较大地面分别测启动是否可靠。
  4. 重新标定 `kObstacleRightTurnControlTicks`，因为转向 PWM 已变。

## References

- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
- ADR-005：`docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md`
- ADR-006：`docs/decisions/ADR-006-teacher-compatible-motor-drive-model.md`
- L9110 真值表：MSKSEMI L9110S-MS 分销页面（LCSC/JLCPCB）
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- SparkFun HC-SR04 datasheet：https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
