# ADR-005: 分层循迹控制与开环 90° 右转避障

## Status

Accepted; ki retention and ambiguous-timeout default superseded by ADR-008; control profiles and PD-driven follow superseded by ADR-012 (line-following retired)

## Date

2026-05-29

## Context

当前固件已经有 Timer1 软件 PWM、10 ms 控制 tick、非阻塞超声波测距、整数 PID、传感器滤波、失线搜索和停车策略。新的改进目标是不新增硬件、不启用蓝牙、不触碰 Timer0/Timer2，并把可取的软件建议吸收到一个显式、可标定、低资源的控制结构中。

重要物理边界：

- 只有左右两个循迹传感器，默认数字模式时线位只有离散信息。
- 没有轮速编码器、陀螺仪或罗盘，固件无法闭环测量车身角度。
- L9110S-MS、减速电机、电池电压、轮胎摩擦和地面都会影响同一 PWM 下的实际角速度。
- D13 与蓝牙 RX 复用；本项目启用超声波时不同时启用蓝牙。

因此，“右转 90°”只能定义为低速开环右转动作，并通过实车标定 `kObstacleRightTurnControlTicks` 让结果接近 90°，不能声明为无需校准的精确角度。

## Decision

控制层采用分层 profile，而不是单一固定 PID 覆盖所有情况：

- 直线 profile：较高基础速度、较小 PID 修正上限、较温和 Q8 增益。
- 弯道 profile：较低基础速度、较大修正上限、较强 P/D 增益。
- 保守 profile：用于双白中心候选或交叉/宽线候选，不更新 PID，只低风险直行。
- 失线 profile：冻结 PID，按最后可信误差低速原地搜线，超时停车。

PID 仍使用整数 Q8 定点形式，但 `PidController::update()` 不再从全局读取唯一一组 `Kp/Ki/Kd/maxCorrection`，而是由 `RobotController` 传入当前 profile 的增益和输出限幅。积分限幅、饱和冻结、状态切换 reset 保持不变。

电机层增加默认关闭的补偿能力：

- `kLeftMotorTrimPermille` 和 `kRightMotorTrimPermille` 用于左右电机速度差补偿。
- `kMotorMinimumEffectivePwm` 用于实测存在启动死区时启用最低有效 PWM。
- 这些补偿都在 `MotorDriver` 内部应用，控制层仍只输出左右有符号速度。

避障状态机改为：

1. 超声波连续确认近距离障碍。
2. 进入 `ObstacleStop`，立即四路电机低电平并短暂停车。
3. 进入 `ObstacleTurnRight`，左轮前进、右轮后退，持续 `kObstacleRightTurnControlTicks` 个 10 ms tick。
4. 结束后立即停车，丢弃旧超声波 latch 和 pending echo，重新进入 `SensorSettle`。

## Rationale

两数字循迹传感器不能产生真实连续线位。把控制分成直线、弯道、保守、失线和避障 profile，比把一个固定 PID 强行用于所有场景更容易解释，也更符合输入信息量。这个设计仍保持小型：没有引入第三方库、动态分配或额外 timer。

`BetweenSensors` 模式下的双白读数既可能是线在两个传感器之间，也可能是完全丢线到白底。默认不只凭持续双白判定失线，因为这会让长直线误触发；如果实际赛道没有这种中心双白场景，可以把 `kAmbiguousCenterLimitTicks` 设置为正数启用该启发式。

开环右转是当前硬件下对“右转 90°”最诚实的实现。若未来要几何意义上的 90°，必须新增角度或轮速反馈，或至少提供可验证的轮距、轮径、速度曲线和地面条件。当前约束明确不新增硬件，所以把校准参数暴露出来是比硬编码“90 度”更正确的做法。

## Consequences

正面影响：

- 直线和弯道可分别调速度与 PID，调参不再互相牵制。
- 双白中心候选不再默认 0.5 s 后强行失线，长直线稳定性更好。
- 避障动作符合用户要求的右转 90°方向，同时不引入阻塞测距或新 timer。
- 电机补偿入口集中在配置，不污染控制状态机。

代价：

- 配置项更多，硬件阶段必须记录调参结果。
- 默认 90°转向只是保守初值，实车必须标定。
- 双数字传感器仍无法从根本上区分“居中双白”和“白底丢线”。
- 若电池电压明显变化，开环转向角度也会漂移。

## Verification

- 静态搜索：生产代码仍不出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`、`pulseIn(`、`delay(`、`delayMicroseconds(`。
- 静态搜索：Timer0/Timer2 寄存器不被项目代码写入；Timer1 寄存器写入仍集中在 `Timer1MotorPwm.cpp`。
- 代码审计：`ObstacleStop` 后必须进入 `ObstacleTurnRight`，不能等待障碍清空才能恢复。
- 硬件阶段：轮子离地确认右转方向；落地低速标定 `kObstacleRightTurnControlTicks`；再验证 20 cm 障碍确认、停车、右转、重新循迹。

## References

- 技术规范：`docs/line-follower-plan-and-spec.md`
- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
- ADR-004：`docs/decisions/ADR-004-ultrasonic-obstacle-avoidance.md`
- Arduino UNO Rev3：https://docs.arduino.cc/hardware/uno-rev3/
- Arduino `pulseIn()`：https://docs.arduino.cc/language-reference/en/functions/advanced-io/pulseIn/
- Arduino `attachInterrupt()`：https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- SparkFun HC-SR04 datasheet：https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
