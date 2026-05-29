# ADR-006: 教师兼容的 L9110S 电机驱动模型

## Status

Accepted

## Date

2026-05-29

## Context

烧录后电机不动的静态审计发现三个高风险点：

- 当前默认左右轮前进都使用 IB；教师参考代码暗示左轮前进使用 IB、右轮前进使用 IA。
- 当前实现使用单输入 PWM、另一输入 LOW 的滑行模型；教师参考代码使用方向输入 HIGH、另一输入 `255-duty` PWM 的反相刹车模型。
- 默认速度和 ramp 参数偏保守，可能在实车启动死区附近停留过久。

教师代码不能作为绝对正确实现，因为它依赖 Arduino `analogWrite()` 和隐含的板卡电机真值表；但它能暴露该教学板更可能使用的 IA/IB 极性和低速驱动方式。项目仍必须保持 Timer1 软件 PWM、固定引脚、无 Arduino 高层 I/O API、无阻塞控制路径和默认安全低电平。

## Decision

新增 `MotorDriveMode` 配置：

- `kBrakeHighSideInversePwm`：默认模式。方向输入整周期 HIGH，另一输入输出 `255-duty` 反相 PWM；有效驱动发生在反相输入为 LOW 的时间片，反相输入为 HIGH 时为 L9110S 双高刹车态。
- `kCoastLowSidePwm`：回退模式。只给方向输入输出 PWM，另一输入保持 LOW。

默认电机极性改为：

- 左轮前进使用 IB/D3。
- 右轮前进使用 IA/D10。

默认速度提高到仍低于满量程的中低速区间，并把 ramp 步长从 5 提高到 8，减少上电后长时间停留在很小占空比的风险。`speed = 0`、emergency stop 和方向切换空档仍保持 IA/IB 全 LOW，不采用教师代码的全 HIGH 停车。

## Rationale

`analogWrite(255 - speed)` 配合另一路 HIGH，可以用现有 Timer1 软件 PWM 的 falling-edge 输出表达：反相输入在每个周期前段为 HIGH，随后变 LOW；LOW 段长度等于有效驱动占空比。这样既靠近教师参考代码暴露出的板卡行为，又不引入 Timer0/Timer2、Arduino pin API 或硬件 PWM 依赖。

保留 `kCoastLowSidePwm` 是为了硬件验证发现刹车模式发热、噪声或低速表现不合适时能退回旧模型，而不用改控制层。停止和换向空档继续全 LOW，是为了保持上电、停车和方向切换时的最低风险状态。

## Consequences

正面影响：

- 默认右轮方向更贴近教师参考代码的隐含极性。
- 默认 PWM 模型更贴近教学板可能已验证过的低速驱动方式。
- 电机启动不再长期停在过低占空比。
- Timer1 软件 PWM、10 ms 控制 tick、无 Arduino 高层 I/O API 的边界不变。

代价：

- 刹车 PWM 会比滑行 PWM 有更强的电气制动和可能更高的驱动发热，必须实车测温。
- 速度默认值改变后，开环 90°右转时间必须重新标定。
- 如果实物 L9110S 输入极性和教师代码不一致，仍需通过 `kInvert*`、`k*ForwardUsesIb` 或 `kMotorDriveMode` 校正。

## Verification

- 静态搜索：生产代码仍不出现 `digitalRead(`、`digitalWrite(`、`analogRead(`、`analogWrite(`、`pulseIn(`、`delay(`、`delayMicroseconds(`。
- 静态搜索：Timer0/Timer2 寄存器仍不被项目代码写入；Timer1 寄存器写入仍集中在 `Timer1MotorPwm.cpp`。
- 代码审计：`kBrakeHighSideInversePwm` 下正向左轮应输出 IB=255、IA=`255-duty`；正向右轮应输出 IA=255、IB=`255-duty`。
- 硬件阶段：轮子离地逐个点动左右电机，确认方向、启动 PWM、L9110S 温升和电池压降；随后重新标定 `kObstacleRightTurnControlTicks`。

## References

- 技术规范：`docs/line-follower-plan-and-spec.md`
- ADR-003：`docs/decisions/ADR-003-timer1-motor-timebase.md`
- ADR-005：`docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md`
