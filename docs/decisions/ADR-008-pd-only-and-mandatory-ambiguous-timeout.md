# ADR-008: 只用 PD 与启用双白超时失线

## Status

Accepted; superseded by ADR-012 (line-following retired; PD entirely removed)

## Date

2026-05-31

## Context

ADR-005 在确立分层控制时，把以下两点都按"保留可能性，默认关闭"处理：

1. PID 中保留 `ki`、`integral_` 状态、`kPidIntegralLimit` 与 `allowIntegral` 参数，
   但所有 profile 的 `ki = 0`。
2. `kAmbiguousCenterLimitTicks = 0`，关闭"双白超时即失线"机制；理由是担心长直线上
   居中行驶时双白持续被误判为失线。

实际审计发现这两点都不是"可选优化"，而是会带来真实问题的默认配置：

### I 项的问题

- 误差是 `{-kLineErrorUnit, 0, +kLineErrorUnit}` 三态离散值，I 在这种信号上没有
  正常 PID 中"吸收稳态小残差"的物理意义。
- 左右电机的稳态偏置已经由 `k{Left,Right}MotorTrimPermille` 在 `MotorDriver` 层
  直接补偿，I 想做的事和它重复。
- 离散误差下 I 还在累计："车偏左持续 N 个 tick → 积分越来越大"——但对应的修正动作
  P 项早就给了，I 反而会过冲、震荡。
- `kPidIntegralLimit = 4000` 与 `kLineErrorUnit = 1000` 配合，意味着只要偏差持续
  4 个 tick 就被夹到上限，根本不是一个合理的 anti-windup 限值。
- 保留为死代码导致两类长期成本：阅读时怀疑 I 是不是在悄悄工作；以后真要开 I 时，
  人会按"取消注释"思路把这套数值直接用上。

### 双白超时的问题

- `LineEstimator` 在 `BetweenSensors` 模式下从不会因为双白把 `lost` 置位；
  `RobotController::profileForEstimate` 在双白时按 `kMotorCautiousPwm` 直行。
- 结果：默认配置下，**小车一旦驶出轨道（变成双白），不会进入失线状态**——会按
  cautious 速度一直直行，直到撞墙、撞到障碍触发避障，或某个传感器再次看到黑线。
- 这违反"默认安全"的项目立场。
- ADR-005 担心"长直线误触发"的论据在当前代码里其实已经被打掉：触发条件包含
  `lastError != 0`，长直线居中场景 `lastError` 一直是 0，永远不会触发。

## Decision

### 1. 删除 I 项的全部基础设施

- `PidController` 重命名为 `PdController`（含文件名 `PdController.h/.cpp`）。
- 删除：`PdGainsQ8::ki` 字段、`integral_` 状态、`allowIntegral` 参数、
  `kPidIntegralLimit` 常量、`clamp32` 辅助函数、anti-windup 分支。
- `RobotConfig.h` 中 `PidGainsQ8 → PdGainsQ8`，`kPidStraightGainsQ8 →
  kPdStraightGainsQ8`，`kPidCurveGainsQ8 → kPdCurveGainsQ8`，
  `kPid*MaxCorrection → kPd*MaxCorrection`，三组常量去掉 ki。
- `RobotController::FollowProfile` 删除 `allowIntegral`，`usePid → usePd`，
  成员 `pid_ → pd_`。

未来若引入连续误差源（如 ADC 模拟传感器加权、IMU、编码器）需要 I，开新 ADR 加新参数、
新类、新测试，而不是在 `PdController` 里加回 ki。

### 2. 默认启用双白超时失线

- `kAmbiguousCenterLimitTicks` 从 0 改到 80（≈ 0.8 s）。
- `ambiguousCenterTimedOut()` 保留 `lastError != 0` 守门，确保长直线居中不被误判。
- 这是默认安全行为，不再是可选启发式。`= 0` 仍可关闭，但配置注释明确标注不推荐。

### 3. 文档同步

- README、AGENTS.md、技术规范、ADR-005 References 更新。
- ADR-005 `Status:` 从 `Accepted` 改为 `Accepted; ki and ambiguous timeout
  policy superseded by ADR-008`。

## Rationale

### 为什么删，而不是只 gate

只在 `PdController` 里加一行 `if (gains.ki == 0)` 跳过累加，能消除 CPU 浪费，
但不能解决：

- 阅读混乱（接口仍叫 PID、仍接收 `allowIntegral`）。
- `kPidIntegralLimit = 4000` 这个错的数还在配置文件里，看起来像"已经调过的值"。
- 未来开 I 的人仍会沿用旧 API 形状和旧限值。

干净的删除把"以后再决定"这件事真正推迟到未来：未来的实现者面对的是一份明确的
PD 实现，要加 I 必须显式扩展接口、显式新加配置、显式写测试。这比"取消一行注释就
启用一份未经核对的旧代码"安全得多。

把控制器叫 PdController 也是诚实化：本项目不是"暂时禁用了 I 的 PID"，就是 PD。

### 为什么 80 tick（0.8 s）

PID 在 ±1000 离散误差下的典型矫正时间（含 3 样本表决、机械响应、ramp）通常在
30-50 tick。80 给出 30+ tick 的安全余量。`kLineLostStopTicks = 80`，总停车上限
约 1.6 s，对教学场景的脱轨保护是合适量级。

### 为什么 ADR-005 的反对论据现在不成立

代码里 `ambiguousCenterTimedOut(ticks, lastError)` 已经包含 `lastError != 0`
这一项：长直线居中场景下系统 `lastError` 一直是 0，超时根本不会触发。ADR-005
当时担心的"长直线 0.5 s 误判"在现在的实现下不存在。

## Consequences

正面：

- 控制器接口与实际行为一致；阅读者不需要追问"I 是不是在悄悄工作"。
- 配置文件不再包含一个错的、未经核对的 `kPidIntegralLimit`。
- 默认配置下，脱轨能在 0.8 s 内被检测到、进入失线搜索；再 0.8 s 仍找不回线就停车。
- ISR/主循环少做一次 32-bit 加 + 一次 clamp（量级可忽略，但意图清晰）。
- 删了一段 SRAM 上的 `int32_t integral_`（4 字节 × 1 实例 = 4 字节）和一组 anti-windup
  分支。

代价：

- 现有任何依赖旧 `PidController` 名字、`PidGainsQ8` 名字、`ki` 字段、`allowIntegral`
  参数的代码必须同步更新（本仓库内已完成）。
- 真要做 I（比如未来加 IMU/编码器），需要新的类与新参数，不是"取消注释"。这是
  本 ADR 想要的成本。
- 旧标定下的 `kAmbiguousCenterLimitTicks = 0` 行为变了：以前会一直直行，现在会
  在 0.8 s 后进入失线搜索。硬件阶段如果发现某些场景（比如赛道有明显白色间断段）
  导致频繁误判，按场景把该值上调或在该场景下关闭避障/失线保护。

## Verification

- 静态搜索：生产代码不出现 `digitalRead`、`digitalWrite`、`analogRead`、
  `analogWrite`、`pulseIn`、`delay`、`delayMicroseconds`、`String`、`new`、`malloc`。
- 静态搜索：项目代码不写 Timer0/Timer2 寄存器；Timer1 寄存器写入仍集中在
  `Timer1MotorPwm.cpp`。
- 静态搜索：仓库内不出现 `PidController`、`PidGainsQ8`、`allowIntegral`、
  `kPidIntegralLimit`（保留在 ADR 文档与少量解释性注释里的旧名字除外）。
- 代码审计：`PdController::update()` 只计算 `kp * error + kd * derivative`，
  没有任何积分相关分支或状态。
- 代码审计：`profileForEstimate` 的 ambiguous 分支仍然 `usePd = false`，
  不更新 PD；`ambiguousCenterTimedOut` 仍保留 `lastError != 0` 守门。
- 硬件阶段：
  1. 长直线场景静态确认不会误进失线（lastError 应保持 0）。
  2. 故意把车手动推出轨道，确认 0.8 s 内进入 lost、再 0.8 s 内停车。
  3. PD 调参流程不变：先调 P，再调 D。

## References

- ADR-001：`docs/decisions/ADR-001-line-follower-architecture.md`
- ADR-005：`docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md`
  （本 ADR 在 ki / 双白超时两点上取代它）
- ADR-007：`docs/decisions/ADR-007-startup-deadband-and-default-speeds.md`
- Astrom & Murray, "Feedback Systems"（章节：Integral wind-up）
- ATmega328P 数据手册：https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
