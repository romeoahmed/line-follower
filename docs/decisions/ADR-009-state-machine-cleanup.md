# ADR-009: RobotController 状态机清理与模块解耦

## Status

Accepted; state set partially superseded by ADR-012 (kFollowLine→kGoStraight, kLineLost retired, kObstacleTurnRight→kAvoidanceTurnRight, kEncounterTurnLeft added). The "single transitionTo() write-gate" + "single stateTicks_" architectural principles stand.

## Date

2026-05-31

## Context

ADR-001 至 ADR-008 已经把控制路径定型：单一 `RobotController` 类持有所有子系统，
按 `RobotState` 枚举驱动一个状态机。但实际代码里几个结构性瑕疵开始累积：

1. **状态进入副作用分散在 6 个 `enter*()` 方法里**。每个 enter 都要清一堆 `*Ticks_`
   字段、调 `pd_.reset()`、按状态调 `motors_.stopNow()` 或 `ultrasonic_.restartAfter
   Maneuver()`。新增状态意味着到处找"应该清什么"，靠人眼维护。已经出现过"清所有
   计数器以防万一"的防御性重复。
2. **状态计数器每个状态一个字段**：`settleTicks_`, `ambiguousTicks_`, `lostTicks_`,
   `obstacleStopTicks_`, `obstacleTurnTicks_`——但同一时刻只有一个状态活跃，所以五个
   字段中四个永远是 0。结构上是没必要的。
3. **状态优先级在 `runControlStep()` 里用嵌套 if + switch 表达**，避障状态优先级、
   `kSensorSettle` 早返回、`kStopped` 重复 ramp 等逻辑彼此交织。
4. **`kStopped` 终态每个 tick 仍然调用 `setTargetSpeeds(0,0)` + `motors_.update()`**。
   `enterStopped()` 已经调过 `stopNow()`，之后的循环都是空转。
5. **`kBoot` 枚举值只在 `begin()` 开头瞬间存在**，对状态机来说从未"可见"。
6. **`PdGainsQ8` 类型定义在 `RobotConfig.h` 里**，但它是 `PdController` 的接口契约，
   不是配置。结果 `PdController.h` 必须 include `RobotConfig.h`，把整个配置常量集都拉进
   控制器实现。
7. **`UltrasonicRangeSensor` 类有成员状态，但 PCINT0 ISR 共享状态保存在匿名命名空间**
   的 `g_*` 变量里——意味着这个类是隐式单例，没人显式保证只能 new 一次。
8. **`MotorDriver::applyCompensation` 把 `kMotorMinimumEffectivePwm` 当 target 的下限
   做 floor**，但 `rampToward` 又在跨 0 时做 deadband kick。两层 floor 语义重复
   （ADR-007 落实 ramp 层 kick 时，没把旧 target 层 floor 删掉）。
9. **`PdController::update()` 仍接收 `maxCorrection`，运行时校验它非负**——但 caller 在
   `RobotConfig` 中已有 `static_assert` 守门，运行时校验是死代码。

这些都不是 bug，但都是"看起来在做，其实没做"或"重复做"的结构噪声，会让未来的修改路径
混乱（"应该在哪儿清 ticks？"、"是不是要把 minimum 也加在这里？"）。本 ADR 一次把它们
清干净。

## Decision

### 1. 状态机重构（A1 + A2）

- 删 `kBoot`；`begin()` 直接进 `kSensorSettle`。
- 删 5 个独立的 `enter*()` 方法和 5 个独立 `handle*()` 方法。
- 引入唯一的 `transitionTo(RobotState newState)` 作为**唯一允许写 `state_` 的入口**。
  所有状态进入副作用集中在它内部的 `switch (newState)`，每个 case 只列出该状态需要的
  动作：
  - 所有状态都先 `stateTicks_ = 0; pd_.reset();`（共通副作用）。
  - `kSensorSettle`: `lastError_ = 0; motors_.stopNow();`
  - `kObstacleStop`, `kStopped`: `motors_.stopNow();`
  - `kObstacleTurnRight`: `ultrasonic_.restartAfterManeuver();`
  - `kFollowLine`, `kLineLost`: 不需要额外副作用。
- 删 5 个状态计数器字段，合并为单一 `uint16_t stateTicks_`。**同一时刻只有一个状态
  活跃**——共享一个计数器是干净的、不是 hack。`transitionTo` 清零它。
- 每个状态的 tick handler 命名为 `run*()`（`runFollowLine`, `runLineLost`,
  `runObstacleStop`, `runObstacleTurnRight`）；`kSensorSettle` 简单到直接内联进
  `runControlStep` 的 settle 分支。
- 状态优先级在 `runControlStep` 中按线性顺序明确：
  1. `kStopped` 早返回（终态不消耗 CPU）。
  2. 避障机动状态（不可打断）。
  3. 障碍 latch（任何"循迹相关"状态被它抢占）。
  4. `kSensorSettle`（不读传感器，等计数器）。
  5. 循迹/失线（读传感器 → estimate → switch）。
- switch 内必须穷举所有 `RobotState` 枚举值（不写 `default`），让编译器在新增状态时
  报"未处理"警告。

### 2. PdGainsQ8 解耦（A6）

- `PdGainsQ8` 移出 `RobotConfig`，定义在 `PdController.h`。
- `RobotConfig` 改为 include `PdController.h`，只提供数值常量。
- 依赖方向变得正确：`RobotController` → `PdController`（类型 + 算法），`RobotController`
  → `RobotConfig`（数值）；`RobotConfig` → `PdController`（类型），不再循环。

### 3. UltrasonicRangeSensor 单例显式化（A4）

- 加 deleted copy / assign。
- 加构造/析构里的 `g_instanceCount` 维护。
- `begin()` 检查 `g_instanceCount == 1`，否则 `cli()` + 死循环 trap——硬件阶段调试者
  会立刻发现，而不是看到"奇怪的 ISR 行为"。
- 选 `begin()` 而非构造检查，因为全局对象的构造顺序难以分析。

### 4. MotorDriver 简化（A7 + R4）

- `applyCompensation` 删掉 minimum-PWM floor。deadband 跳变只在 `rampToward` 一处做。
- `PdController::update` 删掉 `maxCorrection` 负值的运行时校验，依赖 `RobotConfig`
  里的 `static_assert(kPd*MaxCorrection >= 0 && <= kMotorMaxPwm)`。
- `PdController` 内的 `clamp32` 不再需要（删掉了 integral），`clamp16` 移到匿名命名
  空间的自由函数。

### 5. LineSensors 可读性（R2）

- 把 3 样本表决的 "3" 提到 `kHistoryDepth`、`kHistoryMask`、`kHistoryMajority`
  常量。`countThreeBits` 改为通用 `countBits`。
- 保留 `sampleCount_ < kHistoryDepth` 早返回作为防御性 fallback，加注释说明正常
  控制流（`kSensorSettle` 已经把 sample 跑满）不会命中。

### 6. 小清理

- 删 `abs16`（一处使用直接内联）。
- Timer1 `serviceDueEdgesAndScheduleNext` 的 `threshold = TCNT1 + 1` 加详细注释，
  说明 +1 是为了让 `edge.tick == TCNT1` 也计入"已过期"。

## Rationale

### 为什么用单一 transitionTo() 而不是表驱动 / 函数指针表？

C++11 + AVR 上的"现代"状态机有几种典型形式：

- `std::variant + std::visit`（C++17，不可用）
- 函数指针表（OK 但增加间接调用开销，调试器看到的是 `*(state_->handler)(this)`，
  阅读吃力）
- 模板状态机框架（如 Boost.SML，体积爆炸，不能用）
- enum + switch + 集中转换函数（**最朴素，最易读，零开销**）

本项目只有 6 个状态、固定路径。enum + switch 已经足够；引入函数指针表只是为了"看起来
现代"。**真正的现代是 "Don't introduce machinery you don't need."**

关键设计是"只有一个写 `state_` 的入口"，这把"修改路径混乱"的问题根除：以后改状态机
只需在 `transitionTo` 的 switch 里加 case，编译器告诉你漏了哪个。

### 为什么合并状态计数器？

UML 状态图里，每个状态有自己的局部数据是正常的。**但本项目状态机是单线程、非嵌套、
状态互斥**——同一时刻一个 RobotController 只在一个状态。把 5 个 `uint8_t` 字段
（4 个永远是 0）合并为 1 个 `uint16_t`，节省 4 字节 SRAM 是顺带的，**主要收益是删掉
"transitionTo 时该清哪个"的认知负担**。

`uint8_t → uint16_t` 是因为 `kObstacleRightTurnControlTicks` 已经是 `uint16_t`；
窄类型在共享情境下没意义。

### 为什么 PdGainsQ8 必须从 RobotConfig 移出去？

依赖方向。`RobotConfig` 应该是叶子模块——它包含的是数值，不是类型。把控制器的接口
类型放在控制器自己的头文件，是 C/C++ 项目最标准的做法。这次顺手修正。

### 为什么 UltrasonicRangeSensor 不直接改成命名空间？

那是更彻底的改动（删类、改成员调用、改测试），收益与 single-instance trap 相比有限。
保留类的形式，让单例约束**在 begin() 处显式 trap**——硬件阶段一旦有人 new 第二个，
会立刻发现，而不是在控制循环里产生奇怪行为。这是成本最低的根因修复。

### 为什么 deadband 不能放在 target 层？

target 层 floor 把"想要多大"硬抬高，会让用户看到"我设了 50 但实际跑 90"的违反直觉
行为。deadband 是"current 跨 0 时的物理启动需求"——属于执行器层。语义上必须在 ramp
里做。

## Consequences

正面：

- `RobotController` 状态机有单一写入口，新增/修改状态只需改 `transitionTo` 的
  switch，编译器会告诉你漏了哪个。
- 删了约 50 行重复的 enter*() 与 ticks 字段重置代码。
- `kStopped` 进入后控制循环每 tick 早返回，节省机械控制 + 电机 update 的 CPU。
- 模块依赖更干净：`RobotConfig` 不再倒挂控制器类型。
- `UltrasonicRangeSensor` 的"必须单例"从隐含约束变成显式 trap。
- `MotorDriver` 不再有两层 deadband，单一来源。
- 类型与行为更诚实：`PdController` 接口就是 PD 接口，不再保留"以后开 I"的伪装。

代价：

- 任何外部代码（测试、调试 sketch）依赖 `enter*()`、`handle*()`、`RobotConfig::Pid*` /
  `PidGainsQ8`、单个 `*Ticks_` 字段名的，需要同步更新。
- `transitionTo` 的 switch 与 `runControlStep` 的状态 switch 各自必须保持枚举穷举；
  新增状态时编译器会双重提醒。
- `stateTicks_` 在不同状态语义不同（在 `kFollowLine` 内是"连续 ambiguous 计数"，
  在 `kLineLost` 内是"丢线计数"），靠注释维护清楚。

## Verification

- 静态搜索：生产代码不出现 `digitalRead`、`digitalWrite`、`analogRead`、
  `analogWrite`、`pulseIn`、`delay`、`delayMicroseconds`、`String`、`new`、`malloc`。
- 静态搜索：Timer0/Timer2 寄存器未被写入；Timer1 寄存器写入仍集中在
  `Timer1MotorPwm.cpp`。
- 静态搜索：仓库内不出现 `enterLineLost`、`enterObstacle*`、`enterStopped`、
  `enterSensorSettle`、`handleSensorSettle`、`handleFollowLine`、`handleLineLost`、
  `handleObstacle*`、`abs16`、`ambiguousTicks_`、`lostTicks_`、`settleTicks_`、
  `obstacleStopTicks_`、`obstacleTurnTicks_`、`RobotState::kBoot`。
- 代码审计：`state_ =` 赋值只出现在 `transitionTo` 内（`begin()` 末尾的初始化除外）。
- 代码审计：`stateTicks_ = 0` 只在 `transitionTo` 与 `kFollowLine` 的 ambiguous-reset
  分支中出现。
- 代码审计：`MotorDriver::applyCompensation` 不再引用 `kMotorMinimumEffectivePwm`。
- 代码审计：`PdController.h` 不再 include `RobotConfig.h`；只依赖 `<stdint.h>`。

## References

- ADR-001 至 ADR-008（本 ADR 是清理，不修改它们的核心结论）
- AGENTS.md "How To Tell A Real Bug From A Style Nit" — 本次大部分改动属于
  "Architecture / readability"，按该分级标准在硬件未就绪前做是合适的。
