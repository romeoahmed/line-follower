# ADR-010: AtomicGuard RAII、LineState enum 与 .cpp-only 工具

## Status

Accepted; §7 saturation policy superseded by ADR-011 (mixSaturate 差分保持饱和)

## Date

2026-05-31

## Context

ADR-009 把 `RobotController` 状态机与几处长期累计的结构噪声集中清理掉之后，剩下的
不是 bug、也不属于 ADR-001..009 已经覆盖的方向，但仍然让代码"看起来在做正确的事，
但表达方式比 C++11/AVR/no-STL 这套约束允许的最干净路径冗长"。本 ADR 把这些剩余的
精雕一次性收尾，目标是**零行为变化**：编译输出对所有现实控制路径必须等价。

具体观察：

1. **`SREG` save / `cli()` / `SREG = saved` 模式出现 ~13 次**，分散在
   `Timer1MotorPwm.cpp`、`FastIo.h`、`UltrasonicRangeSensor.cpp`。每一次都是
   手动维护"配对"的两行，外加多个早返回路径上的恢复点（实际代码里目前都用单返回
   路径回避了这个问题，但任何一次未来的"加一个 if"都可能漏掉 restore）。
2. **`LineEstimate` 用四个独立 `bool` (`valid`, `ambiguous`, `intersectionLike`,
   `lost`) 表达五个互斥状态**。可表达 16 种组合，合法只有 5 种。`RobotController`
   消费时是 `if (!estimate.valid || estimate.lost) ... else if (estimate.ambiguous)
   ... else if (estimate.intersectionLike) ...`——读起来要回头确认"是 valid 在前
   还是 lost 在前"，而且交叉/双白/失线之间的真值关系不在类型里。
3. **`MotorDriver` 头文件里挂着五个 `private static` pure helper**
   (`clampSignedPwm`, `applyCompensation`, `magnitude`, `signOf`,
   `rampToward`, `applySide`)。它们都是无状态计算函数，本质上是".cpp 实现细节"，
   放在类声明里给读者看的是"MotorDriver 公开承诺了 6 个内部工具的语义"，但其实
   没人调它们。
4. **`Timer1MotorPwm::copyFrame` 有两个近乎相同的重载**——`volatile→volatile` 与
   `const-ref→volatile`，只是源类型不同；体积上 ~20 行重复。
5. **`Timer1MotorPwm::kMinimumEdgeTick = 2` 缺少 rationale 注释**：未来读者不知道
   为什么是 2，也不知道改成 1 或 3 是否安全。
6. **`UltrasonicRangeSensor::pulseTicksToMillimeters` 公式 `(pulseTicks * 5 + 29) / 58`
   没有写为什么是 29**——读者要自己推一遍 round-to-nearest 才能确认。
7. **`RobotController::poll()` 里 `missedControlTicks_ += ticks - 1` 在饱和边界**
   会回卷。当前 control tick 上限是 255（`Timer1MotorPwm::takeControlTicks` 已经
   capped），所以单次最大 +254；而 `missedControlTicks_` 是 `uint16_t`。理论上能
   累计 ~258 次 254 才回卷一次，硬件上不会发生，但行为上是 wrap，不是 saturate；
   既然它是"诊断字段"，saturate 更符合语义。

## Decision

### 1. AtomicGuard：RAII 包装 SREG save/cli/restore

新增 `AtomicGuard.h`（头文件，单类）：

```cpp
class AtomicGuard {
 public:
  AtomicGuard() : saved_(SREG) { cli(); }
  ~AtomicGuard() { SREG = saved_; }
  // 删拷贝/移动：destructor 必须正好跑一次。
 private:
  uint8_t saved_;
};
```

替换路径：

- `Timer1MotorPwm.cpp`：`begin / submit / emergencyStop / takeControlTicks /
  captureTimeTicks` 共 5 处。
- `FastIo.h`：`writeSensorEnable` 共 1 处。
- `UltrasonicRangeSensor.cpp`：`writeTriggerHigh / armEchoCapture /
  echoCaptureBusy / markEchoTimeout / cancelEchoCapture / takeEchoResult /
  beginUltrasonicPinsAndInterrupt` 共 7 处。

成本：单字节存储，构造两条指令（`in r0, SREG; cli`），析构一条（`out SREG, r0`）
——与手写代码完全相同。AVR-GCC 在 `-Os` 下把 header-only RAII 类内联到 zero
overhead。

`UltrasonicRangeSensor::begin()` 的"`g_instanceCount != 1` → `cli()` + 死循环"
trap 保留原状：那一处的语义是"永久关中断停在这里"，不属于"短临界区"模式，套
`AtomicGuard` 反而失去原本"硬件阶段一眼看到"的意图。

### 2. LineState 取代四个 bool

`LineEstimator.h`：

```cpp
enum class LineState : uint8_t {
  kOffsetLeft, kOffsetRight, kCentered, kAmbiguous, kIntersection, kInvalid,
};

struct LineEstimate {
  LineState state;
  int16_t error;
  bool isOffset() const;       // kOffsetLeft | kOffsetRight
  bool isLost() const;         // kInvalid
  bool isAmbiguous() const;    // kAmbiguous
  bool isIntersection() const; // kIntersection
};
```

旧的 `(valid, ambiguous, intersectionLike, lost, error)` 五元组改为
`(state, error)`，可表达组合从 2⁴=16 收缩到 6，且全部合法。

`RobotController::profileForEstimate` 从串联 if 改为 `switch (estimate.state)`，
五个状态各自列出 profile；不写 `default`，让编译器在新增 `LineState` 值时报
未处理警告（与 ADR-009 对 `RobotState` switch 的同款守门）。

`kCentered` 状态只在 `kOnLine` 模式产出；`kIntersection` 只在
`kBetweenSensors` 模式产出。两个模式之间互不影响（`RobotConfig::kCenterMode`
是 compile-time 常量，编译器会优化掉不可达分支）。

### 3. MotorDriver 头文件瘦身

把 `clampSignedPwm`、`applyCompensation`、`magnitude`、`signOf`、`rampToward`、
`applySide` 从 `private static` 全部下沉到 `MotorDriver.cpp` 的匿名命名空间。
公开类只剩 `begin / setTargetSpeeds / update / stopNow + currentLeft /
currentRight` + 一个 private static `stepMotor`（保留为类成员，因为它要写
`MotorState`，而 `MotorState` 是 private 嵌套类型——匿名命名空间无法访问私有
嵌套类型）。

效果：包含 `MotorDriver.h` 的翻译单元再也看不到那些 helper 名字。`RobotConfig.h`
不需要被 helper 用到的常量再"经过头文件"暴露给消费者。

### 4. copyFrame 模板化

`Timer1MotorPwm.cpp` 把两个 `copyFrame` 重载合并为一个 `template <typename Source>`
函数，对 `volatile PreparedFrame&` 与 `const PreparedFrame&` 两类源参数自动实例化。
AVR-GCC 在 `-Os` 下产出与手写两份相同的字节序列。

### 5. 注释补充（hardware/timing rationale）

- `Timer1MotorPwm::kMinimumEdgeTick = 2` 注释解释为什么 2：避免极小 duty 折叠成
  亚微秒 pulse；说明 `duty == 0` 路径独立，floor 不会把"想要关"误升成"始终开"。
- `UltrasonicRangeSensor::pulseTicksToMillimeters` 注释展开 `pulseTicks * 5 / 58`
  的推导，说明 `+ 29` 是 round-to-nearest（29 = 58/2），并给出 echo-timeout 上限
  下中间值不会溢出 32-bit 的边界论证。
- `Timer1MotorPwm` 模板 `copyFrame` 注释说明等价于两份手写重载。

### 6. missedControlTicks_ saturate-on-overflow

`RobotController::poll()` 把 `missedControlTicks_ += (ticks - 1)` 改为带 headroom
检测的饱和加法：

```cpp
const uint16_t add = static_cast<uint16_t>(ticks - 1);
const uint16_t headroom = static_cast<uint16_t>(0xFFFFu - missedControlTicks_);
missedControlTicks_ += (add <= headroom) ? add : headroom;
```

这是诊断字段，wrap 会让连续运行很久的小车回到看似"miss 数变小"的状态，掩盖
真实的实时性问题；saturate 让数值守在 0xFFFF 上更诚实。

### 7. clampMotorCommand 保留——而且这是物理上严格更对的版本

计划阶段把 `RobotController::clampMotorCommand` 视为 `MotorDriver` 内 clamp 的
重复，准备删掉。审计期复查发现两种方案在不饱和区等价，但**饱和时行为分叉**，
且分叉方向是单边的：保留控制器层 clamp（Option A，当前实现）物理上严格更对。
没有 trade-off。

记 trim 比例为 `s = 1 + trim/1000`，硬件上限 `M = kMotorMaxPwm`。两方案：

- Option A（当前）：`final = clamp_M( s × clamp_M(raw) )`
- Option B（删除）：`final = clamp_M( s × raw )`

**trim 的物理意义**：补偿同一 PWM 下左右电机物理输出的不对称。如左电机比右
强 5%，设 `kLeftTrim = -50`（`s_L = 0.95`），目标是同命令下两轮物理转速相等。
**`kMotorMaxPwm` 的物理意义**：单电机 PWM 占空比的安全包络（热、电流、电池压降；
ADR-007）。它约束 PWM 域，不约束"物理效力"域。

**决定性案例**：左强 5%，`s_L = 0.95`、`s_R = 1.0`，命令 `(left, right) = (250, 250)`、
`M = 200`。

| | A：先 clamp 再 trim | B：先 trim 再 clamp |
|---|---|---|
| 左 PWM | clamp(250)=200 → ×0.95 = 190 | 250×0.95=237.5 → clamp=200 |
| 右 PWM | clamp(250)=200 → ×1.0 = 200 | 250 → clamp=200 |
| 左物理效力 | 190 × 1.05 ≈ 199.5 | 200 × 1.05 = 210 |
| 右物理效力 | 200 × 1.0 = 200 | 200 × 1.0 = 200 |
| 结果 | 平衡，直行 ✓ | 左快 5%，向右偏 ✗ |

一般化：一旦 `raw` 进入饱和，Option B 把"trim 补偿"白白送进 clamp 被截掉
（两侧顶到同一个 M），**饱和时 Option B 退化为"无 trim"**。Option A 在 PWM 域
做硬包络，让 trim 始终在物理域内做差分补偿，**饱和时仍保差分**。

弯道不对称命令（如 `(216, 44)`、`s_L = 0.97`）：

- A：左 200×0.97=194，右 44，差速 150。
- B：左 (216×0.97=209.5)→200，右 44，差速 156。

差 6 PWM 看似小，但物理意义是 trim 补偿的左右差距在 Option B 下被 6 PWM 扭曲；
PD 会代偿，代偿会被 D 项当成"系统误差"反向修正，污染调参可重复性。

**更对的方案存在但属于行为变化**：差分保持型饱和（"两个命令成对按比例缩放回
合法包络"）是航空/无人机的常见做法。对线性循迹小车，它隐含"弯道饱和时同时
降低基速"，可能好也可能坏，要硬件标定。这不在本 ADR 范围；如要采用，开新 ADR
讨论 trade-off。但 A vs B 之间，**A 严格胜，无 trade-off**。

代码侧：`RobotController::clampMotorCommand` 保留。在 `runFollowLine` 该调用
处给出一行简短引用："see ADR-010"；ADR-010 是物理论证唯一权威。`MotorDriver`
内层 clamp 也保留：它防御的是 trim 放大后的命令超过包络（如 trim=+50% 下
clamp(200) 后 ×1.5 = 300，必须再 clamp 回 200）。两层 clamp 不冗余——它们守的是
不同域的不同包络。

## Rationale

### 为什么是 AtomicGuard，不是 ATOMIC_BLOCK 宏

avr-libc 的 `<util/atomic.h>` 提供 `ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ... }`，
功能等价。两点偏好用自家 RAII：

- `ATOMIC_BLOCK` 是宏 + for-loop 技巧，调试器单步时跳奇怪步骤；阅读时第一眼以为
  是 `for` 循环，要再读才反应过来。
- 项目已经禁第三方运行时库；尽量收敛对 avr-libc 的依赖面到"必要 register/intrinsic"
  ——`AtomicGuard` 是 4 行的 C++ RAII 类，无新依赖，可读性比宏更直接。

### 为什么 LineState 不带 error 字段做联合体 / variant

C++11 没有 `std::variant`；手写 union 会复杂化 trivial-copy/平凡构造的保证。把
`error` 留作旁挂 `int16_t`、用 `LineState` 决定"error 是否有意义"——`kAmbiguous`、
`kCentered`、`kIntersection`、`kInvalid` 四态下 `error == 0`，但消费者从不依赖
"读 error"，而是依赖 `state` 分支。这种"discriminator + payload"是 C 风格 enum
最朴素也最可读的形式，符合 ADR-009 "Don't introduce machinery you don't need."

### 为什么 stepMotor 没下沉到匿名命名空间

`stepMotor` 写 `MotorDriver::MotorState`——而 `MotorState` 是 `MotorDriver` 的
private 嵌套类型。下沉为自由函数有三条路：

1. 把 `MotorState` 从 private 提到 namespace `lf::` 顶层 → 暴露内部结构。
2. 把 `MotorState` 提到 `public` → 暴露内部结构。
3. 让 `stepMotor` 走 friend 声明 → 添加复杂度换零收益。

第 4 条路（保留为 `private static` 成员）就是本 ADR 选的：私有数据只在私有方法里
被写。和把 5 个 pure helper 下沉到匿名命名空间合在一起，已经把"无状态计算"和
"私有状态修改"的分界画干净了。

### 为什么 LineEstimator 中 kCentered 与 kIntersection 都保留

`kCenterMode` 是 `constexpr`，编译期就知道哪条路径会进哪个分支。但 enum 定义不
按 build configuration 削减——以后切换默认 mode 时只改 `RobotConfig` 即可，无需
回到 enum 里 add/remove case。代价是 6 个枚举值取代了"实际只会出现 5 个"，但
`switch` 穷举覆盖完整，无运行时成本。

### 为什么不动 ADC、PCINT 状态机、Timer1 边沿调度器

按 AGENTS.md "How To Tell A Real Bug From A Style Nit" 与"Hard Constraints" 的
排序，这些是 ISR-critical 且 datasheet-紧的代码。当前实现已经过 ADR-003 / 004 /
009 审核，本仓库本地缺 `arduino-cli`、无硬件，**任何重写都没法在静态分析以外
取得高置信度**。本 ADR 严格回避它们。

## Consequences

正面：

- `cli()` / SREG-restore 配对从手写两行变成 RAII 一行。早返回路径再加新分支
  时，restore 由作用域生命周期自动保证。
- 项目内 SREG 直写只剩两处：`AtomicGuard.h` 与 `UltrasonicRangeSensor::begin()`
  的硬故障 trap。`grep` 自检（见 Verification）能确认这一点。
- `LineEstimate` 不再可表达非法组合（如 `valid && lost`、`ambiguous &&
  intersectionLike`）。`RobotController::profileForEstimate` 的 if 链变成
  switch，编译器穷举守门。
- `MotorDriver.h` 公开面从 6 个 private 工具收缩到 0；翻译单元只看到 4 个
  public 方法 + 2 个 accessor + 1 个 private static `stepMotor`。
- `Timer1MotorPwm::copyFrame` 不再 ~20 行重复。
- `missedControlTicks_` 不会再因为长时间运行后 wrap 让诊断字段失真。
- 三处 hardware/timing rationale 注释补足，符合 AGENTS.md "Add comments only
  when they explain non-obvious hardware timing, register behavior, ISR/main-loop
  interaction, or safety"。

代价：

- 新增 `AtomicGuard.h`：一个 ~30 行的头文件、一个公开类型。
- 任何外部代码（测试、调试 sketch）依赖旧 `LineEstimate.valid / ambiguous /
  intersectionLike / lost` 字段的，必须改用 `state` 与 `isXxx()` helper。本仓库
  内 `RobotController` 已经同步更新。
- 任何外部代码引用 `MotorDriver::clampSignedPwm` 等 private static 名字的，会
  失联——本仓库内无此调用。
- 模板 `copyFrame` 让 `volatile` 限定符在调用点更隐式；如果未来需要做更复杂的
  ISR ↔ main 数据搬运，可能要回到显式 overload。当前两种实例化都是平凡赋值，
  足够。

### 编译验证缺失

`arduino-cli` 在本工作环境未安装；本次改动**没有进行 sketch 级别的编译验证**。
按 AGENTS.md "If `arduino-cli` is unavailable, say so. Do not claim compile
verification that was not actually performed." 必须明确记录这点。改动按以下原则
做到静态分析等价：

- `AtomicGuard` 与原 `uint8_t s = SREG; cli(); ...; SREG = s;` 模式在每个调用点
  是机械等价的——RAII destructor 在作用域结束运行，对应原代码在 return 前的
  `SREG = s`。
- `LineEstimate` 旧字段 → 新 enum 的映射是双向 bijection，每个消费分支按
  `(valid, ambiguous, intersectionLike, lost)` 元组到 `LineState` 一一对应；
  helper 让消费侧的语义意图不变。
- `copyFrame` 模板对当前两类 source（`volatile`、`const&`）实例化的字节码与
  原两份重载等价（成员逐个赋值，volatile 修饰在 `destination` 端）。
- pure helpers 从 `MotorDriver` private static 移到匿名命名空间不改变调用语义；
  AVR-GCC 在 `-Os` 都按 internal linkage 处理。

硬件阶段（不在本 ADR 内）应：
1. 在 ADR-007 / 008 / 009 已有的校准点跑一遍冒烟，确认电机方向、避障序列、失线
   方向搜索与改动前等价。
2. PD 调参流程不变——`PdController::update` 字节不变。

## Verification

静态搜索（仍应零生产代码匹配）：

```sh
grep -nE 'digitalRead\(|digitalWrite\(|analogRead\(|analogWrite\(|pulseIn\(|delay\(|delayMicroseconds\(|String\b|new\b|malloc' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . \
  | grep -v docs/ | grep -v .agents/
grep -nE 'TCCR0|OCR0|TIMSK0|TCCR2|OCR2|TIMSK2' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . \
  | grep -v docs/ | grep -v .agents/
```

新增自检：`cli()` / `SREG` 直写应只出现在 `AtomicGuard.h` 与
`UltrasonicRangeSensor::begin()` 的硬故障 trap：

```sh
grep -nE '\bcli\(\)|\bsei\(\)|\bSREG\b' --include='*.cpp' --include='*.h' --include='*.ino' \
  -r . | grep -v docs/ | grep -v AtomicGuard.h
# 期望：只剩 UltrasonicRangeSensor.cpp 的 begin() trap 那一行 cli()。
```

代码审计：

- `LineEstimator::estimate()` 每条 return 都给出 `(LineState, int16_t)`，且
  `error` 只在 `kOffsetLeft/kOffsetRight` 非零。
- `RobotController::profileForEstimate` switch 覆盖所有 `LineState` 枚举值，
  没有 `default`。
- `MotorDriver.h` 公开类不再声明 `clampSignedPwm`、`applyCompensation`、
  `magnitude`、`signOf`、`rampToward`、`applySide`；它们只出现在 `MotorDriver.cpp`
  匿名命名空间。
- `Timer1MotorPwm.cpp` 只有一个 `copyFrame` 定义（template），不再有两份 overload。
- `RobotController::clampMotorCommand` 仍然存在（本 ADR 第 7 节解释为什么不删）；
  注释指向本 ADR。

## References

- ADR-001 ~ 009（本 ADR 是后续清理，不修改它们的核心结论）
- AGENTS.md "Hard Constraints"、"Coding Rules"、"How To Tell A Real Bug From A
  Style Nit"
- ATmega328P 数据手册 §6 (Status Register, SREG) — global interrupt enable
  semantics
- avr-libc 文档 `<util/atomic.h>` 章节（与 `AtomicGuard` 等价的 ATOMIC_BLOCK
  宏；本项目选 RAII 形式）
- HC-SR04 datasheet（SparkFun PDF）— `distance_cm = echo_us / 58` 推导
