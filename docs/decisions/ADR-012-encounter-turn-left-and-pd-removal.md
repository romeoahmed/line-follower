# ADR-012: 行为切换为「遇黑左转」+ PD 路径全量删除 + 适度提速

## Status

Accepted; supersedes ADR-008 in full and supersedes parts of ADR-005, ADR-007,
ADR-009, ADR-011 — see Supersedes table below for the exact slices.

## Date

2026-06-05

## Context

ADR-001..011 把项目固化在「黑线循迹」(`kFollowLine` + 分层 PD profile + 失线搜索)
这一行为模型上。用户要求把行为改为「**默认直行；任一传感器看到黑线就执行一次开环
左转；然后回到直行**」。

新行为下的事实：

1. 双数字传感器 + BetweenSensors 拓扑下，「见黑」是一个**离散事件**，不是连续误差；
   PD 失去物理意义（ADR-008 已经为完全相同的原因把 I 项删了——本 ADR 把这条原则
   推广到 P 与 D）。
2. 「失线」概念消失：直行模式下「都看到白」是正常巡航态，不是失活态。`kLineLost`、
   `kAmbiguousCenterLimitTicks`、`kLineLostStopTicks`、`kMotorSearchPwm` 全部失去
   消费者。
3. 「分层 profile」（直线 / 弯道 / 保守 / 寻线）失去消费者：只剩一种基速
   `kMotorCruisePwm`。`kMotorCurvePwm` / `kMotorCautiousPwm` / `kMotorSearchPwm`
   全部死代码。
4. `mixSaturate` 是 ADR-011 §3 为 PD 命令饱和场景设计的差分保持饱和——新行为
   `runGoStraight` 双轮等命令、`runEncounterTurnLeft` 双轮反向等幅命令，**不存在
   需要差分保持的饱和场景**，`mixSaturate` 自然失去消费者。
5. 现有的开环右转避障 (`runObstacleTurnRight`, 55 个 control tick) 是天然同构
   参考——遇黑左转就是它的镜像（L 反转 / R 正转）。
6. 用户要求「最现代、干净、优雅」的路径——等价于：**删死代码、起诚实的名字、
   保住安全架构**。

约束：

- `Pins.h` 不动——本次不改任何接线。
- Timer1 / Timer0 / Timer2 / ADC / PCINT / WDT / `.init3` 这些底层与安全
  机制完全不变（ADR-003 / 004 / 011）。
- 所有 `state_` 写入仍只走 `transitionTo()`（ADR-009）。
- AtomicGuard / banned API / 寄存器边界静态闸继续守门（ADR-010、AGENTS.md）。

本机无 `arduino-cli`，按 AGENTS.md "If `arduino-cli` is unavailable, say so."
**本次没有进行 sketch 级别的编译验证**——见 §"编译验证缺失"。

## Decision

### 1. 状态机改名 + 重排

`RobotState` enum：

```cpp
enum class RobotState : uint8_t {
  kSensorSettle = 0,
  kGoStraight,           // new
  kEncounterTurnLeft,    // new
  kObstacleStop,
  kAvoidanceTurnRight,   // renamed from kObstacleTurnRight
  kStopped,
  kFault,
};
```

变化：
- `kFollowLine` → `kGoStraight`（行为换了，名字要诚实）。
- `kLineLost` 删除（直行模式下「双白」是巡航态而非失活态）。
- `kObstacleTurnRight` → `kAvoidanceTurnRight`（与新的 `kEncounterTurnLeft` 同辈
  时，"Obstacle..." 命名读起来失对称；"Avoidance..." 明确意图）。
- `kSensorSettle` / `kObstacleStop` / `kStopped` / `kFault` 保留语义不变。

### 2. PD 与配套机制全量删除

删除：
- `PdController.h` / `PdController.cpp` 两个文件。
- `PdGainsQ8` 结构体（在 `PdController.h` 内已带走）。
- `kPdStraightGainsQ8` / `kPdCurveGainsQ8` / `kPdStraightMaxCorrection` /
  `kPdCurveMaxCorrection` / `kLineCurveErrorThreshold`。
- `RobotController` 的 `PdController pd_;` 成员、`int16_t lastError_;` 成员。
- `RobotController.cpp` 的 `FollowProfile` 结构体、`profileForEstimate()` 函数、
  `ambiguousCenterTimedOut()` 函数、`mixSaturate()` 函数、`LeftRightCommand`
  局部结构体、`kMotorMaxSigned` 常量、`#include "MathUtils.h"`（本文件已无引用）。
- 旧 `runFollowLine` / `runLineLost` 两个 handler。

保留：
- `LineEstimator` 与 `LineState` enum 不变——`runGoStraight` 的触发判定仍用它。
- `kLineErrorUnit` 保留：`LineEstimator` 仍发出离散误差数值，控制层不消费但
  接口稳定；未来切到 `kOnLine` 或加 ADC 加权时无需回填。
- `MathUtils.h` 文件保留——`MotorDriver::applyCompensation` 仍用 `clampSigned`；
  `RobotController.cpp` 已不再 include。

### 3. 删除「分层速度」与「失线相关」常量

删除：
- `kMotorStraightPwm` (160) — 由 `kMotorCruisePwm` (180) 取代。
- `kMotorCurvePwm` / `kMotorCautiousPwm` / `kMotorSearchPwm`。
- `kAmbiguousCenterLimitTicks` / `kLineLostStopTicks`。

### 4. 提速 + 新行为常量

新增：

| 常量 | 值 | 含义 |
|---|---:|---|
| `kMotorCruisePwm` | 180 | 默认直行双轮匀速；原 `kMotorStraightPwm = 160`，+12.5%。 |
| `kMotorMaxPwm` | 220 | 电机 PWM 上限；原 200，+10%。仍显著低于 255。 |
| `kEncounterTurnLeftPwm` | 160 | 开环左转 PWM 幅值，L 反转 / R 正转。 |
| `kEncounterTurnLeftControlTicks` | 55 | 开环左转持续 control tick 数。**初值**，与右转 (`kObstacleRightTurnControlTicks = 55`) 同——硬件阶段按目标转角重新标定。 |
| `kEncounterConfirmTicks` | 2 | 连续 N tick 见黑才触发左转；去抖，镜像 `kObstacleConfirmSamples`。1 = 关闭去抖。 |

新 `static_assert`：
- `kMotorCruisePwm <= kMotorMaxPwm`
- `kEncounterTurnLeftPwm <= kMotorMaxPwm`
- `kEncounterTurnLeftControlTicks > 0`
- `kEncounterConfirmTicks >= 1`
- `kMotorMaxPwm < kPwmFullScale`（守 255-duty 反相 PWM 计算余量）
- `kSensorSettleControlTicks >= LineSensors::kHistoryDepth`（在 `LineSensors.h`
  内；ADR-010 / 严苛审计 L1 遗留：settle 阶段必须把多数表决窗口跑满）

### 5. 触发判定与去抖

`runGoStraight(estimate)`：

```cpp
const bool sawBlack = estimateSawBlack(estimate);   // 4 个见黑 state 的 OR
if (sawBlack) {
  if (stateTicks_ < 0xFFFF) ++stateTicks_;
  if (stateTicks_ >= kEncounterConfirmTicks) {
    transitionTo(kEncounterTurnLeft);
    return;
  }
} else {
  stateTicks_ = 0;  // 见白打断 confirm 序列
}
motors_.setTargetSpeeds(kMotorCruisePwm, kMotorCruisePwm);
motors_.update();
```

`estimateSawBlack` 用 `switch` 穷举 `LineState`，包含 `{kOffsetLeft, kOffsetRight,
kIntersection, kCentered}`。`kCentered` 在默认 `kBetweenSensors` 模式下不会出现，
但保留是 forward-looking——切到 `kOnLine` 时无需改这一行。

### 6. 优先级

`runControlStep()` 早返回顺序：

1. `kStopped` / `kFault` 早返回。
2. **三个机动态**（`kObstacleStop`、`kAvoidanceTurnRight`、`kEncounterTurnLeft`）
   早返回——**已开始的机动不可被任何新事件打断**。
3. 障碍 latch 抢占（高于「直行中遇黑」的触发）。
4. `kSensorSettle`。
5. 采样 + 估计 → `switch(kGoStraight)`。

### 7. transitionTo 副作用

- `kSensorSettle` / `kObstacleStop` / `kStopped`: `motors_.stopNow()`。
- `kFault`: `wdt_disable()` + `motors_.stopNow()`。
- `kAvoidanceTurnRight`: `ultrasonic_.restartAfterManeuver()`（清掉触发它的
  obstacle latch）。
- **`kEncounterTurnLeft`: 不调** `restartAfterManeuver()`——左转入口不是 obstacle
  latch，没有需要清掉的状态；让超声波测距在左转期间持续运行，万一转向中途真撞
  上障碍也能尽快重新 latch。
- `kGoStraight`: 无额外副作用。

所有 case 末尾 `return`，无 `default`——编译器穷举守门（与 ADR-009 同款做法）。

### 8. 文档与 ADR 同步

- **README.md** 重写：行为概述、模块表、状态机图、调参表、首次校准顺序。
- **AGENTS.md**："What This Project Is" 段、Architecture Map（删 `PdController`
  行）、Behavior That Looks Subtle But Is Intentional（删 `mixSaturate` bullet、
  改右转 bullet 为对称的"两个转向都开环"bullet、新增 "encounter confirm window"
  bullet、新增 "in-flight turns uninterruptible" bullet）、Read First（加 ADR-012）。
- **`docs/line-follower-plan-and-spec.md`**：状态机表、默认值表、行为描述对齐。
- **就地更新旧 ADR 的 `Status:` 一行**：见 §Supersedes 表。

## Rationale

### 为什么删 PD，而不是 gate 掉

ADR-008 处理 I 项时已经走过同样的逻辑：「只在 `update()` 里加 `if (gains.kp == 0)`
跳过」能消除 CPU 浪费，但不能解决：

- 阅读混乱：`PdController` 仍在，新接手的人会以为 PD 在工作。
- 接口噪声：`profileForEstimate` 仍返回 `PdGainsQ8`，配置文件仍挂着五组 PD 常量。
- 复活路径错误：「未来再开 PD」的人会沿用旧 API 形状和旧数值——但新行为下 PD
  没有物理基础，旧数值更没有意义。

干净的删除把「未来再决定」推迟到真正可论证的时点：未来如果加 ADC 加权 / IMU /
编码器，需要一个**新的**连续误差源，那时显式扩展接口、显式新加配置、显式写测试。
这比「取消注释复活旧代码」安全得多。

### 为什么 `kGoStraight` 而不是「重命名 `kFollowLine` 但语义换」

ADR-009 已经定下基调：**状态名要诚实**。`kFollowLine` 这个名字承诺的是「我在跟随
一条线」——新行为没有跟随任何东西，它是「直行 + 触发式机动」。换名是为了让读者
一眼看出行为本质，也让 grep `kFollowLine` 不会在新代码里有偶发命中。

### 为什么提速选 +12.5% 而不是 +25%

ADR-007 把基础速度从 27-36% 有效驱动提到 50-65% 是"理论可走 → 桌面/木地板能走"
的关键变量；再上一档到 71% 仍在 L9110S-MS 安全包络内（1.2 A continuous，电机
工况未实测，但有 ~30% 余量到峰值 2.0 A）。`+25%` (200 PWM ≈ 78% 有效驱动) 会
把可用余量压到 ~20%，电池电压随放电从 5.5 V 跌到 4.5 V 时实际驱动会逼近 95%，
触发 brown-out 或 L9110 热限的概率显著上升。本次不引入硬件实测就走这么激进的
档位是不负责任的。

`kMotorMaxPwm` 220 是为了让未来标定 trim / 单边修正时仍有~10% 余量；同时给反向
死区跳变（`-kMotorMinimumEffectivePwm = -90` → 一步跳）留出对称空间。

### 为什么 `kEncounterConfirmTicks = 2`

`kObstacleConfirmSamples` 已经是 2——保持对称读起来不需要解释。20 ms 触发延迟
在 cruise 180 PWM 下小车不会冲过线很远（教学小车在木地板上的典型速度 < 30 cm/s
→ 20 ms ≈ 6 mm 位移），可以接受。如果硬件阶段发现"压线才转"，把它设为 1 是单
常量改动。

### 为什么 `kEncounterTurnLeftControlTicks = 55` 起步

`kObstacleRightTurnControlTicks = 55` 是 ADR-005/007 留下的右转 90° 的标定起点。
左转的对称镜像复用这个数字作为初值是合理的——但**物理上不保证左右对称**（重心、
轮胎、电机老化都会引入差异），所以 ADR-012 明确把它标为「初值，待硬件标定」。

### 为什么 `kEncounterTurnLeft` 不调 `restartAfterManeuver`

`restartAfterManeuver()` 的语义是「清掉旧的 obstacle latch 与未完成的 echo
capture，重新开始测距周期」——它存在是为了让"由 obstacle latch 触发的右转"在
退出时不被自己触发它的旧 latch 立刻再次抢占。

`kEncounterTurnLeft` 不是被 obstacle latch 触发的，没有这层风险。反过来，**保持
超声波测距持续运行**让我们在左转 (550 ms) 期间真撞上障碍时能尽快重新 latch；
左转完进 `kSensorSettle` → 下一帧就能正确响应避障。

### 为什么进行中的转向不可被打断

教学小车的"机动反悔"是最难调试的场景之一：用户看到"它快撞墙时往左转了一半又
退回去"，需要确定是状态机决定的还是机械的。让机动一旦开始就完成，是符合
"现行的右转避障"已经选定的策略——本 ADR 对称延伸到左转。

如果用户后续要"左转中可被避障打断"，那是一个明确的 trade-off（响应更快，但
左转半成品的位置不确定），单独开 ADR-013 讨论。

## Consequences

正面：
- 行为与代码命名一致；阅读者不需要追问"是不是循迹"。
- 删除 1 个文件 (PdController) + 1 个结构体 (FollowProfile) + 2 个 handler
  (runFollowLine, runLineLost) + 5 个 PD 常量 + 3 个分层速度常量 + 2 个失线常量
  + `mixSaturate` + `lastError_` + `profileForEstimate` + `ambiguousCenterTimedOut`
  + `LeftRightCommand` + `kMotorMaxSigned`。`RobotController.cpp` 净减 ~108 行
  (199 行 - 91 行 = 净 -108)。
- `static_assert` 数量净增 5 条（cruise/encounter PWM 守门、settle vs history
  深度守门、255-duty 余量守门）。
- 状态机仍单一写入口；新状态在两个 switch 中都被穷举（编译器守门）。
- WDT + kFault + `.init3` 路径完全不动——失活保护没有任何回归。

代价：
- 旧标定下的速度档位被合并；下次硬件运行必须重新测温、记录电池压降。
- `kEncounterTurnLeftControlTicks` 必须按目标转角重新标定，不能假设与右转
  对称。
- 任何外部代码（测试、调试 sketch）依赖 `PdController` / `PidGainsQ8` /
  `kPd*` / `lastError_` / `mixSaturate` / `kFollowLine` / `kObstacleTurnRight`
  名字的会失联——本仓库内已同步更新。
- 「巡航 180 PWM」在低电量电池上可能触发 brown-out；目前没补 BOD fuse
  （fuse 改动属硬件流程，AGENTS.md "Ask First"）。README 已加首跑测温/压提醒。
- 进行中的转向不可打断的设计意味着：左转过程中如果突然出现障碍，避障会延迟
  ≤ `kEncounterTurnLeftControlTicks × 10 ms ≈ 550 ms`。这是有意为之的。

## Supersedes

旧 ADR 的 `Status:` 一行加注本 ADR 取代了哪一部分（ADR 正文不就地编辑）：

| ADR | 被取代的部分 | 新 Status 后缀 |
|---|---|---|
| ADR-005 | 分层 PD profile 与"循迹核心" | `; control profiles and PD-driven follow superseded by ADR-012` |
| ADR-007 | 速度默认值 (kMotor{Straight,Curve,Cautious,Search}Pwm, kMotorMaxPwm) | `; speed defaults superseded by ADR-012` |
| ADR-008 | 整篇（循迹与 PD 都退役）| `; superseded by ADR-012 (line-following retired)` |
| ADR-009 | 状态集（kFollowLine/kLineLost）| `; state set partially superseded by ADR-012` |
| ADR-011 | §3 (mixSaturate) | `; §3 (mixSaturate) retired by ADR-012` |

ADR-001/002/003/004/006/010 的核心结论（架构、寄存器边界、Timer1 时基、PCINT 超声波、
电机驱动模型、AtomicGuard）**完全保留**——本 ADR 在它们之上做行为层的迁移。

## 编译验证缺失

`arduino-cli` 在本工作环境未安装。按 AGENTS.md "If `arduino-cli` is unavailable,
say so. Do not claim compile verification that was not actually performed."：
**本次改动没有进行 sketch 级别的编译验证**。改动按以下原则做到静态分析高置信度：

- 静态闸 (banned API、Timer0/2 寄存器、PdController/PdGainsQ8/kPd*、
  kMotor{Straight,Curve,Cautious,Search}Pwm、kAmbiguous*/kLineLost*/
  kLineCurve*、kFollowLine/kLineLost/kObstacleTurnRight/mixSaturate/
  profileForEstimate/FollowProfile/ambiguousCenterTimedOut/lastError_/
  runFollowLine/runLineLost、cli/SREG-outside-AtomicGuard、wdt_* 调用、
  Timer1 寄存器位置) 全部通过。
- 新状态名 `kGoStraight` / `kEncounterTurnLeft` / `kAvoidanceTurnRight` 在
  `runControlStep()` 与 `transitionTo()` 的两个 switch 中均有 case；无 `default`。
- `clang-format -i` 已对修改过的文件执行。
- `MotorDriver::setTargetSpeeds` 接受 signed 命令；`runEncounterTurnLeft` 发出
  `(-N, +N)` 与已有 `runAvoidanceTurnRight` 的 `(+N, -N)` 走完全对称的代码路径
  （`rampToward` 的反向 deadband kick 已经覆盖 `runObstacleTurnRight` 多年）。
- WDT 路径：`poll()` 仍在 `takeControlTicks() > 0` 后 `wdt_reset()`；`begin()`
  仍按 `BootStatus::lastResetWasWatchdog()` 选 `wdt_enable` 或 `kFault` 路径；
  `transitionTo(kFault)` 仍 `wdt_disable()`。`grep` 自检确认 `wdt_*` 仅出现在
  `RobotController.cpp` 与 `BootStatus.{h,cpp}`。

硬件阶段（不在本 ADR 范围）应：

1. 静态测试：手动遮黑 A0 / A1，观察是否触发左转；调整 `kEncounterConfirmTicks`。
2. 轮子离地：低 PWM 单轮点动确认方向；提速后温升 30 s 监测。
3. 落地低速：标定 `kEncounterTurnLeftControlTicks` 与 `kObstacleRightTurnControlTicks`
   到目标转角。
4. 联动测试：左转期间放障碍物——确认左转先完成，然后避障右转生效。
5. 失活验证：人为构造 ISR 死循环（受控测试 sketch），确认 120 ms 内 WDT 复位，
   重启后 `kFault` 永久停车。

## References

- ADR-001 ~ 011（本 ADR 是 ADR-008 的全量取代 + ADR-005/007/009/011 的部分取代；
  其余结论保留）。
- AGENTS.md "How To Tell A Real Bug From A Style Nit" — 本次改动属于"行为变更"
  + 结构清理；在硬件未就绪前做行为变更是用户显式要求，配套清理是同一 commit
  自然范围。
- ATmega328P 数据手册 §15 (Timer1 CTC mode)、§11 (System Control and Reset)、
  §24 (ADC): 行为变更不触碰这些；保留链接是因为 Timer1 / WDT / ADC 配置都按
  datasheet 校验过且未变。
- avr-libc `<avr/wdt.h>`: 同上；WDT 路径不变。
- SparkFun HC-SR04 datasheet: 同上；避障路径不变。
