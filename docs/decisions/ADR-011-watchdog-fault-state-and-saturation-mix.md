# ADR-011: Watchdog + kFault、PD reset 守护、差分保持饱和混控、共享数学工具

## Status

Accepted; supersedes ADR-010 §7 on the saturation policy.

## Date

2026-05-31

## Context

ADR-001..010 把架构、状态机、ISR 边界、原子区与数据模型都拉直了。本轮再做一遍
全代码严苛复审，剩余可改的不再是"代码不干净"，而是项目离工业级 AVR 嵌入式默认
还差几件标准动作。本 ADR 一次落实其中可在固件内闭环完成的部分（硬件配置类——BOD
fuse、外部电池监测、编码器闭环——按 AGENTS.md "Ask First" 单独处理）。

具体观察：

1. **没有 watchdog**。任何让 `RobotController::poll()` 停跑（ISR 死循环、SRAM
   撞栈、未定义行为执行漂移）的情形，Timer1 仍在把上一帧 PWM 持续送给电机——小
   车以最后一刻的命令冲出去。AVR 上 watchdog 是 4 行代码就能装上的标准动作，
   缺它就是工业级嵌入式上的硬缺口。
2. **没有 boot 路径上的 MCUSR 捕获**。复位原因（power-on / external / brown-out
   / watchdog）对调试是关键证据；avr-libc 推荐的 .init3 + .noinit + naked 钩
   子是教科书写法，但本项目没有装。
3. **`PdController` 在跳过 update 时不清状态**。`runFollowLine` 在 `kAmbiguous`
   / `kIntersection` 时设 `usePd = false`，**不调** `pd_.update()`。`previousError_`
   保留前一次有效误差；下一次拿到非 ambiguous，`update()` 用 `error - previousError_`
   算 derivative——跨越了 N 帧间隔，D 项被异常放大。`hasPrevious_` 帮不上忙，
   它只在 `pd_.reset()` 时清零。
4. **单边硬 clamp 在饱和时吃掉 PD 差分**。`runFollowLine` 旧做法是
   `clamp(base+correction)` / `clamp(base-correction)` 各自单边裁。命令越过 ±M
   时，两侧被同一上限钳住，**差分被压缩**；PD 解算出的"我需要 N PWM 差分来转弯"
   被吃掉一部分，下一帧 PD 看到误差没改善继续推，再次饱和。**饱和场景下控制环
   失去线性度**。ADR-010 §7 论证了"先 clamp 再 trim"严格优于"先 trim 再 clamp"，
   但同时承认航空 ESC 那一类"差分保持饱和"更对，留给后续 ADR——本 ADR 就是后续
   那个。
5. **`MotorDriver.cpp` 与旧 `RobotController.cpp` 的 clamp 函数体逐字相同**。
   ADR-010 §7 论证两个调用点都该保留，但函数本身写两份是噪声。
6. **`UltrasonicRangeSensor::begin()` 与 `restartAfterManeuver()` 含 ~10 行状态
   重置粘贴两遍**——已经偏差了一处（`lastTriggerTicks_` 的语义注释只写在 begin），
   未来加字段时易漏改一处。

## Decision

### 1. Watchdog + .init3 MCUSR 钩子 + kFault 状态

新增 `BootStatus.{h,cpp}`：把 `MCUSR` 捕获到 `.noinit` 段的 `g_mcusrMirror`，
随后清零 `MCUSR` 并 `wdt_disable()`。钩子用 `naked` + `__attribute__((used))` +
`section(".init3")` 安装在 main() 之前。这是 avr-libc 官方手册推荐的标准模板，
也是 ATmega328P 数据手册 §11 (System Control and Reset) 的硬要求：**WDRF 不清
零时 WDE 被覆写为 1，wdt_disable() 单独调用无效**——必须先清 MCUSR 再 wdt_disable。
参考：https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html

`RobotController` 新增 `kFault` 状态：

- `begin()`：读 `BootStatus::lastResetWasWatchdog()`；为真则**永久进 kFault、
  彻底关 WDT**——不再 wdt_enable。结果是 MCU 仍跑 loop() 不阻塞，但所有控制
  路径早返回、电机断电。硬件调试者从 LED/串口能看到"固件活着但拒绝执行控制"，
  而不是看到"小车又复位又冲出去又复位"的反复循环。
- 正常路径：`wdt_enable(WDTO_120MS)`。120 ms = 12 × 控制 tick——能吸收单次
  ADC/超声波/printf 抖动，又能在持续饥饿 120 ms 内强复位。
- `poll()`：**只在控制 tick 推进时调 `wdt_reset()`**——这是抓"主循环还在跑但
  Timer1 COMPA ISR 死了"这种最阴险失效模式的关键。把 wdt_reset 放在 loop()
  顶部反而会掩盖它。
- `transitionTo(kFault)`：`wdt_disable()` + `motors_.stopNow()`，作为状态机内
  路径不可达保护态的入口。

### 2. PD reset on skip

`runFollowLine` 的 PD 调用从

```cpp
const int16_t correction =
    profile.usePd ? pd_.update(...) : 0;
```

改为

```cpp
int16_t correction = 0;
if (profile.usePd) {
  correction = pd_.update(...);
} else {
  pd_.reset();
}
```

不变式：**PD 内部 `previousError_` 永远只对应"前一次有效 update"**。跳过 update
即视为"PD 序列中断"，下次重启时 derivative 强制为 0。从源头消除跨越 N 帧的
derivative 冲击；调用点意图也比"用三元留 0"更明显。

### 3. 差分保持饱和混控（mixSaturate）

`runFollowLine` 的最终命令计算从单边硬 clamp 改为差分保持饱和：

```cpp
LeftRightCommand mixSaturate(int16_t base, int16_t correction) {
  int16_t left  = base + correction;
  int16_t right = base - correction;

  const int16_t over  = max(max(left, right) - M, 0);  // M = kMotorMaxPwm
  left  -= over;  right -= over;

  const int16_t under = max(-M - min(left, right), 0);
  left  += under; right += under;

  return { clampSigned(left, M), clampSigned(right, M) };  // |correction|>2M 兜底
}
```

**物理含义**：当 `(base+correction, base-correction)` 越过 ±M 包络时，**双边同
步偏移把命令拉回**——左右差分守恒，基速被动下降。等价于"饱和时车自动降速过弯"，
正是线性循迹器在饱和时该做的事。

**与 ADR-010 §7 的关系**：§7 在 Option A（先 clamp 再 trim）vs Option B（先
trim 再 clamp）之间选了 A，并明确说"还有 Option C（差分保持饱和）严格更优，
留给后续 ADR"。本 ADR 把 Option C 落实。

决定性数值对比，单边越界场景：`base=180, correction=40, kMotorMaxPwm=200`，
PD 期望差分 = 2×correction = 80。

| 量 | 单边硬 clamp（旧） | mixSaturate（新） |
|---|---|---|
| left raw  = base + correction | 220 | 220 |
| right raw = base − correction | 140 | 140 |
| over = max(left, right) − M   | — | 20 |
| left final  | clamp(220) = 200 | 220 − 20 = 200 |
| right final | clamp(140) = 140 | 140 − 20 = 120 |
| 实际差分 | 60（少 20，吞掉 25% 的 PD 命令）| 80（守恒）|
| 平均基速 | (200+140)/2 = 170（高 10）| (200+120)/2 = 160（如实降速）|

直线场景 `correction` 小时两路径数值完全相同；只在饱和时分叉，分叉方向是
"差分守恒 vs 基速守恒"——线性循迹器在饱和时应该让差分守恒，让基速被动降。

控制层不再需要 `RobotController::clampMotorCommand`——`mixSaturate` 的最后一
行兜底 clamp（`|correction| > 2M` 的退化情形）覆盖了原本它防的事。

**MotorDriver 内的 `applyCompensation` clamp 保留**：它防的是 trim 放大后越限
（如 trim=+50% 下命令 200 → 300），属于**不同域的不同包络**——trim 在 PWM 域
（MotorDriver 内），mixSaturate 在控制层。两层不冗余。

### 4. MathUtils.h：共享 constexpr 数学工具

新增 `MathUtils.h`，提供 `clampSigned<T>`、`maxOf<T>`、`minOf<T>`——
`constexpr` + template，AVR-GCC 在 `-Os` 下完全内联到调用点，零运行时开销。

`MotorDriver.cpp` 的本地 `clampSignedPwm` 删除，改用 `clampSigned<int16_t>`。
`RobotController.cpp` 的 `mixSaturate` 用同一工具。**单一来源**。

### 5. UltrasonicRangeSensor::resetState()

`begin()` 与 `restartAfterManeuver()` 共享的 ~10 行字段重置抽进一个 private
`resetState()`。这两个入口在生命周期内语义完全相同：把模块状态拉回"上次测距间
隔已过、可以立刻发起测量"。

## Rationale

### 为什么 WDTO_120MS

avr-libc `WDTO_*` 是预定义档位，120 ms = 12 × 10 ms 控制 tick。论据：

- 太短（30/60 ms）：单次 ADC + 超声波 poll + PD 计算偶发抖动就会触发误复位。
- 太长（500 ms+）：失活后小车会以最后命令冲出去半秒，避障/失线保护完全失效。
- 120 ms 给出 12 个控制周期容差：能吸收任何已知的一次性抖动；持续饥饿超过
  120 ms 就是真出问题了，必须强复位。

### 为什么 .init3 而不是在 setup() 里读 MCUSR

Arduino core 的 `init()` 在 `setup()` 调用前不读 MCUSR，但**Optiboot 早期版本
（包括 Arduino UNO 出厂 4.x）会改写 MCUSR**，所以 setup() 看到的可能已经不是
硬件复位时的值。`.init3` 是在 .data/.bss 初始化之前、core init 之前运行——比
任何 C 代码都更靠近上电瞬间，是 avr-libc 手册给出的标准位置。在 Arduino UNO
出厂 Optiboot 上 MCUSR 仍可能被 bootloader 覆写一次，但这是硬件层面的限制，
固件能做的最早捕获就是 .init3。

### 为什么 kFault 不直接复用 kStopped

`kStopped` 是**计划内终态**——失线搜索超时正常退出。`kFault` 是**失活态**——
固件上一帧出过 UB / ISR 死循环 / 撞栈。两者电机行为相同（永久停车），但语义、
诊断含义、对 WDT 的处理（kFault 关 WDT 是为了不再无声重启）都不同。共用一个
枚举值会让"为什么车停了"的现场调试变难。

### 为什么不补 BOD fuse 设定

BOD（brown-out detection）属于 fuse + bootloader 烧录范畴，**不是固件改动**。
ATmega328P 在 16 MHz 下安全工作电压下限是 4.3 V；Arduino UNO 出厂 efuse 默认
2.7 V，电机抽流让 Vcc 跌到 3 V 时 MCU 会执行错误指令但不复位。这是 fuse 改动
（重烧 bootloader 时改 efuse=0xFC），按 AGENTS.md "Ask First" 走单独硬件流程。

本 ADR 装 BOD 反应——`BootStatus::lastResetWasBrownOut()`——是为将来用，让
RobotController 或调试钩子能根据复位原因做不同处理。当前控制路径不消费它。

### 为什么差分保持饱和不另开 ADR

ADR-010 §7 已经明确预告了"差分保持饱和留给后续 ADR"。本 ADR 是那个后续 ADR
的一部分；§2 不另起编号是为了把"PWM 域里关于饱和与差分的所有论证"放在一份文档
里，避免读者横跨多个 ADR 拼物理图景。ADR-010 §7 的"Option A vs B"论证仍然成立
（保留 Option A 比 Option B 严格更对），只是现在更新为：**Option C（mixSaturate）
严格优于 A**，§7 的 trade-off 表里 A 这一列从"当前实现"变成"已被 C 取代"。

ADR-010 头部 Status 更新为 `Accepted; §7 saturation policy superseded by
ADR-011`。

## Consequences

正面：

- 固件失活后小车不再以最后一帧命令冲出去；120 ms 内被 WDT 强复位，复位后
  kFault 永久停车。
- 复位原因可读：`BootStatus::lastMcusr()` 暴露完整 MCUSR 镜像，硬件调试能区分
  power-on / external / brown-out / watchdog。
- PD 跳过帧不再污染下一帧 derivative。
- 饱和场景下 PD 解算的差分不被吃掉，控制环在饱和边界保持线性度；副效应是
  整车被动减速过弯，与"饱和时降速"的物理直觉一致。
- `clampSigned` 单一来源，新增饱和场景时不会再造一个第三份 clamp。
- 超声波状态重置单一来源；以后加字段必然两个入口都改对。
- `kFault` 让"路径不可达"的兜底从 `kStopped` 分流出来，状态语义更清晰。

代价：

- 多一个头/源对（`BootStatus.h/.cpp`，~50 行）和一个头文件（`MathUtils.h`，
  ~25 行）；Flash 成本 < 100 字节（WDT 三函数也都是 avr-libc 内联到几条指令）。
- 多一个枚举值 `kFault`；状态机 switch 必须新增 case（已穷举，编译器守门）。
- 任何外部代码引用 `RobotController::clampMotorCommand` 的会失联（本仓库内无
  此调用）。
- 行为变化：饱和场景下整车速度比旧版略低（差分保持的代价）；硬件阶段直线段
  应观察不到差异（直线 correction 不饱和），弯道+陡转可能比旧版稍慢——这恰是
  线性饱和器该有的行为。
- Optiboot 4.x 等会改写 MCUSR 的 bootloader 上 WDRF 判断可能失真；这是硬件
  层限制，固件已做到最早捕获——文档说明即可。
- 启用 WDT 后，**任何会让 `poll()` 在 120 ms 内拿不到控制 tick 的改动**都会
  触发复位 + kFault。这是想要的：未来加调试 printf 必须留意 Serial.print 在
  9600 波特率下打印长字符串可能阻塞 > 120 ms。

## 编译验证缺失

`arduino-cli` 在本工作环境未安装；按 AGENTS.md "If `arduino-cli` is unavailable,
say so."——本次改动**没有进行 sketch 级别的编译验证**。改动按以下原则做到静态
分析高置信度：

- `BootStatus` 的 .init3 + .noinit + naked + used 四件套与 avr-libc 手册示例
  逐字一致（仅去掉了非 C++11 ABI 的部分用 `extern "C"`，保持链接器看到 C 名
  字解析）。
- `wdt_enable` / `wdt_reset` / `wdt_disable` 是 avr-libc 文档化的 always_inline
  静态函数，签名与 `<avr/wdt.h>` 完全匹配。
- `mixSaturate` 在 `correction = 0` / `correction = ±C 不饱和` 两种常规路径上
  与旧 `clampMotorCommand(base±correction)` 输出位等价；只在两侧同时越界时
  分叉，分叉方向由本 ADR §2 论证。
- `clampSigned<int16_t>` 与原 `clampSignedPwm` 在所有 int16 输入上数值等价。
- `pd_.reset()` 与旧"留 0 不调"行为差异仅当下一帧 `usePd = true` 时显现——
  derivative 强制从 0 起，正是本 ADR 想要的修复。
- `kFault` 不在 boot 默认路径上；启动正常的硬件不会观察到任何行为变化。

硬件阶段（不在本 ADR 范围）应：

1. 故意拔掉传感器一边、让 ambiguous 持续 > 80 tick，确认进入 kLineLost 而不是
   被 WDT 误杀。
2. 故意人为构造 ISR 死循环（在受控测试 sketch 里），确认 ≤ 120 ms 内复位并
   永久停在 kFault。
3. 多次复位观察电机方向、避障序列、失线方向搜索等价 ADR-007/008/009 标定。

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

ADR-011 专用自检：

```sh
# clampSigned 应只在 MathUtils.h 定义；其它源应只调用、不再定义。
grep -nE 'int16_t\s+clamp.*PWM|clampMotorCommand' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . | grep -v docs/
# 期望：零匹配（旧名字已删）。

# WDT 调用只应出现在 RobotController.cpp 与 BootStatus.cpp。
grep -nE 'wdt_(enable|reset|disable)\(' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . | grep -v docs/
# 期望：仅 RobotController.cpp 与 BootStatus.cpp。
```

代码审计：

- `RobotController::poll()` 中 `wdt_reset()` 在 `ticks == 0` 早返回之后、
  控制步之前调用——确保抓住 Timer1 ISR 静默失效。
- `RobotController::begin()` 启动顺序：硬件初始化 → BootStatus 检查 → 正常路径
  装 WDT。kFault 路径走 wdt_disable + return。
- `runFollowLine` 在 `!profile.usePd` 分支显式调用 `pd_.reset()`。
- `mixSaturate` 最后的兜底 clamp 覆盖 `|correction| > 2M` 的退化情形。
- `RobotController::clampMotorCommand` 已删除（声明 + 定义）。
- `UltrasonicRangeSensor::begin()` 与 `restartAfterManeuver()` 通过 `resetState()`
  共享字段重置。

## References

- ADR-001 ~ 010（本 ADR 不修改它们的核心结论；§7 of ADR-010 由本 ADR §3 取代）
- AGENTS.md "Hard Constraints"、"How To Tell A Real Bug From A Style Nit"
- ATmega328P 数据手册 §6 (Status Register)、§11 (System Control and Reset)
  https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
- avr-libc `<avr/wdt.h>` reference：
  https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
- avr-libc `wdt.h` source（WDTO_* 数值定义）：
  https://www.nongnu.org/avr-libc/user-manual/wdt_8h_source.html
- avr-libc 复位原因 boot 模板（.init3 + .noinit + naked）：同上 wdt 文档
  "Handling the Watchdog Reset Flag" 一节
- Astrom & Murray, "Feedback Systems"（章节：actuator saturation, anti-windup,
  differential preservation）
