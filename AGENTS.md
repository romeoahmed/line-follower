# AGENTS.md

This file is the repository-level instruction set for coding agents working on
this firmware. Keep `README.md` in Chinese (human-facing) and this file in
English (agent-facing). `CLAUDE.md` is a symlink to this file.

## What This Project Is

Firmware for a single, fixed ATmega328P-AU teaching car board with two digital
line-tracking sensors, an HC-SR04 family ultrasonic module, and an L9110S-MS
motor driver. Treat this as a fixed-hardware embedded project, not as a
portable Arduino example. Cross-board portability is not a goal — code may, and
should, lean on the standard UNO pin map, the ArduinoCore-avr defaults for
Timer0/Timer2, and the exact wiring captured in `Pins.h`.

Top-level behavior (ADR-012): the car cruises straight by default. When any
line sensor sees black for `kEncounterConfirmTicks` consecutive control ticks,
it executes an open-loop calibrated left turn, then resumes straight. The
ultrasonic obstacle path stays intact and outranks the encounter trigger.

The control story:

- `Timer1MotorPwm` runs Timer1 in CTC mode at 4 kHz, generates a four-channel
  software PWM, and counts off a 10 ms control tick every 40 PWM periods. It
  also exposes a read-only 0.5 µs timestamp used by the ultrasonic capture.
- `MotorDriver` consumes signed `(left, right)` PWM commands, applies clamp,
  trim, ramp, direction blanking, and an L9110-aware drive mode (default:
  high-side brake / inverse PWM). Speed = 0 and direction blanking always
  leave both IA/IB low.
- `LineSensors` samples either digital `PINC` bits or the ADC directly (no
  `analogRead()`), with three-sample majority voting across control ticks.
- `LineEstimator` turns two booleans into one mutually-exclusive `LineState`
  enum (`kOffsetLeft / kOffsetRight / kCentered / kAmbiguous / kIntersection /
  kInvalid`) plus a discrete error. Consumers branch on the enum, not on
  scattered bool flags. See ADR-010.
- `UltrasonicRangeSensor` non-blockingly drives D13 TRIG and captures D12 ECHO
  via `PCINT0_vect`, timestamping with the Timer1 timebase. No `pulseIn()`.
- `RobotController` is a small state machine: `SensorSettle → GoStraight →
  {EncounterTurnLeft | ObstacleStop → AvoidanceTurnRight} → SensorSettle`,
  plus a `Stopped` terminal and a `Fault` failsafe entered on boot when
  `MCUSR.WDRF` was set. All state writes go through a single `transitionTo()`
  to keep entry side effects in one place; a single `stateTicks_` is shared
  across states (only one is active at a time). Both turns are open-loop on
  calibrated tick counts and cannot be interrupted once started. See
  ADR-009 (state-machine shape), ADR-011 (watchdog / failsafe), and
  ADR-012 (current behavior set and the PD removal).
- `BootStatus` captures `MCUSR` in an `.init3` hook before `main()` and disables
  the watchdog, per the avr-libc canonical pattern. `RobotController::begin()`
  consults `lastResetWasWatchdog()` to decide between arming WDT and parking
  in `kFault`. See ADR-011.

## Read First

Before changing code, read:

- `README.md`
- `docs/line-follower-plan-and-spec.md`
- `docs/decisions/ADR-001-line-follower-architecture.md`
- `docs/decisions/ADR-002-direct-register-adc-pwm.md`
- `docs/decisions/ADR-003-timer1-motor-timebase.md`
- `docs/decisions/ADR-004-ultrasonic-obstacle-avoidance.md`
- `docs/decisions/ADR-005-layered-control-and-right-turn-obstacle.md`
- `docs/decisions/ADR-006-teacher-compatible-motor-drive-model.md`
- `docs/decisions/ADR-007-startup-deadband-and-default-speeds.md`
- `docs/decisions/ADR-008-pd-only-and-mandatory-ambiguous-timeout.md`
- `docs/decisions/ADR-009-state-machine-cleanup.md`
- `docs/decisions/ADR-010-raii-atomic-and-line-state-cleanup.md`
- `docs/decisions/ADR-011-watchdog-fault-state-and-saturation-mix.md`
- `docs/decisions/ADR-012-encounter-turn-left-and-pd-removal.md`

The ADRs define hard boundaries. Do not weaken them casually to make an
implementation easier — open an ADR follow-up instead. Status lines on the
older ADRs flag which sections ADR-012 supersedes.

## Hard Constraints

- Target `arduino:avr:uno`, ATmega328P, 16 MHz, ArduinoCore-avr standard
  variant. `BoardProfile.h` enforces this with `#error` and `static_assert`.
- `Pins.h` is the only wiring source of truth. No bare Arduino pin numbers
  elsewhere in production code.
- Production control code must not call `digitalRead()`, `digitalWrite()`,
  `analogRead()`, `analogWrite()`, or `pulseIn()`.
- No `delay()` / `delayMicroseconds()` in the control path. Non-blocking
  state machines only.
- No `String`, no heap allocation (`new` / `malloc`), no third-party runtime
  libraries.
- Timer1 is the only timer this project initializes manually. Do not write
  TCCR0A/B, OCR0A/B, TIMSK0, TCCR2A/B, OCR2A/B, or TIMSK2. Timer0 is reserved
  for `millis()` / `micros()` / `delay()` even though we don't use them in the
  control path.
- All Timer1 register writes live in `Timer1MotorPwm.cpp`. Other modules see
  Timer1 only through the public API (`submit`, `emergencyStop`,
  `takeControlTicks`, `captureTimeTicks`, `captureTimeTicksFromIsr`).
- All ADC register writes live in `AdcDriver.cpp`.
- All sensor EN/OUT direct-port operations live in `FastIo.h`.
- All ultrasonic TRIG/ECHO and PCINT0 register writes live in
  `UltrasonicRangeSensor.cpp`.
- ISRs do only: port set/clear, edge scheduling, counters, flags, and read
  back of the Timer1 timestamp. No Serial output, ADC, PID, sorting, division,
  dynamic allocation, blocking waits, or long loops in any ISR.
- Multi-byte shared state with ISRs must be accessed inside an `AtomicGuard`
  scope (header-only RAII around `cli()` + SREG save/restore; see
  `AtomicGuard.h` and ADR-010) — never with bare reads. Hand-rolled
  `cli()` / `SREG = saved` pairs are no longer used outside that header and
  the singleton-trap path in `UltrasonicRangeSensor::begin()`.
- `A6` / `A7` exist only as ADC channels. They are never usable as digital
  I/O and the standard variant's `NUM_ANALOG_INPUTS` is 6 — keep them out of
  any digital code path.

## Architecture Map

| Module | Responsibility |
|---|---|
| `line-follower.ino` | Sketch entry; constructs a `RobotController` and pumps `poll()` |
| `BoardProfile.h` | Compile-time guard for ATmega328P / 16 MHz / UNO pin map |
| `BootStatus.*` | `.init3` `MCUSR` capture + early `wdt_disable()`; exposes reset cause to `RobotController` (ADR-011) |
| `MathUtils.h` | Header-only `constexpr` `clampSigned` / `maxOf` / `minOf` shared across modules |
| `Pins.h` | Functional names → Arduino pin → AVR port bit → ADC channel |
| `RobotConfig.h` | All tunable constants; `static_assert`s tying invariants to constants |
| `AtomicGuard.h` | Header-only RAII wrapper around SREG save + `cli()` + restore (ADR-010) |
| `FastIo.h` | Header-only sensor EN/OUT port operations |
| `AdcDriver.*` | Direct `ADMUX`/`ADCSRA` access for ADC0/ADC1 with timeout guard |
| `Timer1MotorPwm.*` | Timer1 CTC setup, four-channel software PWM, control tick, 0.5 µs timestamp |
| `MotorDriver.*` | Signed speed in, trim with clamp, ramp with startup-deadband kick, direction blanking, drive-mode-aware duty mapping |
| `LineSensors.*` | Digital or ADC sampling, polarity, three-sample majority filter |
| `LineEstimator.*` | Two-sensor booleans → `LineState` enum (six mutually-exclusive states) + discrete error |
| `UltrasonicRangeSensor.*` | Non-blocking TRIG state machine, PCINT0 ECHO capture, distance + obstacle latch |
| `RobotController.*` | State machine with single `transitionTo()`, 10 ms control loop, go-straight cruise, debounced encounter-turn-left, obstacle stop + open-loop right turn, watchdog + `kFault` (ADR-012) |

Keep hardware register writes in the existing low-level modules. Control
logic does not directly manipulate AVR registers.

## Coding Rules

- C++11 only (Arduino AVR core uses `-std=gnu++11`).
- Follow `.clang-format`. Run `clang-format -i *.cpp *.h *.ino` before
  committing.
- Prefer the existing small-module structure over inventing new abstractions.
  If a new module is needed, name its responsibility narrowly and update this
  file plus an ADR.
- Keep all tunable constants in `RobotConfig.h`; do not bury numeric literals
  in control logic.
- Keep wiring changes in `Pins.h`, then update `README.md`, the spec, and any
  relevant ADRs in the same change.
- Add comments only when they explain non-obvious hardware timing, register
  behavior, ISR/main-loop interaction, or safety. Don't restate the code.
- When you introduce a constant like `kFoo`, also add a `static_assert`
  capturing any invariant the rest of the code assumes about it.

## Behavior That Looks Subtle But Is Intentional

- **Brake-PWM duty semantics.** In the default
  `MotorDriveMode::kBrakeHighSideInversePwm`, `kMotor*Pwm` equals the effective
  drive-duty fraction over 255. `duty = 128` ≈ 50% drive; `duty = 255` = full
  drive. See ADR-006 / ADR-007.
- **Startup deadband kick.** `MotorDriver::rampToward()` jumps from 0 directly
  to `min(target, kMotorMinimumEffectivePwm)` on the first ramp step in each
  direction, so the motor does not idle inside its dead-band while ramping.
  Symmetric for the reverse direction. ADR-007. **This is the only place
  deadband is applied** — `applyCompensation()` does not floor target. ADR-009.
- **No missed-tick catch-up.** If `poll()` is late, `RobotController`
  increments `missedControlTicks_` and still runs only the current step.
  Catching up would queue stale control decisions and amplify latency.
- **`OCIE1B` is only enabled when an edge is pending.** The COMPB ISR disables
  itself when the per-period edge queue drains. Re-enable it via `submit()`
  through the shadow buffer; never poke `TIMSK1` from outside
  `Timer1MotorPwm`.
- **`PCINT0_vect` records pulses, not distance.** All distance math, sample
  confirmation, and obstacle latching run on the main loop, not in the ISR.
- **Both turns are open-loop and calibrated, not geometric.** Neither the
  obstacle-avoidance right turn (`kObstacleRightTurnControlTicks`) nor the
  encounter-driven left turn (`kEncounterTurnLeftControlTicks`) close a loop
  on body angle — there is no encoder or IMU. Their tick counts are the
  user's calibration against a specific battery / surface / tire combination.
  Do not claim or imply geometric accuracy in code or docs. See ADR-005 +
  ADR-012.
- **Encounter-turn-left has a confirmation window.** `kGoStraight` only
  triggers the turn after `kEncounterConfirmTicks` consecutive ticks of "saw
  black"; a single white reading resets the counter. This is symmetric with
  `kObstacleConfirmSamples` on the ultrasonic side and prevents edge-crossing
  noise from triggering spurious turns. See ADR-012.
- **In-flight turns are uninterruptible.** Once `kEncounterTurnLeft` or
  `kAvoidanceTurnRight` starts, no new event (obstacle latch, line edge)
  preempts it — only state's own tick counter ends it. The priority gate in
  `runControlStep()` checks all three maneuver states (`kObstacleStop`,
  `kAvoidanceTurnRight`, `kEncounterTurnLeft`) before any obstacle latch
  inspection. See ADR-012 §priority.
- **`UltrasonicRangeSensor` is a singleton by construction.** PCINT0 ISR state
  is shared file-scope globals; `begin()` traps if a second instance ever
  exists. Don't try to instantiate two.
- **`wdt_reset()` lives in `poll()` after `takeControlTicks()` returns > 0.** A
  watchdog kick at the top of `loop()` would mask the worst failure mode: main
  loop alive but Timer1 COMPA ISR dead. Only tick-gated kicks expose it. See
  ADR-011 §1.
- **`MotorDriver::applyCompensation`'s internal clamp matters even with no
  control-layer saturator.** ADR-012 retired `mixSaturate` along with PD; the
  control layer no longer issues mixed differential commands. `applyCompensation`
  still clamps because trim can push a per-side command over `kMotorMaxPwm`
  (e.g. trim = +50% on cruise = 180 → 270 → must clamp back to 220). Do not
  delete that clamp.
- **`kFault` is entered on boot when the previous reset was a watchdog reset,
  and stays there.** The MCU is alive (LED blinks, `loop()` runs) but the
  control path early-returns and motors are stopped; watchdog is disabled to
  prevent silent reboot cycles. This is how a hardware debugger distinguishes
  "firmware crashed and rebooted" from "firmware ran normally then chose to
  stop" (`kStopped`).

## Commands

Install dependencies when needed:

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr
```

Compile:

```sh
arduino-cli compile --fqbn arduino:avr:uno --warnings all .
```

If `arduino-cli` is unavailable, say so. Do not claim compile verification
that was not actually performed.

Static checks (must return no production-code matches):

```sh
grep -nE 'digitalRead\(|digitalWrite\(|analogRead\(|analogWrite\(|pulseIn\(|delay\(|delayMicroseconds\(|String\b|new\b|malloc' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . \
  | grep -v docs/ | grep -v .agents/
grep -nE 'TCCR0|OCR0|TIMSK0|TCCR2|OCR2|TIMSK2' \
  --include='*.cpp' --include='*.h' --include='*.ino' -r . \
  | grep -v docs/ | grep -v .agents/
```

Timer1 register writes (`TCCR1`, `OCR1`, `TIMSK1`, `TCNT1`, `TIFR1`) must
stay confined to `Timer1MotorPwm.cpp`.

## Hardware Safety (Strict)

Do not upload firmware, run motors, or perform hardware tests unless the user
explicitly asks for it and confirms the hardware setup. For any hardware-side
work:

- Disconnect USB and battery before wiring changes.
- Do not power motors from Arduino 5 V or computer USB.
- Require common ground between MCU, motor driver, sensors, and battery
  negative.
- Keep wheels off the ground for first motor tests; start with one motor at
  a time at the lowest practical PWM.
- Treat L9110S-MS supplier current ratings (1.2 A continuous, 2.0 A peak) as
  package-level upper bounds, not as proof of this PCB's thermal capacity.
  Always measure temperature and battery sag in your specific setup.
- Never treat `A6` / `A7` as ordinary digital I/O.
- `D13` is shared with the ultrasonic TRIG and the on-board LED / SCK /
  Bluetooth RX. Do not enable Bluetooth and ultrasonic at the same time.

## Ask First

Open a question before:

- Changing the functional pin mapping in `Pins.h`.
- Using Timer0 or Timer2.
- Adding third-party libraries.
- Adding Bluetooth, display, buzzer, camera, or any feature that consumes
  timers, real-time budget, or D13.
- Changing the control-tick rate, PWM carrier, or Timer1 prescaler.
- Treating uploading firmware or running on hardware as part of the
  acceptance criteria for a code change.

## Documentation Discipline

When behavior, constraints, or wiring change, update documentation in the
same change:

- Human-facing project usage → `README.md` (Chinese).
- Agent-facing work rules → `AGENTS.md` (English). `CLAUDE.md` is a symlink.
- Significant architectural or hardware-policy decisions →
  `docs/decisions/ADR-NNN-*.md`. New ADRs continue the numbering; never
  edit a merged ADR in place — supersede it with a new one and update the
  `Status:` line of the old one.
- Detailed hardware assumptions, wiring, validation, and calibration notes →
  `docs/line-follower-plan-and-spec.md`.

Do not document obvious code. Document why a hardware or timing choice
exists, what risk it controls, and what verification it requires.

## How To Tell A Real Bug From A Style Nit

When auditing or reviewing, sort findings by what would actually break the
car or the firmware safety story, not by what looks tidy:

1. **Wrong on hardware**: does the change make the motors not turn, turn the
   wrong way, brake when they should coast, miss an obstacle, or exceed the
   ISR budget? These are the only "must-fix" findings.
2. **Wrong against an ADR**: does the change reach into Timer0 / Timer2,
   call a banned API, write a hardware register from the wrong module, or
   make an open-loop motion sound like a geometric guarantee?
3. **Wrong against the datasheet**: ADC read order, TRIG width, PCINT flag
   handling, COMPA/COMPB ordering, etc.
4. **Style / readability**: only after the above are clean.

A clean diff that violates 1–3 is not acceptable; a noisy diff that fixes a
real 1–3 issue usually is.
