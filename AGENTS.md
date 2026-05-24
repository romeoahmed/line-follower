# AGENTS.md

## Purpose

This repository contains firmware for a black-line following car built around an Arduino UNO compatible ATmega328P-AU teaching board. Treat it as a fixed-hardware embedded project, not as a portable Arduino example.

Use this file as the repository-level instruction set for coding agents. Keep `README.md` in Chinese and this file in English.

## Read First

Before changing code, read:

- `README.md`
- `docs/line-follower-plan-and-spec.md`
- `docs/decisions/ADR-001-line-follower-architecture.md`
- `docs/decisions/ADR-002-direct-register-adc-pwm.md`
- `docs/decisions/ADR-003-timer1-motor-timebase.md`

The ADRs define hard boundaries. Do not weaken them casually to make an implementation easier.

## Target And Constraints

- Target `arduino:avr:uno`, ATmega328P, 16 MHz, ArduinoCore-avr standard variant.
- `Pins.h` is the only wiring source of truth.
- Production control code must not call `digitalRead()`, `digitalWrite()`, `analogRead()`, or `analogWrite()`.
- Do not use `String`, heap allocation, or third-party runtime libraries in the control path.
- Timer1 is the only timer this project initializes manually.
- Do not write Timer0 or Timer2 registers.
- Timer1 is dedicated to motor software PWM and the 10 ms control tick.
- Do not add Servo, Tone, SoftwareSerial, or any feature that consumes timers or real-time budget without explicit user approval.

## Architecture Map

- `line-follower.ino`: sketch entry point, delegates to `RobotController`.
- `BoardProfile.h`: compile-time guard for ATmega328P, 16 MHz, UNO pin map.
- `Pins.h`: functional pin names, Arduino pin numbers, AVR port bits, ADC channels.
- `RobotConfig.h`: all tunable firmware constants.
- `FastIo.h`: fixed sensor EN/OUT port operations.
- `AdcDriver.*`: direct ADC0/ADC1 access.
- `Timer1MotorPwm.*`: Timer1 CTC setup, four-channel software PWM, control tick.
- `MotorDriver.*`: signed motor commands, clamp, ramp, direction blanking.
- `LineSensors.*`: digital or ADC sampling, polarity, filtering.
- `LineEstimator.*`: two-sensor line state to discrete error.
- `PidController.*`: integer Q8 PID.
- `RobotController.*`: state machine, line following, lost-line behavior, stop behavior.

Keep hardware register writes contained in the existing low-level modules. Control logic should not directly manipulate AVR registers.

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

Expected static-check shape:

- The first command should return no production-code matches.
- The second command should return no production-code matches.
- Timer1 register matches should stay in `Timer1MotorPwm.cpp`.

If `arduino-cli` is unavailable, state that clearly instead of claiming compile verification.

## Coding Rules

- Prefer the existing small-module structure over new abstractions.
- Use C++11-compatible Arduino AVR code.
- Keep ISR work short: port set/clear, edge scheduling, counters, flags only.
- Do not perform ADC, Serial output, PID, sorting, dynamic allocation, blocking waits, or long loops in Timer1 ISRs.
- Keep all shared ISR data behind `Timer1MotorPwm` APIs and protect multi-byte updates with short interrupt-disabled sections.
- Keep constants and tuning values in `RobotConfig.h`.
- Keep wiring changes in `Pins.h`, then update README, the spec, and any relevant ADR notes.
- Follow `.clang-format` for C++ formatting.
- Add comments only when they explain non-obvious hardware timing, safety, or register behavior.

## Hardware Safety

Do not upload, run motors, or perform hardware tests unless the user explicitly asks for it and confirms the hardware setup.

For hardware instructions or docs:

- Disconnect USB and battery before wiring.
- Do not power motors from Arduino 5 V or computer USB.
- Require common ground between MCU, motor driver, sensors, and battery negative.
- Keep wheels off the ground for first motor tests.
- Start with low PWM and one motor at a time.
- Treat L9110S-MS current ratings as supplier-page guidance, not as proof of this PCB's continuous thermal capacity.
- Never treat A6/A7 as ordinary digital I/O.

## Ask First

Ask before:

- Changing the functional pin mapping in `Pins.h`.
- Using Timer0 or Timer2.
- Adding third-party libraries.
- Adding Bluetooth, ultrasonic sensing, display, buzzer, camera, or other non-line-following features.
- Uploading firmware or using connected hardware as verification.
- Changing the control tick rate, PWM carrier, or Timer1 prescaler.

## Documentation

Update documentation in the same change when behavior or constraints change:

- Human-facing project usage belongs in `README.md` in Chinese.
- Agent-facing work rules belong in `AGENTS.md` in English.
- Significant architectural decisions belong in `docs/decisions/` as ADRs.
- Detailed hardware assumptions, wiring, validation, and calibration notes belong in `docs/line-follower-plan-and-spec.md`.

Do not document obvious code. Document why a hardware or timing choice exists, what risk it controls, and what verification is required.
