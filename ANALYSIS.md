# Refloat Codebase Analysis

## Project Overview

**Refloat** is a VESC package for self-balancing electric skateboards (e.g., Onewheel-style boards). It runs as embedded firmware on VESC motor controllers, implementing real-time balance control through IMU sensor fusion, PID control, and motor current management. Licensed under GPLv3.

**Current Version**: 1.2.1

## Architecture

### Language & Build System
- **Language**: C (embedded), QML (VESC Tool UI), Lisp (VESC package scripting)
- **Toolchain**: `gcc-arm-embedded` v13+, cross-compiled for STM32 microcontrollers
- **Build**: GNU Make, `vesc_tool` for `.vescpkg` packaging
- **CI**: GitHub Actions with `clang-format` v18 (via pre-commit) and Nix-based builds
- **Total**: ~9,168 lines across 61 C source/header files

### Core Structure

```
src/
├── main.c              (2,465 lines - main control loop, state machine, command handling)
├── balance_filter.c    (Mahony AHRS quaternion filter for pitch/roll estimation)
├── pid.c               (PID controller with directional brake scaling)
├── motor_control.c     (Motor current/brake/tone output)
├── motor_data.c        (Motor telemetry: ERPM, voltage, temperature, duty cycle)
├── state.c             (State machine: DISABLED -> STARTUP -> READY -> RUNNING)
├── footpad_sensor.c    (ADC-based footpad pressure detection)
├── imu.c               (IMU data processing: pitch, roll, yaw rates)
├── data.h              (Central Data struct aggregating all subsystem state)
├── leds.c              (1,230 lines - LED strip animations and status)
├── lcm.c               (LED Controller Module serial protocol)
├── atr.c               (Adaptive Torque Response)
├── torque_tilt.c       (Torque-based tilt compensation)
├── turn_tilt.c         (Yaw/turn-based tilt)
├── brake_tilt.c        (Braking tilt compensation)
├── booster.c           (Speed boost current)
├── remote.c            (Remote input handling)
├── haptic_feedback.c   (Motor-based haptic alerts)
├── charging.c          (Charge state detection)
├── bms.c               (Battery Management System integration)
├── data_recorder.c     (Ride data recording to circular buffer)
├── konami.c            (Konami-code footpad sequences for feature toggles)
├── conf/               (Auto-generated config parser from settings.xml)
└── lib/circular_buffer.c
```

### Design Patterns

1. **Monolithic Data struct** (`data.h:44-136`): A single `Data` struct holds all subsystem state, passed by pointer throughout. Pragmatic for embedded C - avoids dynamic dispatch but couples everything together.

2. **Module pattern**: Each subsystem follows `*_init()` / `*_configure()` / `*_update()` / `*_reset()` conventions consistently.

3. **Timer-based state transitions**: Uses relative timers (`timer_older`, `timer_older_ms`) extensively for debouncing sensor inputs and fault detection.

4. **Real-time control loop**: The main thread runs at a configurable frequency (typically 800-1000Hz), performing IMU read -> balance filter -> fault check -> setpoint calculation -> PID -> motor output in each iteration.

5. **Thread model**: Main thread (1536B stack) for real-time control, auxiliary thread (1024B stack) for configuration/telemetry/alerts.

## Key Findings

### Strengths

1. **Clean modular decomposition**: Despite being embedded C, each concern (PID, sensors, LEDs, tilt algorithms) is well-isolated with clear interfaces.

2. **Comprehensive safety system**: Multi-layered fault detection (`check_faults()` at `main.c:352-503`) covers pitch/roll angles, footpad sensors, voltage, temperature, duty cycle, and wheelslip - all with configurable delays to avoid false triggers.

3. **Good backwards compatibility**: Handles migration from older configurations gracefully (e.g., `main.c:196-202` Mahony KP migration).

4. **Proper resource management**: `stop()` function (`main.c:2398-2412`) terminates threads, clears callbacks, and frees memory.

5. **Consistent code style**: Enforced via `clang-format` v18 and pre-commit hooks.

6. **Memory safety**: All `malloc()` calls checked for NULL. No allocations in the hot path - all heap operations occur during initialization/configuration.

7. **Input validation**: All incoming commands are length-checked (`main.c:2148-2189`). No unsafe string functions found.

8. **Physics-informed PID**: Uses Rate P instead of derivative term ("Rate P works better with high Mahony KP" - `pid.h:27`).

### Potential Issues

#### Division by Zero Risk (main.c:171-173)
```c
// TODO handle division by zero
d->tiltback_variable_max_erpm =
    fabsf(d->float_conf.tiltback_variable_max / d->tiltback_variable);
```
If `tiltback_variable` is 0 (derived from `tiltback_variable / 1000`), this produces infinity/NaN. Explicitly marked as a TODO.

#### Large main.c (2,465 lines)
Contains the entire control loop, command protocol handling, flywheel mode, and beeper logic. Several functions could be extracted (e.g., flywheel logic, noted in TODO at `main.c:580`).

#### Magic Numbers
Several hardcoded thresholds without named constants:
- `main.c:271`: `d->motor.abs_erpm > 800` (RC move safety)
- `main.c:276`: `d->rc_counter == 500` and `d->rc_current_target > 2`
- `main.c:412-414`: `abs_erpm < 200`, `pitch > 14`, `setpoint < 30` (quickstop)
- `main.c:546`: `acceleration > 15` (wheelslip detection)

#### Buffer Size Uncertainty (main.c:1078)
```c
// TODO: The required buffer size is not provided by the confparser.
```
Config serialization buffer size isn't properly bounded by the generated parser.

#### No Unit Tests
No test files anywhere in the repository. Given this is safety-critical firmware (a person is standing on the board), the absence of tests for the PID controller, fault detection logic, and state machine is a notable gap. Testing is manual and hardware-dependent.

#### Tight Thread Stack Sizes
Main thread gets 1536 bytes (`main.c:2438`), aux thread 1024 bytes (`main.c:2444`). Deep call chains or large local variables could cause stack overflows on the embedded target.

### Minor Observations

- Typo in comment at `main.c:516`: "accumalete" should be "accumulate"
- `footpad_sensor.c` maps single-sensor configs to `FS_BOTH` rather than `FS_LEFT`/`FS_RIGHT` - deliberate simplification
- `balance_filter.c:38` uses `1.0 / sqrtf(x)` rather than a fast inverse square root - fine for correctness

## Dependencies

- **VESC C Library** (`vesc_c_if.h`): Hardware abstraction for motor control, IMU, GPIO, threading, memory
- **VESC Tool**: Build-time config code generation and package assembly
- **Standard C library**: `math.h`, `string.h`, `stdbool.h`, `stdint.h`
- No external runtime dependencies beyond the VESC firmware

## Summary

| Aspect | Rating | Notes |
|--------|--------|-------|
| Code Quality | Excellent | Well-modularized, consistent style, defensive |
| Documentation | Good | README, dev docs, config help text; no API docs |
| Testing | Weak | No unit/integration tests; hardware-only |
| Error Handling | Good | Comprehensive logging, NULL checks, graceful failures |
| Memory Safety | Good | All mallocs checked, fixed-size arrays, no leaks |
| Security | Good | Input validation present, no obvious vulnerabilities |
| Maintainability | Excellent | Clear modules, consistent patterns, contributor guide |
| Build System | Good | Reproducible Nix-based CI, local make support |

Refloat is a well-structured embedded C project with clean module boundaries, comprehensive safety mechanisms, and consistent coding style. The main areas for improvement are: adding unit tests (especially for safety-critical paths), extracting functionality from the oversized `main.c`, fixing the documented division-by-zero risk, and replacing magic numbers with named constants.
