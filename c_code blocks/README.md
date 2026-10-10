# PID controller (C)

Drop-in module for any C or C++ project. No allocation, no globals, no
dependency beyond `<stdint.h>`.

    pid.h, pid.c            the controller
    pid_tick.h              platform-neutral tick interface: uint32_t pid_tick_ms(void)
    platform/               one tick provider per platform; compile exactly one
                            pid_tick_c11.c       any C11 libc (host, MinGW, Linux)
                            pid_tick_stm32.c     STM32 HAL, HAL_GetTick
                            pid_tick_arduino.cpp Arduino / ESP32 core, millis
    example/main.c          non-blocking loop against a mock plant
    tests/                  unit tests, self-contained, with their own stub tick
                            (`mingw32-make test` from tests/)

## Use

Fill the gains, `tau`, `outMin`, `outMax` and `sampleMs`, call
`PIDController_Init`, then from the main loop:

    float out = PIDController_Run(&pid, setpoint, measurement);
    if (pid.updated) { /* a new sample was computed this pass */ }

`PIDController_Run` never waits. It reads `pid_tick_ms()` once, computes only
when `sampleMs` has elapsed, passes the real elapsed time in as `dt`, and
otherwise returns the held output. `sampleMs` can be anything from 0 (every
tick) to minutes; the 32-bit tick wrap is handled.

Scheduling is phase-locked to the sample grid, so the average period is
exactly `sampleMs` whatever the loop latency. A stall longer than a period
produces one compute with the true elapsed `dt`, never a catch-up burst, and
the next sample lands back on the grid.

`PIDController_Update` is the bare compute step with `pid->dt` as the
interval, for callers that already run on a fixed-rate timer.

`tau` is the derivative low-pass coefficient: 0.0 = unfiltered, 1.0 = the
derivative term is frozen at zero.

## Faults

Every compute sets `pid->fault`; check it whenever `pid->updated` is set.

    PID_FAULT_NONE      the sample was computed
    PID_FAULT_CONFIG    a gain or limit is NaN/infinite, outMin > outMax,
                        tau outside 0..1, or dt <= 0
    PID_FAULT_INPUT     setpoint or measurement is NaN or infinite
    PID_FAULT_NUMERIC   the arithmetic overflowed to a non-finite value

On a fault nothing in the controller state changes and the output stays at
its last good value, so a NaN can never reach the actuator. The fault is not
latched: the next good sample clears it. Holding the output is not a safe
state by itself; the application decides what to do (shut the actuator off,
count faults, trip after N in a row). A NULL controller pointer is ignored
and the functions return 0.

## Coding standard

`pid.h`, `pid.c`, `pid_tick.h` and `platform/pid_tick_stm32.c` are written to
MISRA C:2012 (C99). Checked with cppcheck 2.20 and its MISRA addon, plus
`gcc -std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion
-Wshadow -Wcast-qual -Werror`. One deviation remains:

    Rule 8.7 (advisory)  PIDController_Init/Update/Run have external linkage
                         although the checker sees no caller in another file.
                         They are the module's public API.

The unit tests reach 100 % line and 100 % branch coverage of `pid.c`
(gcov, `-O0`). `platform/pid_tick_c11.c` (uses `<time.h>`), `example/` and
`tests/` are host-only and outside the MISRA scope.

This is what the code can contribute to an ISO 26262 or IEC 61508 project. It
is not a certified or "compliant" component: those standards assess the
whole development process (safety requirements, hazard analysis, traceability,
independent review, qualified tools, target-compiler testing), and the
cppcheck addon covers only part of the MISRA rule set and is not a qualified
checker. Treat the module as an input to your own safety case.
