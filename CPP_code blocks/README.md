# PID controller (C++20, header-only)

Copy `pid.hpp` into any project. Depends on `<cstdint>`, `<limits>` and `<type_traits>`
only. No allocation, no exceptions, no RTTI, usable in constant expressions.

    pid.hpp             pid::Config, pid::Controller (compute), pid::Runner (non-blocking)
    example/main.cpp    free-running loop against a mock plant
    tests/              unit tests, self-contained (`mingw32-make test` from tests/);
                        cross-checks both classes bit-for-bit against ../c_code blocks

## Use

    // tick: any uint32_t() millisecond counter, e.g. pid_tick_ms from the C
    // module's platform/ files, HAL_GetTick, millis, or an engine's own hook
    pid::Runner ctl({.kp = 2, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f,
                     .dt = 0, .outMin = 0, .outMax = 100}, 60000, &pid_tick_ms);

    float out = ctl.run(setpoint, measurement);   // every main-loop pass
    if (ctl.updated()) { /* a sample ran this pass: check ctl.fault(), then use out */ }
    ctl.reset();                                   // clear state, re-arm

`run()` never waits: one tick read, compute only when `sampleMs` has elapsed,
real elapsed time passed in as `dt`, held output otherwise. The 32-bit tick
wrap is handled. `sampleMs` can be 0 (every tick) up to minutes.

Scheduling is phase-locked to the sample grid, so the average period is
exactly `sampleMs` whatever the loop latency. A stall longer than a period
produces one compute with the true elapsed `dt`, never a catch-up burst, and
the next sample lands back on the grid.

`pid::Controller` on its own is the bare compute step with `Config::dt` as the
interval, for callers already on a fixed-rate timer.

Arithmetic is single-precision `float`, in the same order as the C module, so
both produce identical results. `tau`: 0.0 = unfiltered derivative, 1.0 =
derivative frozen at zero.

## Faults

Every compute sets `fault()`; check it whenever `updated()` is true. The
values and behaviour are the same as the C module's `pid->fault`.

    pid::Fault::None      the sample was computed
    pid::Fault::Config    a gain or limit is NaN/infinite, outMin > outMax,
                          tau outside 0..1, dt <= 0, or the Runner has no tick
    pid::Fault::Input     setpoint or measurement is NaN or infinite
    pid::Fault::Numeric   the arithmetic overflowed to a non-finite value

On a fault nothing in the controller state changes and `output()` stays at
its last good value, so a NaN can never reach the actuator. The fault is not
latched: the next good sample clears it. Holding the output is not a safe
state by itself; the application decides what to do. A `Runner` built without
a tick function never computes and reports `Fault::Config`.

## Coding standard

`pid.hpp` follows the same rules as the C module, in their MISRA C++:2023
form: braces on every body, every `if ... else if` closed with `else`, one
exit per function, explicit comparisons instead of conversions to `bool`,
unsigned literals suffixed, no casts of compound expressions, no exceptions,
RTTI or dynamic memory, a fixed-size `pid::float32_t`, and `[[nodiscard]]` on
every pure accessor.

No MISRA C++ checker was available, so unlike the C module this has not been
verified rule by rule; the rules above were applied by hand. What was run:

    g++ 16 -std=c++20 and -std=c++23, -Wall -Wextra -Wpedantic -Wconversion
        -Wsign-conversion -Wdouble-promotion -Wshadow -Wold-style-cast
        -Wuseless-cast -Wcast-qual -Weffc++ -Werror        clean
    cppcheck 2.20 --enable=all                             clean

Two points to settle before a formal MISRA C++:2023 claim: that standard
targets C++17, while this header needs C++20 (`constexpr` member functions
that modify state, designated initialisers in the examples), and `TickFn` is
a plain function pointer because the tick source is a C function.

The unit tests reach 100 % line and 100 % branch coverage of `pid.hpp`
(gcov, `-O0 -fno-inline`) and cross-check it bit-for-bit against the C
module, including with bad inputs and bad tuning injected.

As with the C module, this supports an ISO 26262 or IEC 61508 project but is
not a certified or "compliant" component; those standards assess the whole
development process.
