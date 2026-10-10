# PID controller blocks

A non-blocking PID controller in C and C++: derivative on measurement with a
low-pass filter, trapezoidal integral, anti-windup, fault reporting. It runs
on its own sample period from a free-running main loop and never waits.

    c_code blocks/     C module: pid.h + pid.c, platform tick providers, example, unit tests
                       (MISRA C:2012)
    CPP_code blocks/   C++20 header-only port: pid.hpp, example, unit tests
                       (the tests also cross-check it against the C module)

Each folder has its own README with usage, the fault table and the coding
standard notes. Each `tests/` folder is self-contained; run `mingw32-make test`
(or `make test`) inside it.

The C module is also packaged for the Arduino IDE as **PIDNonBlocking**:
https://github.com/GEORGE-MULTITECH-SYSTEMS/PID-Non-Blocking-

Neither module is certified to ISO 26262 or IEC 61508; see the READMEs for
what was and was not verified.

## License

MIT, see [LICENSE](LICENSE).
