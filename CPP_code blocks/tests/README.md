# PID C++ unit tests

Self-contained: builds into `tests/build/` only. Mirrors the C suite and adds
a bit-for-bit cross-check against `../../c_code blocks/pid.c`, which it
compiles itself.

    mingw32-make test    build and run
    mingw32-make demo    build and run ../example/main.cpp
    mingw32-make clean
