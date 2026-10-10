# PID unit tests

Self-contained: everything builds into `tests/build/`, nothing is written
outside this folder and the sources in the parent folder are not modified.

    make test    build and run the unit tests
    make demo    build and run ../main.c
    make clean

Use `mingw32-make` on Windows. Override the compiler with `make CC=clang`.
