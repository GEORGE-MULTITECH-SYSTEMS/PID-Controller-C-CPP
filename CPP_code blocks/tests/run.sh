#!/usr/bin/env sh
cd "$(dirname "$0")" && ${MAKE:-mingw32-make} test
