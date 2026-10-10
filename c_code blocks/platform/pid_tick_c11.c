#include <time.h>
#include "../pid_tick.h"

/* Host build: any C11 libc (MinGW UCRT, glibc, musl). */
uint32_t pid_tick_ms(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}
