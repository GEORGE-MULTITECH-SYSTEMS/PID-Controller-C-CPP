#ifndef PID_TICK_H
#define PID_TICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Monotonic millisecond counter. Free to wrap at 2^32; the PID subtracts
 * in uint32_t so the wrap is harmless. Implemented once per platform in
 * platform/pid_tick_<platform>.c, or by a stub in tests. */
uint32_t pid_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_TICK_H */
