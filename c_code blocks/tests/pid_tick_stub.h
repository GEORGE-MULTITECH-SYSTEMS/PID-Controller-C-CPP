#ifndef PID_TICK_STUB_H
#define PID_TICK_STUB_H

#include <stdint.h>

/* Test clock: fully under the test's control. */
void     pid_tick_stub_set(uint32_t ms);
void     pid_tick_stub_advance(uint32_t ms);
uint32_t pid_tick_stub_calls(void);     /* how many times pid_tick_ms() was read */
void     pid_tick_stub_reset_calls(void);

#endif /* PID_TICK_STUB_H */
