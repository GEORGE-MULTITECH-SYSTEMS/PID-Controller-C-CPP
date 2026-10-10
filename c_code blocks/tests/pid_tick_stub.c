#include "pid_tick.h"
#include "pid_tick_stub.h"

static uint32_t g_now = 0;
static uint32_t g_calls = 0;

uint32_t pid_tick_ms(void) {
    g_calls++;
    return g_now;
}

void pid_tick_stub_set(uint32_t ms) { g_now = ms; }
void pid_tick_stub_advance(uint32_t ms) { g_now += ms; }
uint32_t pid_tick_stub_calls(void) { return g_calls; }
void pid_tick_stub_reset_calls(void) { g_calls = 0; }
