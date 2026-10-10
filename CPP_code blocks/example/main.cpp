#include <cstdio>
#include "pid.hpp"

extern "C" {
#include "pid_tick.h"
}

// Non-blocking use: the main loop spins freely and calls run() on every pass.
// The runner computes only every sampleMs on the injected tick (here the C
// module's platform/pid_tick_c11.c) and holds its output in between.
int main() {
    pid::Runner ctl({.kp = 2.0f, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f, .dt = 0.01f, .outMin = -10.0f, .outMax = 10.0f},
                    10, &pid_tick_ms);

    const float setpoint = 100.0f;
    float current_pv = 0.0f;

    std::printf("Sample, Target, Current, Control_Output, dt(s), loop_passes_between_samples\n");

    int samples = 0;
    unsigned long passes = 0;
    while (samples < 20) {
        const float output = ctl.run(setpoint, current_pv);
        passes++;
        if (!ctl.updated()) {
            continue;
        }
        current_pv += output * 0.5f;
        samples++;
        std::printf("%d, %0.1f, %0.2f, %0.2f, %0.3f, %lu\n", samples, setpoint, current_pv, output, ctl.dt(), passes);
        passes = 0;
    }
    return 0;
}
