#include <stdio.h>
#include "pid.h"

/* Non-blocking use: the main loop spins freely and calls PIDController_Run
 * on every pass. The controller computes only every sampleMs on the platform
 * tick (platform/pid_tick_c11.c here) and holds its output in between. */
int main() {
    PIDController pid;

    pid.Kp = 2.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.1f;
    pid.tau = 0.2f;
    pid.outMin = -10.0f;
    pid.outMax = 10.0f;
    pid.sampleMs = 10;          /* 10 ms control period */

    PIDController_Init(&pid);

    float setpoint = 100.0f;
    float current_pv = 0.0f;

    printf("Sample, Target, Current, Control_Output, dt(s), loop_passes_between_samples\n");

    int samples = 0;
    unsigned long passes = 0;
    while (samples < 20) {
        float output = PIDController_Run(&pid, setpoint, current_pv);
        passes++;
        if (!pid.updated) {
            continue;           /* nothing to do this pass; other work goes here */
        }

        current_pv += output * 0.5f;
        samples++;
        printf("%d, %0.1f, %0.2f, %0.2f, %0.3f, %lu\n", samples, setpoint, current_pv, output, pid.dt, passes);
        passes = 0;
    }

    return 0;
}
