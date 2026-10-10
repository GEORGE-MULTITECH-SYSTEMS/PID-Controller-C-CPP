#include <float.h>
#include <stdbool.h>
#include <stddef.h>
#include "pid.h"
#include "pid_tick.h"

/* False for NaN and for either infinity. */
static bool isFinite(pid_float32_t v) {
    return (v >= -FLT_MAX) && (v <= FLT_MAX);
}

static pid_float32_t clampTo(pid_float32_t v, pid_float32_t lo, pid_float32_t hi) {
    pid_float32_t r = v;

    if (v > hi) {
        r = hi;
    } else if (v < lo) {
        r = lo;
    } else {
        /* in range */
    }
    return r;
}

static bool configValid(const PIDController *pid) {
    const bool finite = isFinite(pid->Kp) && isFinite(pid->Ki) && isFinite(pid->Kd) &&
                        isFinite(pid->outMin) && isFinite(pid->outMax) && isFinite(pid->dt);

    /* The comparisons are false for a NaN tau, so that is rejected too. */
    return finite && (pid->outMin <= pid->outMax) &&
           (pid->tau >= 0.0f) && (pid->tau <= 1.0f) && (pid->dt > 0.0f);
}

void PIDController_Init(PIDController *pid) {
    if (pid != NULL) {
        pid->integrator = 0.0f;
        pid->prevError = 0.0f;
        pid->prevMeasurement = 0.0f;
        pid->differentiator = 0.0f;

        pid->state = PID_IDLE;
        pid->lastTick = 0U;
        pid->lastRun = 0U;
        pid->output = clampTo(0.0f, pid->outMin, pid->outMax);
        pid->updated = 0U;
        pid->fault = PID_FAULT_NONE;
    }
}

pid_float32_t PIDController_Update(PIDController *pid, pid_float32_t setpoint, pid_float32_t measurement) {
    pid_float32_t output = 0.0f;

    if (pid != NULL) {
        /* Held value: returned unchanged if this step faults. */
        output = pid->output;

        if (!configValid(pid)) {
            pid->fault = PID_FAULT_CONFIG;
        } else if ((!isFinite(setpoint)) || (!isFinite(measurement))) {
            pid->fault = PID_FAULT_INPUT;
        } else {
            // 1. Calculate error
            const pid_float32_t error = setpoint - measurement;

            // 2. Proportional term
            const pid_float32_t proportional = pid->Kp * error;

            // 3. Integral term (trapezoidal), then anti-windup:
            //    clamp integrator to output limits
            pid_float32_t integrator = pid->integrator +
                                       (0.5f * pid->Ki * pid->dt * (error + pid->prevError));
            integrator = clampTo(integrator, pid->outMin, pid->outMax);

            // 4. Derivative term (with Low-Pass Filter to handle sensor noise)
            // Uses 'Derivative on Measurement' to prevent step-change spikes
            const pid_float32_t deltaMeasurement = measurement - pid->prevMeasurement;
            const pid_float32_t differentiator =
                (pid->tau * pid->differentiator) +
                ((1.0f - pid->tau) * (((-pid->Kd) * deltaMeasurement) / pid->dt));

            // 5. Total Output, clamped to actuator limits
            const pid_float32_t total =
                clampTo(proportional + integrator + differentiator, pid->outMin, pid->outMax);

            // 6. Commit state for the next iteration only if every result is
            //    a number; a NaN would otherwise pass straight through the clamp
            if (isFinite(integrator) && isFinite(differentiator) && isFinite(total)) {
                pid->integrator = integrator;
                pid->differentiator = differentiator;
                pid->prevError = error;
                pid->prevMeasurement = measurement;
                pid->output = total;
                pid->fault = PID_FAULT_NONE;
                output = total;
            } else {
                pid->fault = PID_FAULT_NUMERIC;
            }
        }
    }

    return output;
}

pid_float32_t PIDController_Run(PIDController *pid, pid_float32_t setpoint, pid_float32_t measurement) {
    pid_float32_t output = 0.0f;

    if (pid != NULL) {
        const uint32_t now = pid_tick_ms();
        pid->updated = 0U;

        if (pid->state == PID_IDLE) {
            pid->lastTick = now;
            pid->lastRun = now;
            pid->state = PID_ARMED;
        } else {
            const uint32_t elapsed = now - pid->lastTick;   /* uint32 wrap-safe */

            if ((elapsed != 0U) && (elapsed >= pid->sampleMs)) {
                /* Scheduling stays phase-locked to the sample grid (average
                 * period exactly sampleMs, one compute per call after a stall,
                 * next sample on the grid); dt is the true time since the
                 * previous compute. */
                const uint32_t sinceRun = now - pid->lastRun;

                if (pid->sampleMs != 0U) {
                    pid->lastTick = now - (elapsed % pid->sampleMs);
                } else {
                    pid->lastTick = now;
                }
                pid->dt = (pid_float32_t)sinceRun * 0.001f;
                pid->lastRun = now;
                (void)PIDController_Update(pid, setpoint, measurement);
                pid->updated = 1U;
            }
        }
        output = pid->output;
    }

    return output;
}
