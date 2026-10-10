#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Fixed-size floating type (MISRA C:2012 Dir 4.6). IEEE 754 single precision.
typedef float pid_float32_t;

typedef enum {
    PID_IDLE = 0,   /* no tick latched yet; the first Run() arms the timer */
    PID_ARMED       /* waiting for sampleMs to elapse, output held */
} PIDState;

// Result of the most recent compute. Not latched: each compute overwrites it.
// On any fault the controller state is left untouched and the output is held
// at its last good value; the application decides what its safe state is.
typedef enum {
    PID_FAULT_NONE = 0,
    PID_FAULT_CONFIG,   /* gains/limits not finite, outMin > outMax, tau outside 0..1, dt <= 0 */
    PID_FAULT_INPUT,    /* setpoint or measurement is NaN or infinite */
    PID_FAULT_NUMERIC   /* the computation overflowed to a non-finite value */
} PIDFault;

typedef struct {
    // Tuning Gains
    pid_float32_t Kp;
    pid_float32_t Ki;
    pid_float32_t Kd;

    // Derivative low-pass filter coefficient (0.0 to 1.0)
    // 0.0 = no filtering, higher values = heavier filtering,
    // 1.0 = derivative term frozen at zero
    pid_float32_t tau;

    // Actuator Limits (Anti-windup)
    pid_float32_t outMin;
    pid_float32_t outMax;

    // Sample time (in seconds). Read by PIDController_Update;
    // PIDController_Run overwrites it with the measured interval.
    pid_float32_t dt;

    // Sample period for PIDController_Run, milliseconds.
    // 0 = compute on every call that sees the tick advance.
    uint32_t sampleMs;

    // Controller State Variables
    pid_float32_t integrator;
    pid_float32_t prevError;
    pid_float32_t prevMeasurement;
    pid_float32_t differentiator;

    // Run() state machine
    PIDState state;
    uint32_t lastTick;      /* sample-grid anchor used for scheduling */
    uint32_t lastRun;       /* tick of the last compute; dt is measured from here */
    pid_float32_t output;   /* last good output, held between samples and on a fault */
    uint8_t updated;        /* 1 on the call that ran a compute, else 0 */
    PIDFault fault;         /* result of the most recent compute */
} PIDController;

// Clears controller state, the state machine and the fault. Tuning fields
// are kept. A NULL pid is ignored.
void PIDController_Init(PIDController *pid);

// One PID step using pid->dt as the interval. No timing. Sets pid->fault;
// returns the new output, or the held output on a fault. Returns 0 for NULL.
pid_float32_t PIDController_Update(PIDController *pid, pid_float32_t setpoint, pid_float32_t measurement);

// Non-blocking. Call from the main loop as often as you like. Computes only
// when sampleMs has elapsed on pid_tick_ms(); otherwise returns the held
// output immediately. Never waits. Check pid->fault whenever pid->updated
// is set. Returns 0 for NULL.
pid_float32_t PIDController_Run(PIDController *pid, pid_float32_t setpoint, pid_float32_t measurement);

#ifdef __cplusplus
}
#endif

#endif // PID_H
