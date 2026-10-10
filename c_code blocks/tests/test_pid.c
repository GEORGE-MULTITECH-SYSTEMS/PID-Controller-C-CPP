#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include "pid.h"
#include "pid_tick_stub.h"

static int g_fail = 0, g_pass = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (cond) { g_pass++; }                                            \
        else { g_fail++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(fabsf((a) - (b)) <= (eps))

static PIDController make(float kp, float ki, float kd, float tau, float dt, float lo, float hi) {
    PIDController p;
    p.Kp = kp; p.Ki = ki; p.Kd = kd; p.tau = tau; p.dt = dt; p.outMin = lo; p.outMax = hi;
    p.integrator = 99.f; p.prevError = 99.f; p.prevMeasurement = 99.f; p.differentiator = 99.f;
    PIDController_Init(&p);
    return p;
}

static void test_init_clears_state(void) {
    printf("init clears state\n");
    PIDController p = make(1, 1, 1, 0.5f, 0.1f, -1, 1);
    CHECK(p.integrator == 0.f);
    CHECK(p.prevError == 0.f);
    CHECK(p.prevMeasurement == 0.f);
    CHECK(p.differentiator == 0.f);
    CHECK(p.Kp == 1.f && p.Ki == 1.f && p.Kd == 1.f);
}

static void test_proportional_only(void) {
    printf("proportional only\n");
    PIDController p = make(2, 0, 0, 0, 0.01f, -100, 100);
    CHECK_NEAR(PIDController_Update(&p, 10, 0), 20.f, 1e-5f);
    CHECK_NEAR(PIDController_Update(&p, 10, 4), 12.f, 1e-5f);
    CHECK_NEAR(PIDController_Update(&p, 0, 4), -8.f, 1e-5f);
}

static void test_integral_is_trapezoidal(void) {
    printf("integral is trapezoidal on (error + prevError)\n");
    PIDController p = make(0, 1, 0, 0, 0.1f, -100, 100);
    CHECK_NEAR(PIDController_Update(&p, 1, 0), 0.05f, 1e-6f);
    CHECK_NEAR(PIDController_Update(&p, 1, 0), 0.15f, 1e-6f);
    CHECK_NEAR(PIDController_Update(&p, 1, 0), 0.25f, 1e-6f);
    CHECK_NEAR(p.integrator, 0.25f, 1e-6f);
}

static void test_integrator_clamped_to_output_limits(void) {
    printf("integrator clamps at outMin/outMax and recovers\n");
    PIDController p = make(0, 100, 0, 0, 0.1f, -1, 1);
    for (int i = 0; i < 10; i++) PIDController_Update(&p, 10, 0);
    CHECK(p.integrator <= 1.f);
    CHECK_NEAR(p.integrator, 1.f, 1e-6f);
    for (int i = 0; i < 10; i++) PIDController_Update(&p, -10, 0);
    CHECK_NEAR(p.integrator, -1.f, 1e-6f);
}

static void test_output_clamped(void) {
    printf("total output clamps at outMin/outMax\n");
    PIDController p = make(100, 0, 0, 0, 0.01f, -5, 5);
    CHECK_NEAR(PIDController_Update(&p, 10, 0), 5.f, 0.f);
    CHECK_NEAR(PIDController_Update(&p, -10, 0), -5.f, 0.f);
}

static void test_derivative_on_measurement_no_setpoint_kick(void) {
    printf("derivative acts on measurement only (tau=0 is unfiltered)\n");
    PIDController p = make(0, 0, 1, 0, 0.1f, -100, 100);
    CHECK_NEAR(PIDController_Update(&p, 0, 0), 0.f, 1e-6f);
    CHECK_NEAR(PIDController_Update(&p, 50, 0), 0.f, 1e-6f);
    CHECK_NEAR(PIDController_Update(&p, 50, 1), -10.f, 1e-4f);
    CHECK_NEAR(PIDController_Update(&p, 50, 1), 0.f, 1e-6f);
}

static void test_derivative_filter(void) {
    printf("derivative low-pass: d = tau*d_prev + (1-tau)*raw\n");
    PIDController p = make(0, 0, 1, 0.5f, 0.1f, -100, 100);
    PIDController_Update(&p, 0, 0);
    CHECK_NEAR(PIDController_Update(&p, 0, 1), -5.f, 1e-4f);
    CHECK_NEAR(PIDController_Update(&p, 0, 1), -2.5f, 1e-4f);
    CHECK_NEAR(PIDController_Update(&p, 0, 1), -1.25f, 1e-4f);
}

static void test_tau_one_disables_derivative(void) {
    printf("tau=1.0 freezes the derivative at zero (header comment says the opposite)\n");
    PIDController p = make(0, 0, 1, 1.0f, 0.1f, -100, 100);
    PIDController_Update(&p, 0, 0);
    CHECK_NEAR(PIDController_Update(&p, 0, 1), 0.f, 1e-6f);
    CHECK_NEAR(PIDController_Update(&p, 0, 5), 0.f, 1e-6f);
}

static void test_deterministic(void) {
    printf("identical inputs give identical outputs\n");
    PIDController a = make(2, 0.5f, 0.1f, 0.2f, 0.01f, -10, 10);
    PIDController b = a;
    float pv = 0.f;
    for (int i = 0; i < 50; i++) {
        float oa = PIDController_Update(&a, 100, pv);
        float ob = PIDController_Update(&b, 100, pv);
        CHECK(oa == ob);
        pv += oa * 0.5f;
    }
}

static void test_demo_tuning_stays_bounded(void) {
    printf("example/main.c tuning: output always within limits (loop limit-cycles, does not converge)\n");
    PIDController p = make(2, 0.5f, 0.1f, 0.2f, 0.01f, -10, 10);
    float pv = 0.f;
    int bounded = 1;
    for (int i = 0; i < 200; i++) {
        float out = PIDController_Update(&p, 100, pv);
        if (out < -10.f || out > 10.f) bounded = 0;
        pv += out * 0.5f;
    }
    CHECK(bounded);
}

static void test_closed_loop_converges_with_sane_tuning(void) {
    printf("closed loop converges with Kp=1 Ki=0.5 Kd=0 on the 0.5-gain plant\n");
    PIDController p = make(1, 0.5f, 0, 0, 0.01f, -10, 10);
    float pv = 0.f, sp = 100.f;
    for (int i = 0; i < 5000; i++) pv += PIDController_Update(&p, sp, pv) * 0.5f;
    CHECK(fabsf(pv - sp) < 0.01f);
    CHECK(fabsf(p.integrator) < 1e-3f);
}

/* ---- PIDController_Run: non-blocking state machine on the stub tick ---- */

static PIDController makeRun(float kp, float ki, float kd, float lo, float hi, uint32_t sampleMs) {
    PIDController p = make(kp, ki, kd, 0, 0.01f, lo, hi);
    p.sampleMs = sampleMs;
    PIDController_Init(&p);
    return p;
}

static void test_run_init_state(void) {
    printf("Run: Init leaves IDLE, output clamped zero, updated=0\n");
    PIDController p = makeRun(1, 0, 0, 20, 80, 1000);
    CHECK(p.state == PID_IDLE);
    CHECK(p.output == 20.f);
    CHECK(p.updated == 0);
    PIDController p2 = makeRun(1, 0, 0, -5, 5, 1000);
    CHECK(p2.output == 0.f);
}

static void test_run_first_call_arms_only(void) {
    printf("Run: first call latches the tick, computes nothing\n");
    pid_tick_stub_set(5000);
    PIDController p = makeRun(2, 0, 0, -100, 100, 100);
    CHECK(PIDController_Run(&p, 10, 0) == 0.f);
    CHECK(p.state == PID_ARMED);
    CHECK(p.lastTick == 5000);
    CHECK(p.updated == 0);
    CHECK(p.prevError == 0.f);
}

static void test_run_holds_until_period(void) {
    printf("Run: holds output and does not compute before sampleMs\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(2, 0, 0, -100, 100, 100);
    PIDController_Run(&p, 10, 0);
    for (uint32_t t = 1; t < 100; t += 7) {
        pid_tick_stub_set(t);
        CHECK(PIDController_Run(&p, 10, 0) == 0.f);
        CHECK(p.updated == 0);
    }
    pid_tick_stub_set(100);
    CHECK_NEAR(PIDController_Run(&p, 10, 0), 20.f, 1e-5f);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 0.1f, 1e-6f);
    pid_tick_stub_set(150);
    CHECK_NEAR(PIDController_Run(&p, 50, 0), 20.f, 1e-5f);
    CHECK(p.updated == 0);
}

static void test_run_one_minute_period(void) {
    printf("Run: 60 s period, 10 ms loop: exactly one compute per minute\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(0, 1, 0, -1000, 1000, 60000);
    PIDController_Run(&p, 1, 0);
    int computes = 0;
    for (uint32_t t = 10; t <= 300000; t += 10) {
        pid_tick_stub_set(t);
        PIDController_Run(&p, 1, 0);
        computes += p.updated;
    }
    CHECK(computes == 5);
    CHECK_NEAR(p.dt, 60.f, 1e-4f);
    CHECK_NEAR(p.integrator, 0.5f * 60.f + 4.f * 60.f, 1e-2f);
}

static void test_run_uses_measured_interval_and_stays_on_grid(void) {
    printf("Run: late sample uses the real dt, next sample returns to the grid\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(0, 1, 0, -100, 100, 1000);
    PIDController_Run(&p, 1, 0);
    pid_tick_stub_set(1237);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 1.237f, 1e-6f);
    CHECK(p.lastTick == 1000);
    pid_tick_stub_set(1999);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 0);
    pid_tick_stub_set(2000);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 0.763f, 1e-6f);
    CHECK(p.lastTick == 2000);
}

static void test_run_stall_gives_one_compute_no_burst(void) {
    printf("Run: a 2.5-period stall gives exactly one compute with dt=2.5 periods, no burst\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(0, 1, 0, -100, 100, 1000);
    PIDController_Run(&p, 1, 0);
    pid_tick_stub_set(2500);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 2.5f, 1e-6f);
    CHECK(p.lastTick == 2000);
    pid_tick_stub_set(2501);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 0);
    pid_tick_stub_set(2999);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 0);
    pid_tick_stub_set(3000);
    PIDController_Run(&p, 1, 0);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 0.5f, 1e-6f);
}

static void test_run_average_period_is_exact_under_jitter(void) {
    printf("Run: with loop latency jitter the sample count over 1000 periods is exactly 1000\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(0, 1, 0, -1e9f, 1e9f, 100);
    PIDController_Run(&p, 1, 0);
    uint32_t seed = 99u, t = 0;
    int computes = 0;
    float dtSum = 0.f;
    while (t < 100000u) {
        seed = seed * 1664525u + 1013904223u;
        t += 1 + (seed >> 8) % 23;
        pid_tick_stub_set(t);
        PIDController_Run(&p, 1, 0);
        if (p.updated) { computes++; dtSum += p.dt; }
    }
    CHECK(computes == 1000);
    CHECK_NEAR(dtSum / (float)computes, 0.1f, 1e-3f);
}

static void test_run_tick_wraparound(void) {
    printf("Run: survives the 32-bit tick wrap\n");
    pid_tick_stub_set(0xFFFFFFF0u);
    PIDController p = makeRun(2, 0, 0, -100, 100, 100);
    PIDController_Run(&p, 10, 0);
    pid_tick_stub_set(0xFFFFFFFFu);
    CHECK(p.updated == 0 && PIDController_Run(&p, 10, 0) == 0.f);
    pid_tick_stub_set(84u);
    CHECK_NEAR(PIDController_Run(&p, 10, 0), 20.f, 1e-5f);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 0.1f, 1e-6f);
}

static void test_run_period_zero_every_tick(void) {
    printf("Run: sampleMs=0 computes whenever the tick advanced, never on a repeat tick\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(2, 0, 0, -100, 100, 0);
    PIDController_Run(&p, 10, 0);
    PIDController_Run(&p, 10, 0);
    CHECK(p.updated == 0);
    pid_tick_stub_set(3);
    CHECK_NEAR(PIDController_Run(&p, 10, 0), 20.f, 1e-5f);
    CHECK(p.updated == 1);
    CHECK_NEAR(p.dt, 0.003f, 1e-7f);
}

static void test_run_reset_rearms(void) {
    printf("Run: Init after running clears state and re-arms on the next call\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(0, 1, 0, -100, 100, 100);
    PIDController_Run(&p, 1, 0);
    pid_tick_stub_set(100);
    PIDController_Run(&p, 1, 0);
    CHECK(p.integrator != 0.f);
    PIDController_Init(&p);
    CHECK(p.state == PID_IDLE && p.integrator == 0.f && p.output == 0.f);
    pid_tick_stub_set(5000);
    PIDController_Run(&p, 1, 0);
    CHECK(p.state == PID_ARMED && p.lastTick == 5000 && p.updated == 0);
}

static void test_run_matches_update_on_exact_grid(void) {
    printf("Run: on an exact tick grid it is bit-identical to Update with fixed dt\n");
    PIDController a = make(2, 0.5f, 0.1f, 0.2f, 0.1f, -10, 10);
    PIDController b = makeRun(2, 0.5f, 0.1f, -10, 10, 100);
    b.tau = 0.2f;
    pid_tick_stub_set(0);
    PIDController_Run(&b, 100, 0);
    float pv = 0.f;
    int same = 1;
    for (int i = 1; i <= 100; i++) {
        pid_tick_stub_set((uint32_t)i * 100u);
        float oa = PIDController_Update(&a, 100, pv);
        float ob = PIDController_Run(&b, 100, pv);
        if (!b.updated || oa != ob) same = 0;
        pv += oa * 0.5f;
    }
    CHECK(same);
}

static void test_run_never_blocks(void) {
    printf("Run: one tick read per call, returns without the clock advancing\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(1, 0, 0, -100, 100, 60000);
    pid_tick_stub_reset_calls();
    for (int i = 0; i < 1000; i++) PIDController_Run(&p, 1, 0);
    CHECK(pid_tick_stub_calls() == 1000);
    CHECK(p.updated == 0);
}

static void test_null_is_ignored(void) {
    printf("fault: NULL controller is ignored and returns 0\n");
    PIDController_Init(NULL);
    CHECK(PIDController_Update(NULL, 1, 0) == 0.f);
    CHECK(PIDController_Run(NULL, 1, 0) == 0.f);
}

static void test_fault_config(void) {
    printf("fault: invalid configuration holds the output and leaves state alone\n");
    PIDController p = make(1, 1, 0, 0, 0.1f, -100, 100);
    float good = PIDController_Update(&p, 10, 0);
    CHECK(p.fault == PID_FAULT_NONE);
    float integ = p.integrator;

    p.dt = 0.f;
    CHECK(PIDController_Update(&p, 50, 0) == good);
    CHECK(p.fault == PID_FAULT_CONFIG);
    p.dt = -0.1f;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.dt = NAN;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.dt = 0.1f;

    p.tau = 1.5f;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.tau = -0.1f;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.tau = NAN;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.tau = 0.f;

    p.outMin = 200.f;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.outMin = -INFINITY;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.outMin = -100.f;
    p.outMax = INFINITY;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.outMax = 100.f;

    p.Kp = NAN;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.Kp = 1.f; p.Ki = INFINITY;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.Ki = 1.f; p.Kd = NAN;
    CHECK(PIDController_Update(&p, 50, 0) == good && p.fault == PID_FAULT_CONFIG);
    p.Kd = 0.f;

    CHECK(p.integrator == integ);
    CHECK(p.prevError == 10.f);
    PIDController_Update(&p, 10, 0);
    CHECK(p.fault == PID_FAULT_NONE);
}

static void test_fault_input(void) {
    printf("fault: NaN or infinite input holds the output and clears on recovery\n");
    PIDController p = make(1, 1, 1, 0.2f, 0.1f, -100, 100);
    float good = PIDController_Update(&p, 10, 1);
    float integ = p.integrator, diff = p.differentiator;

    CHECK(PIDController_Update(&p, 10, NAN) == good && p.fault == PID_FAULT_INPUT);
    CHECK(PIDController_Update(&p, NAN, 1) == good && p.fault == PID_FAULT_INPUT);
    CHECK(PIDController_Update(&p, 10, INFINITY) == good && p.fault == PID_FAULT_INPUT);
    CHECK(PIDController_Update(&p, -INFINITY, 1) == good && p.fault == PID_FAULT_INPUT);
    CHECK(p.integrator == integ && p.differentiator == diff);
    CHECK(p.prevMeasurement == 1.f && p.prevError == 9.f);
    CHECK(p.output == good);

    float next = PIDController_Update(&p, 10, 1);
    CHECK(p.fault == PID_FAULT_NONE);
    CHECK(isfinite(next) && next >= -100.f && next <= 100.f);
}

static void test_fault_numeric(void) {
    printf("fault: overflow in the computation holds the output\n");
    PIDController p = make(0, 0, FLT_MAX, 0, 1e-6f, -100, 100);
    float good = PIDController_Update(&p, 0, 0);
    CHECK(p.fault == PID_FAULT_NONE);
    CHECK(PIDController_Update(&p, 0, FLT_MAX) == good);
    CHECK(p.fault == PID_FAULT_NUMERIC);
    CHECK(p.differentiator == 0.f && p.prevMeasurement == 0.f);

    /* error overflows to +inf; a zero gain times inf is NaN */
    PIDController q = make(1, 0, 0, 0, 0.1f, -100, 100);
    CHECK(PIDController_Update(&q, FLT_MAX, -FLT_MAX) == 0.f);
    CHECK(q.fault == PID_FAULT_NUMERIC && q.integrator == 0.f);

    PIDController r = make(0, 1, 0, 0, 0.1f, -100, 100);
    CHECK(PIDController_Update(&r, FLT_MAX, -FLT_MAX) == 0.f);
    CHECK(r.fault == PID_FAULT_NUMERIC && r.integrator == 0.f && r.prevError == 0.f);
}

static void test_run_reports_fault(void) {
    printf("Run: a faulted sample is flagged, output held, schedule kept\n");
    pid_tick_stub_set(0);
    PIDController p = makeRun(1, 0, 0, -100, 100, 10);
    PIDController_Run(&p, 5, 0);
    pid_tick_stub_advance(10);
    CHECK(PIDController_Run(&p, 5, 0) == 5.f && p.updated == 1 && p.fault == PID_FAULT_NONE);
    pid_tick_stub_advance(10);
    CHECK(PIDController_Run(&p, 5, NAN) == 5.f);
    CHECK(p.updated == 1 && p.fault == PID_FAULT_INPUT);
    pid_tick_stub_advance(5);
    CHECK(PIDController_Run(&p, 7, 0) == 5.f && p.updated == 0 && p.fault == PID_FAULT_INPUT);
    pid_tick_stub_advance(5);
    CHECK(PIDController_Run(&p, 7, 0) == 7.f && p.updated == 1 && p.fault == PID_FAULT_NONE);
}

int main(void) {
    test_init_clears_state();
    test_proportional_only();
    test_integral_is_trapezoidal();
    test_integrator_clamped_to_output_limits();
    test_output_clamped();
    test_derivative_on_measurement_no_setpoint_kick();
    test_derivative_filter();
    test_tau_one_disables_derivative();
    test_deterministic();
    test_demo_tuning_stays_bounded();
    test_closed_loop_converges_with_sane_tuning();
    test_run_init_state();
    test_run_first_call_arms_only();
    test_run_holds_until_period();
    test_run_one_minute_period();
    test_run_uses_measured_interval_and_stays_on_grid();
    test_run_stall_gives_one_compute_no_burst();
    test_run_average_period_is_exact_under_jitter();
    test_run_tick_wraparound();
    test_run_period_zero_every_tick();
    test_run_reset_rearms();
    test_run_matches_update_on_exact_grid();
    test_run_never_blocks();
    test_null_is_ignored();
    test_fault_config();
    test_fault_input();
    test_fault_numeric();
    test_run_reports_fault();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
