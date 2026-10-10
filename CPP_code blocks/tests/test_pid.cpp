#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include "pid.hpp"
extern "C" {
#include "pid.h"
#include "pid_tick.h"
#include "pid_tick_stub.h"
}

static int g_fail = 0, g_pass = 0;

#define CHECK(cond)                                                                            \
    do {                                                                                       \
        if (cond) { g_pass++; }                                                                \
        else { g_fail++; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); }      \
    } while (0)

#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

static pid::Controller make(float kp, float ki, float kd, float tau, float dt, float lo, float hi) {
    return pid::Controller({.kp = kp, .ki = ki, .kd = kd, .tau = tau, .dt = dt, .outMin = lo, .outMax = hi});
}

static void test_init_clears_state() {
    std::printf("construction and reset clear state\n");
    pid::Controller p = make(1, 1, 1, 0.5f, 0.1f, -1, 1);
    CHECK(p.integrator() == 0.f && p.prevError() == 0.f && p.prevMeasurement() == 0.f && p.differentiator() == 0.f);
    p.update(5, 1);
    p.update(5, 2);
    CHECK(p.integrator() != 0.f);
    p.reset();
    CHECK(p.integrator() == 0.f && p.prevError() == 0.f && p.prevMeasurement() == 0.f && p.differentiator() == 0.f);
    CHECK(p.config().kp == 1.f && p.config().ki == 1.f && p.config().kd == 1.f);
}

static void test_proportional_only() {
    std::printf("proportional only\n");
    pid::Controller p = make(2, 0, 0, 0, 0.01f, -100, 100);
    CHECK_NEAR(p.update(10, 0), 20.f, 1e-5f);
    CHECK_NEAR(p.update(10, 4), 12.f, 1e-5f);
    CHECK_NEAR(p.update(0, 4), -8.f, 1e-5f);
}

static void test_integral_is_trapezoidal() {
    std::printf("integral is trapezoidal on (error + prevError)\n");
    pid::Controller p = make(0, 1, 0, 0, 0.1f, -100, 100);
    CHECK_NEAR(p.update(1, 0), 0.05f, 1e-6f);
    CHECK_NEAR(p.update(1, 0), 0.15f, 1e-6f);
    CHECK_NEAR(p.update(1, 0), 0.25f, 1e-6f);
    CHECK_NEAR(p.integrator(), 0.25f, 1e-6f);
}

static void test_integrator_clamped_to_output_limits() {
    std::printf("integrator clamps at outMin/outMax and recovers\n");
    pid::Controller p = make(0, 100, 0, 0, 0.1f, -1, 1);
    for (int i = 0; i < 10; i++) p.update(10, 0);
    CHECK(p.integrator() <= 1.f);
    CHECK_NEAR(p.integrator(), 1.f, 1e-6f);
    for (int i = 0; i < 10; i++) p.update(-10, 0);
    CHECK_NEAR(p.integrator(), -1.f, 1e-6f);
}

static void test_output_clamped() {
    std::printf("total output clamps at outMin/outMax\n");
    pid::Controller p = make(100, 0, 0, 0, 0.01f, -5, 5);
    CHECK_NEAR(p.update(10, 0), 5.f, 0.f);
    CHECK_NEAR(p.update(-10, 0), -5.f, 0.f);
}

static void test_derivative_on_measurement_no_setpoint_kick() {
    std::printf("derivative acts on measurement only (tau=0 is unfiltered)\n");
    pid::Controller p = make(0, 0, 1, 0, 0.1f, -100, 100);
    CHECK_NEAR(p.update(0, 0), 0.f, 1e-6f);
    CHECK_NEAR(p.update(50, 0), 0.f, 1e-6f);
    CHECK_NEAR(p.update(50, 1), -10.f, 1e-4f);
    CHECK_NEAR(p.update(50, 1), 0.f, 1e-6f);
}

static void test_derivative_filter() {
    std::printf("derivative low-pass: d = tau*d_prev + (1-tau)*raw\n");
    pid::Controller p = make(0, 0, 1, 0.5f, 0.1f, -100, 100);
    p.update(0, 0);
    CHECK_NEAR(p.update(0, 1), -5.f, 1e-4f);
    CHECK_NEAR(p.update(0, 1), -2.5f, 1e-4f);
    CHECK_NEAR(p.update(0, 1), -1.25f, 1e-4f);
}

static void test_tau_one_disables_derivative() {
    std::printf("tau=1.0 freezes the derivative at zero\n");
    pid::Controller p = make(0, 0, 1, 1.0f, 0.1f, -100, 100);
    p.update(0, 0);
    CHECK_NEAR(p.update(0, 1), 0.f, 1e-6f);
    CHECK_NEAR(p.update(0, 5), 0.f, 1e-6f);
}

static void test_setconfig_keeps_state() {
    std::printf("setConfig retunes without resetting state\n");
    pid::Controller p = make(0, 1, 0, 0, 0.1f, -100, 100);
    p.update(1, 0);
    p.update(1, 0);
    const float before = p.integrator();
    pid::Config c = p.config();
    c.kp = 3.f;
    p.setConfig(c);
    CHECK(p.integrator() == before);
    CHECK(p.config().kp == 3.f);
}

static void test_deterministic() {
    std::printf("identical inputs give identical outputs\n");
    pid::Controller a = make(2, 0.5f, 0.1f, 0.2f, 0.01f, -10, 10);
    pid::Controller b = a;
    float pv = 0.f;
    for (int i = 0; i < 50; i++) {
        const float oa = a.update(100, pv), ob = b.update(100, pv);
        CHECK(oa == ob);
        pv += oa * 0.5f;
    }
}

static void test_demo_tuning_stays_bounded() {
    std::printf("example tuning: output always within limits (loop limit-cycles, does not converge)\n");
    pid::Controller p = make(2, 0.5f, 0.1f, 0.2f, 0.01f, -10, 10);
    float pv = 0.f;
    bool bounded = true;
    for (int i = 0; i < 200; i++) {
        const float out = p.update(100, pv);
        if (out < -10.f || out > 10.f) bounded = false;
        pv += out * 0.5f;
    }
    CHECK(bounded);
}

static void test_closed_loop_converges_with_sane_tuning() {
    std::printf("closed loop converges with Kp=1 Ki=0.5 Kd=0 on the 0.5-gain plant\n");
    pid::Controller p = make(1, 0.5f, 0, 0, 0.01f, -10, 10);
    float pv = 0.f;
    const float sp = 100.f;
    for (int i = 0; i < 5000; i++) pv += p.update(sp, pv) * 0.5f;
    CHECK(std::fabs(pv - sp) < 0.01f);
    CHECK(std::fabs(p.integrator()) < 1e-3f);
}

static void test_constexpr_usable() {
    std::printf("usable in constant expressions\n");
    constexpr float r = [] {
        pid::Controller p({.kp = 2, .ki = 0, .kd = 0, .tau = 0, .dt = 0.01f, .outMin = -100, .outMax = 100});
        return p.update(10, 0);
    }();
    static_assert(r == 20.f);
    CHECK(r == 20.f);
}

static void test_matches_c_implementation() {
    std::printf("bit-for-bit identical to the C module over 4 tunings x 2000 steps\n");
    const pid::Config cfgs[] = {
        {.kp = 2, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f, .dt = 0.01f, .outMin = -10, .outMax = 10},
        {.kp = 0.7f, .ki = 3.0f, .kd = 0.0f, .tau = 0.0f, .dt = 0.1f, .outMin = -1, .outMax = 1},
        {.kp = 0, .ki = 0, .kd = 5.0f, .tau = 0.9f, .dt = 0.002f, .outMin = -1000, .outMax = 1000},
        {.kp = 50, .ki = 20, .kd = 2, .tau = 0.5f, .dt = 0.05f, .outMin = 0, .outMax = 4095},
    };
    std::uint32_t seed = 12345u;
    int mismatches = 0;
    for (const pid::Config &c : cfgs) {
        pid::Controller cpp(c);
        PIDController cc;
        cc.Kp = c.kp;
        cc.Ki = c.ki;
        cc.Kd = c.kd;
        cc.tau = c.tau;
        cc.dt = c.dt;
        cc.outMin = c.outMin;
        cc.outMax = c.outMax;
        PIDController_Init(&cc);
        for (int i = 0; i < 2000; i++) {
            seed = seed * 1664525u + 1013904223u;
            const float sp = static_cast<float>((seed >> 8) % 2001) - 1000.f;
            seed = seed * 1664525u + 1013904223u;
            const float pv = static_cast<float>((seed >> 8) % 2001) - 1000.f;
            const float a = cpp.update(sp, pv);
            const float b = PIDController_Update(&cc, sp, pv);
            if (a != b || cpp.integrator() != cc.integrator || cpp.differentiator() != cc.differentiator) mismatches++;
        }
    }
    CHECK(mismatches == 0);
}

// ---- pid::Runner: non-blocking state machine, tick injected ----------------

static pid::Runner makeRun(float kp, float ki, float kd, float lo, float hi, std::uint32_t sampleMs) {
    return pid::Runner({.kp = kp, .ki = ki, .kd = kd, .tau = 0, .dt = 0.01f, .outMin = lo, .outMax = hi}, sampleMs,
                       &pid_tick_ms);
}

static void test_run_init_state() {
    std::printf("Runner: construction is Idle, output clamped zero, updated=false\n");
    pid::Runner p = makeRun(1, 0, 0, 20, 80, 1000);
    CHECK(p.state() == pid::State::Idle);
    CHECK(p.output() == 20.f);
    CHECK(!p.updated());
    CHECK(makeRun(1, 0, 0, -5, 5, 1000).output() == 0.f);
}

static void test_run_first_call_arms_only() {
    std::printf("Runner: first call latches the tick, computes nothing\n");
    pid_tick_stub_set(5000);
    pid::Runner p = makeRun(2, 0, 0, -100, 100, 100);
    CHECK(p.run(10, 0) == 0.f);
    CHECK(p.state() == pid::State::Armed);
    CHECK(p.lastTick() == 5000);
    CHECK(!p.updated());
    CHECK(p.controller().prevError() == 0.f);
}

static void test_run_holds_until_period() {
    std::printf("Runner: holds output and does not compute before sampleMs\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(2, 0, 0, -100, 100, 100);
    p.run(10, 0);
    for (std::uint32_t t = 1; t < 100; t += 7) {
        pid_tick_stub_set(t);
        CHECK(p.run(10, 0) == 0.f);
        CHECK(!p.updated());
    }
    pid_tick_stub_set(100);
    CHECK_NEAR(p.run(10, 0), 20.f, 1e-5f);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 0.1f, 1e-6f);
    pid_tick_stub_set(150);
    CHECK_NEAR(p.run(50, 0), 20.f, 1e-5f);
    CHECK(!p.updated());
}

static void test_run_one_minute_period() {
    std::printf("Runner: 60 s period, 10 ms loop: exactly one compute per minute\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(0, 1, 0, -1000, 1000, 60000);
    p.run(1, 0);
    int computes = 0;
    for (std::uint32_t t = 10; t <= 300000; t += 10) {
        pid_tick_stub_set(t);
        p.run(1, 0);
        computes += p.updated() ? 1 : 0;
    }
    CHECK(computes == 5);
    CHECK_NEAR(p.dt(), 60.f, 1e-4f);
    CHECK_NEAR(p.controller().integrator(), 0.5f * 60.f + 4.f * 60.f, 1e-2f);
}

static void test_run_uses_measured_interval_and_stays_on_grid() {
    std::printf("Runner: late sample uses the real dt, next sample returns to the grid\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(0, 1, 0, -100, 100, 1000);
    p.run(1, 0);
    pid_tick_stub_set(1237);
    p.run(1, 0);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 1.237f, 1e-6f);
    CHECK(p.lastTick() == 1000);
    pid_tick_stub_set(1999);
    p.run(1, 0);
    CHECK(!p.updated());
    pid_tick_stub_set(2000);
    p.run(1, 0);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 0.763f, 1e-6f);
    CHECK(p.lastTick() == 2000);
}

static void test_run_stall_gives_one_compute_no_burst() {
    std::printf("Runner: a 2.5-period stall gives exactly one compute with dt=2.5 periods, no burst\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(0, 1, 0, -100, 100, 1000);
    p.run(1, 0);
    pid_tick_stub_set(2500);
    p.run(1, 0);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 2.5f, 1e-6f);
    CHECK(p.lastTick() == 2000);
    pid_tick_stub_set(2501);
    p.run(1, 0);
    CHECK(!p.updated());
    pid_tick_stub_set(3000);
    p.run(1, 0);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 0.5f, 1e-6f);
}

static void test_run_average_period_is_exact_under_jitter() {
    std::printf("Runner: with loop latency jitter the sample count over 1000 periods is exactly 1000\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(0, 1, 0, -1e9f, 1e9f, 100);
    p.run(1, 0);
    std::uint32_t seed = 99u, t = 0;
    int computes = 0;
    float dtSum = 0.f;
    while (t < 100000u) {
        seed = seed * 1664525u + 1013904223u;
        t += 1 + (seed >> 8) % 23;
        pid_tick_stub_set(t);
        p.run(1, 0);
        if (p.updated()) { computes++; dtSum += p.dt(); }
    }
    CHECK(computes == 1000);
    CHECK_NEAR(dtSum / static_cast<float>(computes), 0.1f, 1e-3f);
}

static void test_run_tick_wraparound() {
    std::printf("Runner: survives the 32-bit tick wrap\n");
    pid_tick_stub_set(0xFFFFFFF0u);
    pid::Runner p = makeRun(2, 0, 0, -100, 100, 100);
    p.run(10, 0);
    pid_tick_stub_set(0xFFFFFFFFu);
    CHECK(p.run(10, 0) == 0.f && !p.updated());
    pid_tick_stub_set(84u);
    CHECK_NEAR(p.run(10, 0), 20.f, 1e-5f);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 0.1f, 1e-6f);
}

static void test_run_period_zero_every_tick() {
    std::printf("Runner: sampleMs=0 computes whenever the tick advanced, never on a repeat tick\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(2, 0, 0, -100, 100, 0);
    p.run(10, 0);
    p.run(10, 0);
    CHECK(!p.updated());
    pid_tick_stub_set(3);
    CHECK_NEAR(p.run(10, 0), 20.f, 1e-5f);
    CHECK(p.updated());
    CHECK_NEAR(p.dt(), 0.003f, 1e-7f);
}

static void test_run_reset_rearms() {
    std::printf("Runner: reset after running clears state and re-arms on the next call\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(0, 1, 0, -100, 100, 100);
    p.run(1, 0);
    pid_tick_stub_set(100);
    p.run(1, 0);
    CHECK(p.controller().integrator() != 0.f);
    p.reset();
    CHECK(p.state() == pid::State::Idle && p.controller().integrator() == 0.f && p.output() == 0.f);
    pid_tick_stub_set(5000);
    p.run(1, 0);
    CHECK(p.state() == pid::State::Armed && p.lastTick() == 5000 && !p.updated());
}

static void test_run_matches_controller_on_exact_grid() {
    std::printf("Runner: on an exact tick grid it is bit-identical to Controller with fixed dt\n");
    pid::Controller a = make(2, 0.5f, 0.1f, 0.2f, 0.1f, -10, 10);
    pid::Runner b({.kp = 2, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f, .dt = 0.1f, .outMin = -10, .outMax = 10}, 100,
                  &pid_tick_ms);
    pid_tick_stub_set(0);
    b.run(100, 0);
    float pv = 0.f;
    bool same = true;
    for (int i = 1; i <= 100; i++) {
        pid_tick_stub_set(static_cast<std::uint32_t>(i) * 100u);
        const float oa = a.update(100, pv);
        const float ob = b.run(100, pv);
        if (!b.updated() || oa != ob) same = false;
        pv += oa * 0.5f;
    }
    CHECK(same);
}

static void test_run_never_blocks() {
    std::printf("Runner: one tick read per call, returns without the clock advancing\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(1, 0, 0, -100, 100, 60000);
    pid_tick_stub_reset_calls();
    for (int i = 0; i < 1000; i++) p.run(1, 0);
    CHECK(pid_tick_stub_calls() == 1000);
    CHECK(!p.updated());
}

static void test_run_constexpr_usable() {
    std::printf("Runner: usable in constant expressions with a constexpr tick\n");
    constexpr float r = [] {
        pid::Runner p({.kp = 2, .ki = 0, .kd = 0, .tau = 0, .dt = 0.01f, .outMin = -100, .outMax = 100}, 100,
                      [] { return static_cast<std::uint32_t>(0); });
        p.run(10, 0);
        pid::Runner q = p;
        return q.output();
    }();
    static_assert(r == 0.f);
    CHECK(r == 0.f);
}

// Bitwise cross-check of Runner against PIDController_Run: both read the same
// stub tick, irregular sample spacing, several tunings
static void test_runner_matches_c_run() {
    std::printf("Runner: bit-for-bit identical to C PIDController_Run over 4 tunings, jittered ticks\n");
    struct Case { pid::Config c; std::uint32_t ms; };
    const Case cases[] = {
        {{.kp = 2, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f, .dt = 0, .outMin = -10, .outMax = 10}, 10},
        {{.kp = 0.7f, .ki = 3.0f, .kd = 0.0f, .tau = 0.0f, .dt = 0, .outMin = -1, .outMax = 1}, 1000},
        {{.kp = 0, .ki = 0, .kd = 5.0f, .tau = 0.9f, .dt = 0, .outMin = -1000, .outMax = 1000}, 250},
        {{.kp = 50, .ki = 20, .kd = 2, .tau = 0.5f, .dt = 0, .outMin = 0, .outMax = 4095}, 60000},
    };
    std::uint32_t seed = 777u;
    int mismatches = 0;
    for (const Case &k : cases) {
        pid::Runner cpp(k.c, k.ms, &pid_tick_ms);
        PIDController cc;
        cc.Kp = k.c.kp; cc.Ki = k.c.ki; cc.Kd = k.c.kd; cc.tau = k.c.tau; cc.dt = 0;
        cc.outMin = k.c.outMin; cc.outMax = k.c.outMax; cc.sampleMs = k.ms;
        PIDController_Init(&cc);
        pid_tick_stub_set(0xFFFFF000u);
        for (int i = 0; i < 3000; i++) {
            seed = seed * 1664525u + 1013904223u;
            pid_tick_stub_advance((seed >> 8) % (k.ms / 4 + 2));
            seed = seed * 1664525u + 1013904223u;
            const float sp = static_cast<float>((seed >> 8) % 2001) - 1000.f;
            seed = seed * 1664525u + 1013904223u;
            const float pv = static_cast<float>((seed >> 8) % 2001) - 1000.f;
            const float a = cpp.run(sp, pv);
            const float b = PIDController_Run(&cc, sp, pv);
            if (a != b || cpp.updated() != (cc.updated != 0) || cpp.lastTick() != cc.lastTick ||
                cpp.lastRun() != cc.lastRun || cpp.dt() != cc.dt ||
                cpp.controller().integrator() != cc.integrator || cpp.controller().differentiator() != cc.differentiator)
                mismatches++;
        }
    }
    CHECK(mismatches == 0);
}

static const float kNaN = std::numeric_limits<float>::quiet_NaN();
static const float kInf = std::numeric_limits<float>::infinity();
static const float kMax = std::numeric_limits<float>::max();

static void test_fault_config() {
    std::printf("fault: invalid configuration holds the output and leaves state alone\n");
    pid::Controller p = make(1, 1, 0, 0, 0.1f, -100, 100);
    const float good = p.update(10, 0);
    CHECK(p.fault() == pid::Fault::None && p.output() == good);
    const float integ = p.integrator();
    const pid::Config ok = p.config();

    const auto bad = [&](pid::Config c) {
        p.setConfig(c);
        const float out = p.update(50, 0);
        return out == good && p.fault() == pid::Fault::Config;
    };
    pid::Config c = ok;
    c.dt = 0.f;        CHECK(bad(c));
    c.dt = -0.1f;      CHECK(bad(c));
    c.dt = kNaN;       CHECK(bad(c));
    c = ok; c.tau = 1.5f;     CHECK(bad(c));
    c.tau = -0.1f;            CHECK(bad(c));
    c.tau = kNaN;             CHECK(bad(c));
    c = ok; c.outMin = 200.f; CHECK(bad(c));
    c.outMin = -kInf;         CHECK(bad(c));
    c = ok; c.outMax = kInf;  CHECK(bad(c));
    c = ok; c.kp = kNaN;      CHECK(bad(c));
    c = ok; c.ki = kInf;      CHECK(bad(c));
    c = ok; c.kd = kNaN;      CHECK(bad(c));

    CHECK(p.integrator() == integ && p.prevError() == 10.f);
    p.setConfig(ok);
    p.update(10, 0);
    CHECK(p.fault() == pid::Fault::None);
}

static void test_fault_input() {
    std::printf("fault: NaN or infinite input holds the output and clears on recovery\n");
    pid::Controller p = make(1, 1, 1, 0.2f, 0.1f, -100, 100);
    const float good = p.update(10, 1);
    const float integ = p.integrator(), diff = p.differentiator();

    CHECK(p.update(10, kNaN) == good && p.fault() == pid::Fault::Input);
    CHECK(p.update(kNaN, 1) == good && p.fault() == pid::Fault::Input);
    CHECK(p.update(10, kInf) == good && p.fault() == pid::Fault::Input);
    CHECK(p.update(-kInf, 1) == good && p.fault() == pid::Fault::Input);
    CHECK(p.integrator() == integ && p.differentiator() == diff);
    CHECK(p.prevMeasurement() == 1.f && p.prevError() == 9.f && p.output() == good);

    const float next = p.update(10, 1);
    CHECK(p.fault() == pid::Fault::None);
    CHECK(std::isfinite(next) && next >= -100.f && next <= 100.f);
    p.update(10, kNaN);
    p.reset();
    CHECK(p.fault() == pid::Fault::None && p.output() == 0.f);
}

static void test_fault_numeric() {
    std::printf("fault: overflow in the computation holds the output\n");
    pid::Controller p = make(0, 0, kMax, 0, 1e-6f, -100, 100);
    const float good = p.update(0, 0);
    CHECK(p.fault() == pid::Fault::None);
    CHECK(p.update(0, kMax) == good && p.fault() == pid::Fault::Numeric);
    CHECK(p.differentiator() == 0.f && p.prevMeasurement() == 0.f);

    // error overflows to +inf; a zero gain times inf is NaN
    pid::Controller q = make(1, 0, 0, 0, 0.1f, -100, 100);
    CHECK(q.update(kMax, -kMax) == 0.f && q.fault() == pid::Fault::Numeric && q.integrator() == 0.f);
    pid::Controller r = make(0, 1, 0, 0, 0.1f, -100, 100);
    CHECK(r.update(kMax, -kMax) == 0.f && r.fault() == pid::Fault::Numeric);
    CHECK(r.integrator() == 0.f && r.prevError() == 0.f);
}

static void test_run_reports_fault() {
    std::printf("Runner: a faulted sample is flagged, output held, schedule kept\n");
    pid_tick_stub_set(0);
    pid::Runner p = makeRun(1, 0, 0, -100, 100, 10);
    p.run(5, 0);
    pid_tick_stub_advance(10);
    CHECK(p.run(5, 0) == 5.f && p.updated() && p.fault() == pid::Fault::None);
    pid_tick_stub_advance(10);
    CHECK(p.run(5, kNaN) == 5.f);
    CHECK(p.updated() && p.fault() == pid::Fault::Input && p.output() == 5.f);
    pid_tick_stub_advance(5);
    CHECK(p.run(7, 0) == 5.f && !p.updated() && p.fault() == pid::Fault::Input);
    pid_tick_stub_advance(5);
    CHECK(p.run(7, 0) == 7.f && p.updated() && p.fault() == pid::Fault::None);
}

static void test_run_without_tick() {
    std::printf("Runner: no tick source never computes and reports a config fault\n");
    pid::Runner d;
    CHECK(d.run(1, 0) == 0.f && !d.updated() && d.fault() == pid::Fault::Config);
    pid::Runner p({.kp = 1, .ki = 0, .kd = 0, .tau = 0, .dt = 0, .outMin = 20, .outMax = 80}, 10, nullptr);
    CHECK(p.run(50, 0) == 20.f && !p.updated() && p.state() == pid::State::Idle);
    CHECK(p.fault() == pid::Fault::Config);
}

static void test_faults_match_c() {
    std::printf("faults: identical to the C module with bad inputs and bad tuning injected\n");
    const pid::Config base{.kp = 2, .ki = 0.5f, .kd = 0.1f, .tau = 0.2f, .dt = 0, .outMin = -10, .outMax = 10};
    pid::Runner cpp(base, 10, &pid_tick_ms);
    PIDController cc;
    cc.Kp = base.kp; cc.Ki = base.ki; cc.Kd = base.kd; cc.tau = base.tau; cc.dt = 0;
    cc.outMin = base.outMin; cc.outMax = base.outMax; cc.sampleMs = 10;
    PIDController_Init(&cc);
    pid_tick_stub_set(0);
    std::uint32_t seed = 4242u;
    int mismatches = 0, faults = 0;
    for (int i = 0; i < 5000; i++) {
        seed = seed * 1664525u + 1013904223u;
        pid_tick_stub_advance((seed >> 8) % 7);
        seed = seed * 1664525u + 1013904223u;
        const std::uint32_t pick = (seed >> 8) % 12;
        float sp = static_cast<float>((seed >> 16) % 201) - 100.f;
        float pv = static_cast<float>((seed >> 4) % 201) - 100.f;
        pid::Config c = base;
        if (pick == 0) sp = kNaN;
        if (pick == 1) pv = kInf;
        if (pick == 2) pv = -kMax, sp = kMax;
        if (pick == 3) c.tau = 2.f;
        if (pick == 4) c.outMin = 50.f;
        if (pick == 5) c.kd = kNaN;
        c.dt = cpp.dt();
        cpp.controller().setConfig(c);
        cc.Kd = c.kd; cc.tau = c.tau; cc.outMin = c.outMin;
        const float a = cpp.run(sp, pv);
        const float b = PIDController_Run(&cc, sp, pv);
        if (cpp.fault() != pid::Fault::None) faults++;
        if (a != b || cpp.updated() != (cc.updated != 0) ||
            static_cast<int>(cpp.fault()) != static_cast<int>(cc.fault) ||
            cpp.controller().integrator() != cc.integrator ||
            cpp.controller().differentiator() != cc.differentiator ||
            cpp.controller().prevError() != cc.prevError || cpp.output() != cc.output)
            mismatches++;
    }
    CHECK(mismatches == 0);
    CHECK(faults > 500);
    CHECK(static_cast<int>(pid::Fault::Config) == PID_FAULT_CONFIG && static_cast<int>(pid::Fault::Input) == PID_FAULT_INPUT &&
          static_cast<int>(pid::Fault::Numeric) == PID_FAULT_NUMERIC);
}

int main() {
    test_init_clears_state();
    test_proportional_only();
    test_integral_is_trapezoidal();
    test_integrator_clamped_to_output_limits();
    test_output_clamped();
    test_derivative_on_measurement_no_setpoint_kick();
    test_derivative_filter();
    test_tau_one_disables_derivative();
    test_setconfig_keeps_state();
    test_deterministic();
    test_demo_tuning_stays_bounded();
    test_closed_loop_converges_with_sane_tuning();
    test_constexpr_usable();
    test_matches_c_implementation();
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
    test_run_matches_controller_on_exact_grid();
    test_run_never_blocks();
    test_run_constexpr_usable();
    test_runner_matches_c_run();
    test_fault_config();
    test_fault_input();
    test_fault_numeric();
    test_run_reports_fault();
    test_run_without_tick();
    test_faults_match_c();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
