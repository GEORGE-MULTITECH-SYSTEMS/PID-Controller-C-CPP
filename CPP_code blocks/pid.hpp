#ifndef PID_HPP
#define PID_HPP

#include <cstdint>
#include <limits>
#include <type_traits>

namespace pid {

// Fixed-size floating type. IEEE 754 single precision.
using float32_t = float;
static_assert(std::numeric_limits<float32_t>::is_iec559 && (sizeof(float32_t) == 4U),
              "pid requires IEEE 754 single precision");

struct Config {
    float32_t kp = 0.0f;
    float32_t ki = 0.0f;
    float32_t kd = 0.0f;
    // Derivative low-pass coefficient, 0.0 = unfiltered, 1.0 = derivative frozen at zero
    float32_t tau = 0.0f;
    float32_t dt = 1.0f;
    float32_t outMin = 0.0f;
    float32_t outMax = 0.0f;
};

// Result of the most recent compute. Not latched: each compute overwrites it.
// On any fault the controller state is left untouched and the output is held
// at its last good value; the application decides what its safe state is.
enum class Fault : std::uint8_t {
    None = 0,
    Config,     // gains/limits not finite, outMin > outMax, tau outside 0..1, dt <= 0, no tick source
    Input,      // setpoint or measurement is NaN or infinite
    Numeric     // the computation overflowed to a non-finite value
};

namespace detail {

// False for NaN and for either infinity.
[[nodiscard]] constexpr bool isFinite(float32_t v) noexcept {
    return (v >= std::numeric_limits<float32_t>::lowest()) && (v <= std::numeric_limits<float32_t>::max());
}

[[nodiscard]] constexpr float32_t clampTo(float32_t v, float32_t lo, float32_t hi) noexcept {
    float32_t r = v;
    if (v > hi) {
        r = hi;
    } else if (v < lo) {
        r = lo;
    } else {
        // in range
    }
    return r;
}

[[nodiscard]] constexpr bool configValid(const Config &c) noexcept {
    const bool finite = isFinite(c.kp) && isFinite(c.ki) && isFinite(c.kd) &&
                        isFinite(c.outMin) && isFinite(c.outMax) && isFinite(c.dt);

    // The comparisons are false for a NaN tau, so that is rejected too.
    return finite && (c.outMin <= c.outMax) && (c.tau >= 0.0f) && (c.tau <= 1.0f) && (c.dt > 0.0f);
}

} // namespace detail

class Controller {
public:
    constexpr Controller() noexcept = default;
    constexpr explicit Controller(const Config &cfg) noexcept
        : cfg_(cfg), output_(detail::clampTo(0.0f, cfg.outMin, cfg.outMax)) {}

    constexpr void reset() noexcept {
        integrator_ = 0.0f;
        prevError_ = 0.0f;
        prevMeasurement_ = 0.0f;
        differentiator_ = 0.0f;
        output_ = detail::clampTo(0.0f, cfg_.outMin, cfg_.outMax);
        fault_ = Fault::None;
    }

    // One PID step using Config::dt as the interval. Sets fault(); returns
    // the new output, or the held output on a fault.
    constexpr float32_t update(float32_t setpoint, float32_t measurement) noexcept {
        if (!detail::configValid(cfg_)) {
            fault_ = Fault::Config;
        } else if ((!detail::isFinite(setpoint)) || (!detail::isFinite(measurement))) {
            fault_ = Fault::Input;
        } else {
            const float32_t error = setpoint - measurement;
            const float32_t proportional = cfg_.kp * error;

            // Trapezoidal integral, then anti-windup clamp to the output limits
            float32_t integrator = integrator_ + (0.5f * cfg_.ki * cfg_.dt * (error + prevError_));
            integrator = detail::clampTo(integrator, cfg_.outMin, cfg_.outMax);

            // Derivative on measurement, low-pass filtered
            const float32_t deltaMeasurement = measurement - prevMeasurement_;
            const float32_t differentiator =
                (cfg_.tau * differentiator_) +
                ((1.0f - cfg_.tau) * (((-cfg_.kd) * deltaMeasurement) / cfg_.dt));

            const float32_t total =
                detail::clampTo(proportional + integrator + differentiator, cfg_.outMin, cfg_.outMax);

            // Commit only if every result is a number; a NaN would otherwise
            // pass straight through the clamp
            if (detail::isFinite(integrator) && detail::isFinite(differentiator) && detail::isFinite(total)) {
                integrator_ = integrator;
                differentiator_ = differentiator;
                prevError_ = error;
                prevMeasurement_ = measurement;
                output_ = total;
                fault_ = Fault::None;
            } else {
                fault_ = Fault::Numeric;
            }
        }
        return output_;
    }

    [[nodiscard]] constexpr const Config &config() const noexcept { return cfg_; }
    // Retunes without touching the controller state
    constexpr void setConfig(const Config &cfg) noexcept { cfg_ = cfg; }

    [[nodiscard]] constexpr float32_t integrator() const noexcept { return integrator_; }
    [[nodiscard]] constexpr float32_t differentiator() const noexcept { return differentiator_; }
    [[nodiscard]] constexpr float32_t prevError() const noexcept { return prevError_; }
    [[nodiscard]] constexpr float32_t prevMeasurement() const noexcept { return prevMeasurement_; }
    // Last good output, held on a fault
    [[nodiscard]] constexpr float32_t output() const noexcept { return output_; }
    [[nodiscard]] constexpr Fault fault() const noexcept { return fault_; }

private:
    Config cfg_{};
    float32_t integrator_ = 0.0f;
    float32_t prevError_ = 0.0f;
    float32_t prevMeasurement_ = 0.0f;
    float32_t differentiator_ = 0.0f;
    float32_t output_ = 0.0f;
    Fault fault_ = Fault::None;
};

static_assert(std::is_trivially_copyable_v<Controller>, "Controller must stay trivially copyable");
static_assert(std::is_nothrow_default_constructible_v<Controller>, "Controller must not throw");

// Platform-neutral tick: a monotonic millisecond counter, free to wrap at 2^32.
// Supply pid_tick_ms from the C module's platform/ files, or any other source.
using TickFn = std::uint32_t (*)();

enum class State : std::uint8_t {
    Idle = 0,   // no tick latched yet; the first run() arms the timer
    Armed       // waiting for sampleMs to elapse, output held
};

// Non-blocking sample-period state machine around a Controller.
// run() never waits: one tick read, compute only when the period has elapsed,
// otherwise the held output is returned immediately. Check fault() whenever
// updated() is set.
class Runner {
public:
    constexpr Runner() noexcept = default;
    constexpr Runner(const Config &cfg, std::uint32_t sampleMs, TickFn tick) noexcept
        : ctl_(cfg), sampleMs_(sampleMs), tick_(tick) {}

    constexpr void reset() noexcept {
        ctl_.reset();
        state_ = State::Idle;
        lastTick_ = 0U;
        lastRun_ = 0U;
        updated_ = false;
    }

    constexpr float32_t run(float32_t setpoint, float32_t measurement) noexcept {
        updated_ = false;

        // Without a tick source nothing is ever computed; fault() reports it.
        if (tick_ != nullptr) {
            const std::uint32_t now = tick_();

            if (state_ == State::Idle) {
                lastTick_ = now;
                lastRun_ = now;
                state_ = State::Armed;
            } else {
                const std::uint32_t elapsed = now - lastTick_;   // uint32 wrap-safe

                if ((elapsed != 0U) && (elapsed >= sampleMs_)) {
                    // Scheduling is phase-locked to the sample grid (exact
                    // average period, one compute per call after a stall);
                    // dt is the true time since the previous compute.
                    const std::uint32_t sinceRun = now - lastRun_;

                    if (sampleMs_ != 0U) {
                        lastTick_ = now - (elapsed % sampleMs_);
                    } else {
                        lastTick_ = now;
                    }
                    Config c = ctl_.config();
                    c.dt = static_cast<float32_t>(sinceRun) * 0.001f;
                    lastRun_ = now;
                    ctl_.setConfig(c);
                    static_cast<void>(ctl_.update(setpoint, measurement));
                    updated_ = true;
                }
            }
        }
        return ctl_.output();
    }

    [[nodiscard]] constexpr Controller &controller() noexcept { return ctl_; }
    [[nodiscard]] constexpr const Controller &controller() const noexcept { return ctl_; }
    constexpr void setSampleMs(std::uint32_t ms) noexcept { sampleMs_ = ms; }
    [[nodiscard]] constexpr std::uint32_t sampleMs() const noexcept { return sampleMs_; }

    [[nodiscard]] constexpr State state() const noexcept { return state_; }
    [[nodiscard]] constexpr std::uint32_t lastTick() const noexcept { return lastTick_; }
    [[nodiscard]] constexpr std::uint32_t lastRun() const noexcept { return lastRun_; }
    [[nodiscard]] constexpr float32_t output() const noexcept { return ctl_.output(); }
    [[nodiscard]] constexpr bool updated() const noexcept { return updated_; }
    [[nodiscard]] constexpr float32_t dt() const noexcept { return ctl_.config().dt; }
    [[nodiscard]] constexpr Fault fault() const noexcept {
        return (tick_ != nullptr) ? ctl_.fault() : Fault::Config;
    }

private:
    Controller ctl_{};
    std::uint32_t sampleMs_ = 0U;
    TickFn tick_ = nullptr;
    State state_ = State::Idle;
    std::uint32_t lastTick_ = 0U;
    std::uint32_t lastRun_ = 0U;
    bool updated_ = false;
};

static_assert(std::is_trivially_copyable_v<Runner>, "Runner must stay trivially copyable");

} // namespace pid

#endif // PID_HPP
