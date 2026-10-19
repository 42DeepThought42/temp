#include "tdsim/simulator.hpp"

#include <boost/numeric/odeint.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tdsim {

using namespace boost::numeric::odeint;

Simulator::Simulator(BuiltSystem built, SimParams params)
    : built_(std::move(built)), sys_(built_.sys), params_(std::move(params)) {
    // Flatten the LatchRef list to raw Latched* (the period is carried on the
    // latch itself, set by the factory from each type's collect_latches hook).
    for (auto& lr : built_.latches) {
        latches_.push_back(lr.latch);
    }
    // Multi-rate: the driver's base tick must equal the factory's resolved
    // base rate so that period_ticks matches the driver's k counter.
    base_hz_ = built_.base_hz;
    if (base_hz_ > 0) {
        // Each flight computer's period in base ticks is fc_rate / base_hz,
        // where fc_rate = 1/dt_macro. With the LCM base, base_hz is a multiple
        // of fc_rate, so fc_rate / base_hz <= 1... that's wrong. The period is
        // base_hz / fc_rate (how many base ticks make up one FC tick).
        // For dt_macro=0.01 (fc_rate=100), base_hz=100: period = 100/100 = 1.
        // For a hypothetical base_hz=200: period = 200/100 = 2.
        std::size_t fc_rate_hz =
            (params_.dt_macro > 0.0)
                ? static_cast<std::size_t>(std::llround(1.0 / params_.dt_macro))
                : base_hz_; // fallback: FC ticks at base rate
        std::size_t fc_period = (base_hz_ % fc_rate_hz == 0) ? (base_hz_ / fc_rate_hz) : 1;
        for (auto& fc : computers_) {
            fc.period_ticks = (fc_period >= 1) ? fc_period : 1;
            fc.dt = static_cast<double>(fc.period_ticks) * base_period();
        }
    }
}

void Simulator::add_flight_computer(const std::string& model_id,
                                    std::unique_ptr<IFlightSoftware> sw,
                                    const std::vector<std::string>& sensor_signals,
                                    const std::vector<std::string>& command_signals) {
    computers_.push_back(FlightComputer{model_id, std::move(sw), sensor_signals, command_signals});
}

void Simulator::add_jettison_event(JettisonEvent ev) { events_.push_back(std::move(ev)); }

void Simulator::build_telemetry_columns() {
    if (params_.telemetry_file.empty() || params_.telemetry_rate <= 0.0) {
        return; // telemetry disabled; keep telemetry_ default-constructed (no file)
    }
    std::vector<TelemetryLogger::Column> cols;
    // ODE columns: from each model slot's descriptor. Use the slot's unique
    // id (not desc.name, which is the type name and not unique across
    // instances) so that two tanks don't both produce "tank.mass".
    for (const auto& slot : sys_.slots()) {
        const auto& desc = slot->descriptor();
        for (std::size_t i = 0; i < desc.size(); i++) {
            std::string name = slot->id + "." + desc.slot_names[i];
            std::size_t offset = slot->state_offset + i;
            cols.push_back(TelemetryLogger::Column{name, [this, offset]() { return sys_.state()[offset]; }});
        }
    }
    // Algebraic signal columns: the factory attached named readers for the
    // signals it wants logged.
    for (const auto& [name, _] : built_.core->reader_names()) {
        cols.push_back(TelemetryLogger::Column{name, [this, name]() { return sys_.read_signal(name); }});
    }
    // Make the CSV schema fully deterministic. The ODE columns above follow
    // sys_.slots() order (= topological evaluation order, which varies with
    // the model declaration order in the spec) and the reader table is an
    // unordered_map. Sorting ALL columns by name makes telemetry.csv stable
    // across runs, platforms, hash seeds, and equivalent-but-reordered specs
    // — required for it to be diffable and for the TOML-vs-programmatic oracle
    // test to compare rows as strings.
    std::sort(cols.begin(), cols.end(),
              [](const TelemetryLogger::Column& a, const TelemetryLogger::Column& b) {
                  return a.name < b.name;
              });
    telemetry_ = TelemetryLogger(std::move(cols), params_.telemetry_file);
}

// Guard: record() on a default-constructed (no-file) logger is a no-op, so the
// driver can call it unconditionally without tracking telemetry_on separately.

void Simulator::discrete_tick(double t) {
    for (auto& fc : computers_) {
        SensorInputs sensors;
        for (const auto& sig : fc.sensor_signals) {
            sensors[sig] = sys_.read_signal(sig);
        }
        auto cmds = fc.sw->update(t, sensors);
        for (const auto& [name, val] : cmds) {
            sys_.propose_signal(name, val);
        }
    }
}

void Simulator::commit_latches() {
    for (auto* l : latches_) {
        l->commit();
    }
}

void Simulator::check_events(double t) {
    // Refresh the core's state view from the live ODE vector BEFORE the
    // triggers run: SystemFunctor::state() (which triggers typically read) is
    // backed by the core's x_, and that copy is only updated when the functor
    // is called as the ODE system — not by the integrator's direct writes to
    // the live vector between windows.
    built_.core->set_state(x_live_);
    for (auto& ev : events_) {
        if (!ev.fired && ev.trigger(t, sys_)) {
            ev.fired = true;
            t_at_jettison_ = t;
            handle_jettison(ev);
        }
    }
}

void Simulator::handle_jettison(const JettisonEvent& ev) {
    spdlog::info("[EVENT] Jettisoning stage '{}' at t = {:.3f}", ev.stage_id, t_at_jettison_);
    // Mark the stage's models inactive in BOTH the builder (for re-sort and
    // reporting) and the core slots (so Pass 2/3 skips them — per the
    // decision log: skip inactive models, keep their state slots so spans
    // stay valid).
    for (const auto& slot : sys_.slots()) {
        if (slot->stage_id == ev.stage_id) {
            slot->active = false;
        }
    }
    built_.builder.inactivate_stage(ev.stage_id);
    auto new_order = built_.builder.resolve_topology_after_jettison();
    // Push the re-sorted order into the core so the next derivative call
    // evaluates only the surviving subgraph.
    built_.core->set_evaluation_order(std::move(new_order));
    // NOTE: the stepper is rebuilt by the run() loop on the next window
    // because it is constructed per-window (see run()). This is what makes
    // the discontinuity safe: no adaptive history carries across the jettison.
}

void Simulator::run() {
    build_telemetry_columns();
    spdlog::info("[TELEMETRY] Writing to '{}' at {:.1f} Hz ({} columns)",
                 params_.telemetry_file, params_.telemetry_rate, telemetry_.num_columns());

    double t = 0.0;
    // The live ODE vector is the single source of truth. The integrator
    // writes it in place; the core's x_ is refreshed from it on every
    // derivative call (and from here before event triggers read it).
    x_live_ = sys_.state();

    // The stepper is (re)built at the start of every base tick so that no
    // adaptive-step history survives a jettison discontinuity.
    size_t total_steps = 0;
    size_t n_windows = 0;
    // Base-tick counter. Multi-rate latches and flight computers tick only
    // when their period is due (k % period_ticks == 0). A period-1 entity
    // (the default, and the only kind in the single-rate reference vehicle)
    // ticks every base tick — k % 1 == 0 always — so the single-rate path is
    // unchanged. When multi-rate is active (base_hz_ > 0), the base tick is
    // finer than dt_macro and the FC ticks only every fc.period_ticks base
    // ticks (its own rate); sensor latches with period_ticks > 1 tick at
    // their own slower rate.
    size_t k = 0;
    while (t < params_.t_end - 1e-12) {
        double t_window_end = std::min(t + base_period(), params_.t_end);

        // Discrete tick at the base-tick start, but ONLY for entities whose
        // period is due this tick. In the single-rate case every entity has
        // period 1, so this is equivalent to the original unconditional
        // discrete_tick(t) — the behavior is byte-identical.
        for (auto& fc : computers_) {
            if (k % fc.period_ticks == 0) {
                SensorInputs sensors;
                for (const auto& sig : fc.sensor_signals) {
                    sensors[sig] = sys_.read_signal(sig);
                }
                auto cmds = fc.sw->update(t, sensors);
                for (const auto& [name, val] : cmds) {
                    sys_.propose_signal(name, val);
                }
            }
        }

        // Build a fresh controlled stepper for this window. RK5(4) Dopri5 on
        // std::vector<double>. Chosen over RK78 Fehlberg78 after benchmarking
        // on the example vehicle: at the dt_max=1 ms cap the two produce
        // identical step counts and final state, but RK54 does 6 RHS evals
        // per step vs 7, making it ~2.2x faster for this light algebraic RHS.
        // (See SESSION_NOTES §8.)
        using stepper_t = runge_kutta_dopri5<std::vector<double>, double, std::vector<double>, double>;
        auto stepper = make_controlled(params_.eps_abs, params_.eps_rel, stepper_t{});

        // Integrate within the window, bounded by t_window_end, using
        // integrate_adaptive (the correct high-level API). It:
        //   * drives the 3-arg try_step internally (the 5-arg form with an
        //     externally-sized dxdt is broken on Boost 1.92 — see SESSION_NOTES §6.2),
        //   * clamps dt so the final step lands exactly on end_time (no overshoot),
        //   * has a built-in failed_step_checker that throws if the step size
        //     collapses (so a pathological RHS can't hang the sim).
        // NOTE: start_time and dt are passed BY VALUE — they are NOT updated
        // in place. The final time and step count must be captured via the
        // observer (this was the source of the earlier 't=1875' misdiagnosis:
        // reading a stale local after the call).
        double window_start = t;
        double observed_t = t;
        size_t window_steps = 0;
        size_t n = integrate_adaptive(
            stepper, sys_, x_live_, window_start, t_window_end, params_.dt_max,
            [&](const std::vector<double>&, double obs_t) {
                observed_t = obs_t;
                ++window_steps;
            });
        // integrate_adaptive returns the number of steps taken; advance t to
        // the (clamped) window end — it lands exactly on t_window_end by
        // construction (verified against analytic solutions).
        total_steps += n;
        t = observed_t; // == t_window_end (or t_end on the final window)
        if (window_steps > 20) {
            spdlog::debug("[STEP] window [t={:.4f}..{:.4f}]: {} steps",
                          window_start, t, window_steps);
        }

        // Window end boundary: commit the latches whose period is due this
        // tick (they become 'current' for the next window), then check events.
        // Multi-rate: a latch with period_ticks > 1 is a zero-order hold at a
        // slower rate; it commits only on its own tick, not every base tick.
        for (auto* l : latches_) {
            if (k % l->period_ticks == 0) l->commit();
        }
        check_events(t);
        if (t >= next_telem_ - 1e-12) {
            telemetry_.record(t);
            next_telem_ += 1.0 / params_.telemetry_rate;
        }
        ++n_windows;
        ++k;
    }

    if (telemetry_.num_columns() > 1) {
        telemetry_.record(t); // final row; close() is handled by the logger's destructor
    }
    spdlog::info("[SIM] Complete. Final t = {:.3f} ({} adaptive steps over {} macro windows)",
                 t, total_steps, n_windows);
}

} // namespace tdsim
