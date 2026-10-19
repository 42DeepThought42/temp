#pragma once
// td_sim3 — simulator.hpp
//
// The driver owns time. It advances the adaptive odeint integrator in macro
// windows (default 10 ms = 100 Hz), and at each window boundary:
//
//   1. latches sensor snapshots,
//   2. runs each FlightComputer's IFlightSoftware (discrete tick),
//   3. proposes new command latches,
//   4. commits all Latched signals (current <- next) — the 1-step delay
//      boundary,
//   5. checks jettison event triggers.
//
// Inside a window, discrete state is frozen: odeint's speculative trial steps
// never see it change (the dual-domain temporal isolator).
//
// Jettison: on trigger, stop the integrator, mark the stage + its models
// inactive, rebuild the stepper (adaptive history is invalid across a
// discontinuity), re-resolve the surviving topology, and resume.

// NOTE: <type_traits> MUST be included before <memory> to work around a GCC 16
// libstdc++ header-ordering bug (stl_tree.h references type traits unqualified
// when pulled in via memoryfwd.h).
#include <string>
#include <type_traits>
#include <memory>
#include <vector>

#include "factory.hpp"
#include "flight_software.hpp"
#include "signal.hpp"
#include "simparams.hpp" // SimParams
#include "system.hpp"
#include "telemetry.hpp"
#include "topology.hpp"

namespace tdsim {

// A jettison event trigger: fires when a named condition is true at a window
// boundary. The trigger is a pure function of time + latched/ODE state.
struct JettisonEvent {
    std::string stage_id;
    std::function<bool(double t, const SystemFunctor& sys)> trigger;
    bool fired = false;
};

class Simulator {
public:
    // Takes ownership of the built system (moves the unique_ptr) and keeps the
    // model bag + storage alive via the BuiltSystem members it retains.
    explicit Simulator(BuiltSystem built, SimParams params);

    // Non-copyable and non-movable: the simulator holds a reference to the
    // BuiltSystem's SystemFunctor and owns the live ODE state + telemetry file.
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;
    Simulator(Simulator&&) = delete;
    Simulator& operator=(Simulator&&) = delete;

    void add_flight_computer(const std::string& model_id,
                             std::unique_ptr<IFlightSoftware> sw,
                             const std::vector<std::string>& sensor_signals,
                             const std::vector<std::string>& command_signals);
    void add_jettison_event(JettisonEvent ev);

    void run();

    // Access the live ODE state (for inspection / tests).
    std::vector<double>& state() { return sys_.state(); }
    const std::vector<double>& state() const { return sys_.state(); }

    // The flight computer's actual inter-tick interval (s). Equals dt_macro in
    // the single-rate case (base_hz==0); equals period_ticks*base_period when
    // multi-rate is active. The example uses this to seed the PID's dt so the
    // derivative term matches the rate at which update() is actually called.
    double fc_dt() const {
        return computers_.empty() ? params_.dt_macro : computers_[0].dt;
    }

private:
    void discrete_tick(double t);
    void commit_latches();
    void check_events(double t);
    void handle_jettison(const JettisonEvent& ev);
    void build_telemetry_columns();

    BuiltSystem built_;
    SystemFunctor& sys_;
    SimParams params_;

    // Flight computers: model id -> (software, sensor signal names, command signal names).
    // period_ticks: how many base ticks between FC ticks (1 = every base tick,
    // the original behavior). When the driver's base tick equals dt_macro
    // (single-rate, base_hz==0), period_ticks is always 1 and the FC ticks
    // every window exactly as before. When multi-rate is active (base_hz>0),
    // the base tick is finer and the FC ticks only every period_ticks base
    // ticks (its own rate). The PID derivative's dt is the FC's actual
    // inter-tick interval (period_ticks * base_period), not the base period.
    struct FlightComputer {
        std::string model_id;
        std::unique_ptr<IFlightSoftware> sw;
        std::vector<std::string> sensor_signals;
        std::vector<std::string> command_signals;
        std::size_t period_ticks = 1;
        double dt = 0.0; // FC's inter-tick interval (s); set by the driver
    };
    std::vector<FlightComputer> computers_;

    // The resolved multi-rate base rate (Hz), copied from BuiltSystem.
    // 0 = single-rate (driver steps at 1/dt_macro); >0 = multi-rate (driver
    // steps at 1/base_hz, which is finer than or equal to dt_macro).
    std::size_t base_hz_ = 0;
    // The driver's base tick period (s): 1/base_hz when multi-rate, else dt_macro.
    double base_period() const { return base_hz_ > 0 ? 1.0 / static_cast<double>(base_hz_) : params_.dt_macro; }

    std::vector<JettisonEvent> events_;

    // Latches to commit at each window boundary.
    std::vector<Latched<double>*> latches_;

    TelemetryLogger telemetry_;
    double next_tick_ = 0.0;
    double next_telem_ = 0.0;
    double t_at_jettison_ = 0.0;

    // Live ODE state (owned by the driver; the core's x_ mirrors it).
    std::vector<double> x_live_;
};

} // namespace tdsim
