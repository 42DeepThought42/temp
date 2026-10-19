#pragma once
// td_sim3 — flight_software.hpp
//
// The flight software is a strategy object run by the FlightComputer model at
// the fixed discrete rate (default 100 Hz). It is swappable at runtime via the
// config file. The interface is deliberately minimal: a pure function of the
// current sensor snapshot and time, returning a command snapshot. It has no
// access to the continuous state vector — it sees only latched sensor values,
// which is what keeps it isolated from odeint's speculative steps.

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <map>
#include <stdexcept>

namespace tdsim {

// Named sensor inputs, as read from latched sensor signals at the tick.
using SensorInputs = std::map<std::string, double>;

// Named command outputs, applied to latched command registers at the tick.
using CommandOutputs = std::map<std::string, double>;

class IFlightSoftware {
public:
    virtual ~IFlightSoftware() = default;

    // Called once per discrete tick. Pure: no mutation of external state.
    virtual CommandOutputs update(double t, const SensorInputs& sensors) = 0;

    // Optional telemetry: the software may expose internal state (e.g. filter
    // states, mode) for the telemetry log. Return an empty map if none.
    virtual SensorInputs getTelemetry() const { return {}; }

    // Name for the factory registry.
    virtual std::string name() const = 0;

    // Configure a named software parameter from the [flight_computer].params
    // table (e.g. throttle setpoint, PID gains). Throwing on unknown names is
    // the default: the config loader rejects typos. Implementations override
    // this to accept their own parameter names.
    virtual void set_param(const std::string& /*name*/, double /*value*/) {
        throw std::runtime_error("flight software " + std::string(name()) +
                                 " does not accept any params "
                                 "([flight_computer.params] must be empty for this software)");
    }
};

// ---------------------------------------------------------------------------
// VehicleFlightSoftware — the single flight software for the example vehicle.
// A vehicle contains exactly ONE flight computer running ONE flight software
// instance. That instance may internally run multiple control loops (throttle
// management, gimbal stabilization, mode logic, etc.) and propose commands to
// any number of actuators. This is the canonical "one brain, many loops"
// pattern.
// ---------------------------------------------------------------------------
class VehicleFlightSoftware : public IFlightSoftware {
public:
    // Throttle: open-loop hold (demo). A real implementation would close a
    // throttle loop or follow a mission profile.
    double throttle_setpoint = 1.0;

    // Gimbal: PID hold (demo). Keeps the gimbal at its setpoint (0 = aligned).
    double gimbal_kp = 10.0;
    double gimbal_ki = 0.0;
    double gimbal_kd = 0.0;
    double gimbal_setpoint = 0.0;
    double gimbal_integral = 0.0;
    double gimbal_prev_error = 0.0;
    // Discrete-tick period (s). Used by the PID derivative term. MUST be set
    // by the driver from [sim_params].dt_macro — it is the rate at which
    // update() is actually called. Configurable via the 'dt_macro' param so
    // a config file can override the driver's default (e.g. for testing).
    double dt = 0.01;

    CommandOutputs update(double t, const SensorInputs& sensors) override {
        (void)t;
        CommandOutputs cmds;

        // Throttle command (open-loop hold for the demo).
        cmds["throttle_act.cmd"] = throttle_setpoint;

        // Gimbal PID.
        double measured = sensors.count("gimbal_act.position")
                              ? sensors.at("gimbal_act.position") : 0.0;
        double error = gimbal_setpoint - measured;
        gimbal_integral += error * dt;
        double derivative = (error - gimbal_prev_error) / dt;
        gimbal_prev_error = error;
        cmds["gimbal_act.cmd"] =
            gimbal_kp * error + gimbal_ki * gimbal_integral + gimbal_kd * derivative;

        return cmds;
    }

    std::string name() const override { return "VehicleFlightSoftware"; }

    // Configurable from [flight_computer].params (unknown names are an error).
    void set_param(const std::string& name, double v) override {
        if (name == "throttle_setpoint") throttle_setpoint = v;
        else if (name == "gimbal_setpoint") gimbal_setpoint = v;
        else if (name == "gimbal_kp") gimbal_kp = v;
        else if (name == "gimbal_ki") gimbal_ki = v;
        else if (name == "gimbal_kd") gimbal_kd = v;
        else if (name == "dt_macro") dt = v; // override the driver's dt_macro
        else throw std::runtime_error("VehicleFlightSoftware: unknown param '" + name + "'");
    }

    // Telemetry: expose the gimbal PID's internal state.
    SensorInputs getTelemetry() const override {
        return {{"gimbal_pid.integral", gimbal_integral},
                {"gimbal_pid.prev_error", gimbal_prev_error}};
    }
};

// ---------------------------------------------------------------------------
// FlightSoftwareRegistry — the factory for flight software by name, mirroring
// VehicleFactory's model-type registry. Concrete flight-software knowledge
// (which classes exist and how to construct them) lives in a separate TU
// (src/registry_flight.cpp), keeping the core and the example free of it. The
// config's [flight_computer].software names an entry here.
// ---------------------------------------------------------------------------
class FlightSoftwareRegistry {
public:
    // Register a flight software by name. Re-registering the same name
    // replaces the previous registration (idempotent; handy for tests).
    static void register_software(const std::string& name,
                                  std::function<std::unique_ptr<IFlightSoftware>()> make);

    // Register all built-in flight software. Safe to call repeatedly.
    static void register_builtin_software();

    // Look up a factory by name. Returns nullptr if not registered.
    static const std::function<std::unique_ptr<IFlightSoftware>()>*
    find(const std::string& name);

    // Access the full registry (introspection / --list-types / tests).
    static const std::map<std::string, std::function<std::unique_ptr<IFlightSoftware>()>>&
    software();
};

} // namespace tdsim
