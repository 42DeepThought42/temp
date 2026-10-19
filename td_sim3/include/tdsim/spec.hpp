#pragma once
// td_sim3 — spec.hpp
//
// The typed vehicle specification. This is the single intermediate
// representation between the user-facing TOML document and the
// factory/topology APIs: the loader (config.cpp) parses and validates a TOML
// file into this struct; the factory (factory.cpp) consumes it to build the
// wired system.
//
// Design notes:
//   * The TOML document is the user-facing format (human-authored, commentable).
//     There is NO intermediate JSON step — the typed struct IS the IR. If a
//     machine-readable form is ever needed, toml++ can format this back to
//     TOML (see --dump-config).
//   * Strict validation at load time (unknown sections/keys rejected) is the
//     "compilation" of the config: a spec that fails to load is invalid, full
//     stop.
//   * Model parameters are kept as a raw key->double map (toml::value) so the
//     loader does not need to know each model type's parameter set; the
//     factory's per-type apply_params() interprets them and rejects unknown
//     keys with the model name in the error.

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "signal.hpp"
#include "simparams.hpp" // SimParams

namespace tdsim {

// One model instance: [models.<id>] in the TOML document.
struct ModelSpec {
    std::string id;
    std::string type;   // registered model type name (e.g. "PropellantTank")
    std::string stage;  // empty for vehicle-level models
    std::map<std::string, double> params; // model-type-specific parameters
};

// One named signal: [[connections]] in the TOML document. The signal name
// ("producer.port") identifies the port on the producing model; kind selects
// the signal contract (Signal / Accumulator / Latched).
struct ConnectionSpec {
    std::string signal;    // "<producer_id>.<producer_port>"
    std::string to_port;   // optional consumer-side port name (defaults to producer_port)
    SignalKind kind = SignalKind::Signal;
    std::vector<std::string> producers; // exactly 1 for Signal/Latched, 0+ for Accumulator
    std::vector<std::string> consumers;
};

// A jettison event trigger. type = "fixed_time" (fires at t >= t).
// More trigger types can be added (e.g. "tank_empty") in the factory.
struct EventSpec {
    std::string id;
    std::string type = "fixed_time";
    std::string stage;
    double t = 0.0;
};

// Flight computer config: [flight_computer] in the TOML document.
// Exactly one flight computer per vehicle, running one flight software
// instance (the software may internally run multiple control loops).
struct FlightComputerSpec {
    std::string model = "flightcomp"; // the model id the FC is attached to
    std::string software;             // registered flight software name (required)
    std::vector<std::string> sensors;
    std::vector<std::string> commands;
    std::map<std::string, double> params; // software-specific parameters
};

// The complete typed vehicle specification.
struct VehicleSpec {
    SimParams sim;
    std::vector<ModelSpec> models;
    std::vector<ConnectionSpec> connections;
    std::vector<EventSpec> events;
    std::optional<FlightComputerSpec> flight_computer; // present if [flight_computer] was given
};

// Parse one TOML file into a spec. Throws std::runtime_error with the file
// name + line on any parse or validation error.
VehicleSpec load_spec(const std::string& path);

// Parse a TOML document from memory (tests). Throws std::runtime_error.
VehicleSpec parse_spec(const std::string& toml_text, const std::string& source_name = "<memory>");

} // namespace tdsim
