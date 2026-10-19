// td_sim3 — examples/td_sim.cpp
//
// The command-line entry point for the template. The vehicle, its models,
// parameters, signal wiring, events, flight software, and integrator settings
// all come from a TOML config file — nothing about the vehicle is baked into
// this program or the tdsim core. The only model knowledge in the executable
// is the registry of available model types and flight software (both in the
// tdsim library: registry_builtin.cpp and registry_flight.cpp), which the
// config file references by name. This executable only looks them up by name.
//
// Usage:
//   ./td_sim <config.toml>
//   ./td_sim --check <config.toml>     # load + build + topology check, no run
//   ./td_sim --list-types              # list registered model types & software

#include "tdsim/flight_software.hpp"
#include "tdsim/spec.hpp"
#include "tdsim/factory.hpp"
#include "tdsim/simulator.hpp"

#include <spdlog/spdlog.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace tdsim;

namespace {

// Build the flight software instance named by the spec and apply its params.
// The driver first seeds the software's discrete-tick period from
// [sim_params].dt_macro (the rate update() is actually called at), then the
// config's [flight_computer.params] table is applied on top (a 'dt_macro'
// entry there overrides the seeded value).
std::unique_ptr<IFlightSoftware> build_flight_software(const FlightComputerSpec& fc,
                                                       double dt_macro) {
    if (fc.software.empty()) {
        throw std::runtime_error("[flight_computer]: missing required key 'software' "
                                 "(name of the flight software class)");
    }
    auto* make = FlightSoftwareRegistry::find(fc.software);
    if (make == nullptr) {
        std::string known;
        for (const auto& [k, _] : FlightSoftwareRegistry::software()) known += " '" + k + "'";
        throw std::runtime_error("[flight_computer].software '" + fc.software +
                                 "' is not registered. Available:" + known);
    }
    auto sw = (*make)();
    // Seed the discrete-tick period. set_param may throw on unknown names for
    // software that doesn't accept params, so only call it if the software
    // explicitly supports 'dt_macro' (VehicleFlightSoftware does; a future
    // software that doesn't will throw, which is the desired strictness).
    try {
        sw->set_param("dt_macro", dt_macro);
    } catch (const std::exception&) {
        // Software that rejects 'dt_macro' simply doesn't use a tick period;
        // that's fine — it has no PID derivative term to drive.
    }
    for (const auto& [k, v] : fc.params) {
        sw->set_param(k, v);
    }
    return sw;
}

void list_types() {
    // Populate both registries (idempotent) so --list-types shows the full set.
    VehicleFactory::register_builtin_types();
    FlightSoftwareRegistry::register_builtin_software();
    auto& types = VehicleFactory::model_types();
    std::cout << "Registered model types:\n";
    for (const auto& [name, mt] : types) {
        std::cout << "  " << name << " (state_dim=" << mt.state_dim << ")\n";
        std::cout << "    ports:";
        for (const auto& p : mt.ports) {
            std::cout << " " << p.name << (p.produced ? "(out)" : "(in)");
        }
        std::cout << "\n    params:";
        for (const auto& [k, _] : mt.params) std::cout << " " << k;
        std::cout << "\n";
    }
    std::cout << "Registered flight software:\n";
    for (const auto& [k, _] : FlightSoftwareRegistry::software()) std::cout << "  " << k << "\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string config_path;
    bool check_only = false;
    bool list = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--check") check_only = true;
        else if (arg == "--list-types") list = true;
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--check] <config.toml>\n"
                      << "       " << argv[0] << " --list-types\n";
            return 0;
        } else {
            if (!config_path.empty() && !check_only) {
                std::cerr << "error: unexpected argument '" << arg << "'\n";
                return 2;
            }
            config_path = arg;
        }
    }

    if (list) {
        list_types();
        return 0;
    }
    if (config_path.empty()) {
        std::cerr << "error: no config file given.\n"
                  << "Usage: " << argv[0] << " [--check] <config.toml>\n";
        return 2;
    }

    try {
        // Load + validate the config (strict: file:line context on any error).
        VehicleSpec spec = load_spec(config_path);

        // Register the available model types and build the vehicle from the
        // spec. This also resolves the signal topology and emits the
        // engineering data-flow report.
        VehicleFactory::register_builtin_types();
        FlightSoftwareRegistry::register_builtin_software();
        BuiltSystem built = VehicleFactory::build_from_spec(spec);

        if (check_only) {
            std::cout << "[check] OK: config loaded, models instantiated, topology resolved\n";
            return 0;
        }

        // Run the simulation.
        Simulator sim(std::move(built), spec.sim);

        // Attach the flight computer named in the config (one per vehicle).
        if (spec.flight_computer.has_value()) {
            auto sw = build_flight_software(*spec.flight_computer, spec.sim.dt_macro);
            sim.add_flight_computer(
                spec.flight_computer->model, std::move(sw),
                spec.flight_computer->sensors, spec.flight_computer->commands);
        }

        // Attach discrete events (stage jettison at fixed time, currently).
        for (const auto& ev : spec.events) {
            if (ev.type != "fixed_time") {
                throw std::runtime_error("event '" + ev.id +
                                         "': unsupported trigger type '" + ev.type + "'");
            }
            double t_trigger = ev.t;
            sim.add_jettison_event(JettisonEvent{
                ev.stage,
                [t_trigger](double t, const SystemFunctor&) { return t >= t_trigger; },
                false,
            });
        }

        sim.run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}
