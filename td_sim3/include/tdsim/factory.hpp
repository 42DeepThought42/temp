#pragma once
// td_sim3 — factory.hpp
//
// The factory turns a declarative vehicle description (VehicleSpec, loaded
// from TOML) into a running SystemFunctor + SimulationBuilder.
//
// Extensibility is registry-driven. The CORE (this header + factory.cpp)
// knows nothing about concrete model types. A model type is added by calling
// register_model_type() once (normally in main, or via
// register_builtin_types() for the reference set), supplying:
//
//   1. a constructor + descriptor — creates the model instance;
//   2. a ports table  — every signal port the model exposes (name, kind,
//                        role, and a binder that attaches the port to the
//                        model instance). Port names are stable and
//                        documented per type; a full signal name in the
//                        config is "<model_id>.<port_name>".
//   3. param setters  — the model-type-specific numeric parameters accepted
//                        in [models.<id>] (unknown config keys are rejected);
//   4. optional hooks — initial ODE state, accumulator registration (Pass 1
//                        clears these), telemetry readers (also the telemetry
//                        column set), discrete command proposers (the flight
//                        computer's write surface), and latch collection.
//
// The config file then references the type by name; the factory instantiates,
// binds, and wires everything generically.
//
// Downcasting: StatefulModel<Dim> for different Dim are unrelated types, so
// the concrete model lives INSIDE its ConcreteSlot (unique_ptr<T>). The slot
// exposes the erased pointer via model(); every hook lambda captures the
// concrete type at registration time and downcasts slot.model() inside (safe
// by construction — there is exactly one object).

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "spec.hpp"
#include "system.hpp"
#include "topology.hpp"

namespace tdsim {

// Metadata about the built model instances. The bag does NOT own the models:
// the SimulationCore owns the slots (and the concrete models inside them) via
// std::shared_ptr<ModelSlot>. The bag is a lookup table that lets the factory
// (and tests) find a model's slot by id after the core has taken ownership.
struct ModelBag {
    struct Entry {
        std::string id;
        std::string type;
        std::string stage;
        std::size_t state_dim = 0;
    };
    std::vector<Entry> entries;
};

// Look up a model's owning slot by id within a slot table (e.g. the core's).
// Returns nullptr if the id is not present.
inline ModelSlot* find_slot(const std::string& id, const std::vector<std::shared_ptr<ModelSlot>>& slots) {
    for (const auto& s : slots) {
        if (s->id == id) return s.get();
    }
    return nullptr;
}

// A latch the driver must commit, annotated with its multi-rate cadence.
// period_ticks is the number of base ticks between commits (1 = every base
// tick, the original single-rate behavior; see Latched::period_ticks).
struct LatchRef {
    Latched<double>* latch;
    std::size_t period_ticks = 1;
};

// The built, wired system.
struct BuiltSystem {
    std::unique_ptr<SimulationCore> core;   // owns slots (and the models in them), ODE state, accessor tables
    SystemFunctor sys;                      // copyable odeint system -> core
    SimulationBuilder builder;
    // Metadata about the built model instances (id/type/stage/dim lookup).
    // The concrete models themselves are owned by the core's slots.
    std::shared_ptr<ModelBag> models;
    // Latches the driver commits when their period is due (see LatchRef).
    std::vector<LatchRef> latches;
    // The resolved multi-rate base rate (Hz). 0 = no multi-rate subsystems;
    // the driver then steps at 1/dt_macro (single-rate, original behavior).
    // When > 0, the driver's base tick is 1/base_hz (finer than or equal to
    // dt_macro), and every discrete period is an integer number of base ticks.
    std::size_t base_hz = 0;
    // Keeps the algebraic SlotStore alive (signal views point into it).
    std::shared_ptr<void> storage;
};

// Registration record for one model TYPE (not an instance).
struct ModelType {
    std::size_t state_dim = 0;
    // Creates a new instance AND its owning slot. The slot is the sole owner
    // of the concrete model object; the core drives eval() through it.
    std::function<std::unique_ptr<ModelSlot>()> construct;
    // One signal port on the model.
    struct Port {
        std::string name; // primary config-facing name (without the model id prefix)
        std::vector<std::string> aliases; // alternate config-facing names for this same slot
        SignalKind kind;  // Signal / Accumulator / Latched
        bool produced;    // true if this model is the producing side
        bool matches(const std::string& n) const {
            if (n == name) return true;
            for (const auto& a : aliases) if (n == a) return true;
            return false;
        }
        // Attach the port to this instance. cur/nxt are the slot addresses
        // (Latched uses both; Signal/Accumulator use cur only).
        std::function<void(ModelSlot&, double* cur, double* nxt)> bind;
    };
    std::vector<Port> ports;
    // Model-type-specific numeric parameters: name -> setter on the instance.
    std::map<std::string, std::function<void(ModelSlot&, double)>> params;
    // Initial ODE state for a fresh instance (called after params are applied).
    // Default (unset): all zeros.
    std::function<std::vector<double>(ModelSlot&)> initial_state;
    // Pass-1 accumulators this instance owns or feeds (the framework clears
    // them on every derivative call).
    std::function<void(ModelSlot&, std::vector<Accumulator<double>*>&)> accumulators;
    // Telemetry readers: named getters (registered readers ARE the telemetry
    // column set, alongside the ODE state columns).
    std::function<void(ModelSlot&, const std::function<void(const std::string&, std::function<double()>)>&)> readers;
    // Discrete command proposers: named command sinks (flight computer writes).
    std::function<void(ModelSlot&, const std::function<void(const std::string&, std::function<void(double)>)>&)> proposers;
    // Latched ports this instance owns, each annotated with its commit period
    // in base ticks (1 = every base tick). The driver commits a latch only
    // when (base_tick_index % period_ticks == 0).
    std::function<void(ModelSlot&, std::vector<LatchRef>&)> collect_latches;
    // Post-param setup (runs after params are applied AND port slots are
    // bound; e.g. initialize latches from derived values). Unset: no-op.
    std::function<void(ModelSlot&)> post_configure;
};

class VehicleFactory {
public:
    // Register a model type by name. Re-registering the same name replaces
    // the previous registration (idempotent; handy for tests).
    static void register_model_type(const std::string& type_name, ModelType def);

    // Register all built-in reference types (rigid-body vehicle, propellant
    // tank, liquid engine, first-order actuator, pressure sensor). Safe to
    // call repeatedly.
    static void register_builtin_types();

    // Build the wired system from a loaded spec. Throws std::runtime_error on
    // unknown model types, unknown parameters, unknown ports, duplicate model
    // ids, or signal-contract violations.
    static BuiltSystem build_from_spec(const VehicleSpec& spec);

    // Build the reference example vehicle programmatically (test oracle).
    // It is the same vehicle the reference config config/vehicle.toml
    // describes, so the two build paths must produce equivalent systems.
    static BuiltSystem build_example_vehicle();

    // Access the type registry (tests / introspection / --list-types).
    static const std::unordered_map<std::string, ModelType>& model_types();

    // -----------------------------------------------------------------------
    // Multi-rate base-tick resolution (divisible-only clock tree).
    //
    // The sim runs on ONE base tick. Every discrete subsystem (flight computer,
    // multi-rate sensor latch) AND the macro window itself must tick at an
    // integer divisor of the base, so "is it tick time?" is k % period_ticks
    // == 0 (a branch, never a floating-point epsilon test) and no clock
    // drifts. The base rate is the GCD of the macro-window rate (1/dt_macro,
    // Hz) and every discrete rate: base_hz = GCD(window_hz, all rates),
    // base_period = 1/base_hz, and each subsystem's period_ticks = base_hz /
    // rate (an integer by construction). Including the window rate is what
    // keeps the base tick from being coarser than the window (which would
    // under-sample the FC) — e.g. a 100 Hz window + 25 Hz sensor -> base 25 Hz
    // -> window period 4, sensor period 1.
    //
    // This is the divisible-only contract: rates that are not integer Hz, or
    // whose GCD base tick is finer than the integrator's dt_max, are rejected
    // at build time with the offending values named. (A 100 Hz FC next to a
    // 37 Hz sensor has GCD 1 Hz -> a 1 s base tick, which this rejects rather
    // than silently re-forming the window grid.)
    static std::size_t resolve_base_hz(const std::vector<std::size_t>& rates_hz);
    // Returns the per-rate period_ticks map (one entry per discrete rate), or
    // throws if any rate is invalid. window_hz is the macro-window rate
    // (1/dt_macro); it is folded into the GCD so the base divides it too.
    // dt_max (s) is the integrator's max substep; the base tick must be >= it.
    static std::map<std::size_t, std::size_t>
    resolve_periods(const std::vector<std::size_t>& rates_hz, std::size_t window_hz,
                    double dt_max);
};

} // namespace tdsim
