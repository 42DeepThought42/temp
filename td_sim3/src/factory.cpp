// td_sim3 — factory.cpp
//
// Registry-driven vehicle assembly. The generic build_from_spec() path knows
// nothing about concrete model types; it dispatches through the ModelType
// registry. register_builtin_types() registers the reference model set for
// this template (rigid-body vehicle, propellant tank, liquid engine,
// first-order actuator, pressure sensor).
//
// Adding a new model type (the ritual, kept small per SESSION_NOTES §3.6):
//   1. write the struct deriving StatefulModel<Dim> (see models.hpp);
//   2. add one register_model_type() block: construct, ports, params, and the
//      optional hooks (initial_state, post_configure, accumulators, readers,
//      proposers, collect_latches);
//   3. instantiate + wire it in the vehicle config.

#include "tdsim/factory.hpp"
// (Built-in model-type registrations live in registry_builtin.cpp; this file
//  is the generic build pipeline and needs no concrete model headers.)

#include <spdlog/spdlog.h>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace tdsim {

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

namespace {
std::unordered_map<std::string, ModelType>& registry() {
    static std::unordered_map<std::string, ModelType> r;
    return r;
}

const char* kind_name(SignalKind k) {
    switch (k) {
        case SignalKind::Signal: return "Signal";
        case SignalKind::Accumulator: return "Accumulator";
        case SignalKind::Latched: return "Latched";
    }
    return "?";
}
} // namespace

void VehicleFactory::register_model_type(const std::string& type_name, ModelType def) {
    if (!def.construct) {
        throw std::runtime_error("register_model_type: registration for '" + type_name +
                                 "' has no constructor");
    }
    registry()[type_name] = std::move(def);
}

const std::unordered_map<std::string, ModelType>& VehicleFactory::model_types() {
    return registry();
}

namespace {
const ModelType* require_type(const std::string& type, const std::string& where) {
    auto it = registry().find(type);
    if (it == registry().end()) {
        throw std::runtime_error(where + ": unknown model type '" + type + "'");
    }
    return &it->second;
}
} // namespace

// ---------------------------------------------------------------------------
// Generic build
// ---------------------------------------------------------------------------

namespace {

// One model instance as assembled during the build.
struct Instance {
    const ModelSpec* spec;
    const ModelType* type;
    std::shared_ptr<ModelSlot> slot;         // OWNS the concrete model (slot->model())
    std::size_t state_offset = 0;
    std::vector<double> x0;                  // initial ODE state slice
    struct PortBinding {
        const ModelType::Port* port;
        double* cur = nullptr;
        double* nxt = nullptr; // Latched only
    };
    std::vector<PortBinding> ports;
};

} // namespace

BuiltSystem VehicleFactory::build_from_spec(const VehicleSpec& spec) {
    // ---- 1. Instantiate models, apply params, bind ports ------------------
    auto bag = std::make_shared<ModelBag>();
    std::vector<Instance> instances;
    instances.reserve(spec.models.size());

    auto store = std::make_shared<SlotStore>(1024);
    std::size_t slot_i = 0;
    auto new_slot = [&]() { return static_cast<double*>(store->slot(slot_i++)); };

    for (const auto& ms : spec.models) {
        const ModelType* mt = require_type(ms.type, "model '" + ms.id + "'");
        Instance inst;
        inst.spec = &ms;
        inst.type = mt;

        inst.slot = std::shared_ptr<ModelSlot>(mt->construct());
        if (!inst.slot) {
            throw std::runtime_error("model '" + ms.id + "': constructor for type '" +
                                     ms.type + "' returned null");
        }
        inst.slot->id = ms.id;
        inst.slot->stage_id = ms.stage;
        inst.slot->state_dim = mt->state_dim;

        // Apply model-type-specific parameters (unknown keys are an error).
        for (const auto& [k, v] : ms.params) {
            auto it = mt->params.find(k);
            if (it == mt->params.end()) {
                throw std::runtime_error("model '" + ms.id + "' (type " + ms.type +
                                         "): unknown parameter '" + k + "'");
            }
            it->second(*inst.slot, v);
        }

        // Initial ODE state slice.
        inst.x0 = mt->initial_state ? mt->initial_state(*inst.slot)
                                    : std::vector<double>(mt->state_dim, 0.0);
        if (inst.x0.size() != mt->state_dim) {
            throw std::runtime_error("model '" + ms.id + "': initial state has " +
                                     std::to_string(inst.x0.size()) + " values, expected " +
                                     std::to_string(mt->state_dim));
        }

        // Allocate one slot per port and bind the view in place.
        for (const auto& port : mt->ports) {
            Instance::PortBinding pb;
            pb.port = &port;
            pb.cur = new_slot();
            if (port.kind == SignalKind::Latched) pb.nxt = new_slot();
            port.bind(*inst.slot, pb.cur, pb.nxt);
            inst.ports.push_back(pb);
        }
        // Latch init from the model's own initial conditions. Must run before
        // any consumer could read the latch; the committed 'current' value is
        // what the engine sees for the very first derivative call.
        if (mt->post_configure) {
            mt->post_configure(*inst.slot);
        }

        // The bag keeps type metadata (for the post-build hooks below).
        // The slot itself is moved into the core in step 6; the core owns it
        // (and the concrete model inside it) for the lifetime of the sim.
        ModelBag::Entry entry;
        entry.id = ms.id;
        entry.type = ms.type;
        entry.stage = ms.stage;
        entry.state_dim = mt->state_dim;
        bag->entries.push_back(std::move(entry));
        instances.push_back(std::move(inst));
    }

    // ---- 2. Allocate the ODE state vector ---------------------------------
    std::vector<double> x;
    for (auto& inst : instances) {
        inst.state_offset = x.size();
        x.resize(x.size() + inst.type->state_dim, 0.0);
        for (std::size_t i = 0; i < inst.x0.size(); i++) {
            x[inst.state_offset + i] = inst.x0[i];
        }
        inst.slot->state_offset = inst.state_offset;
    }

    // ---- 3. Register models in the builder --------------------------------
    SimulationBuilder builder;
    for (const auto& inst : instances) {
        builder.register_model(inst.spec->id, inst.spec->stage);
    }

    // ---- 4. Resolve connections --------------------------------------------
    // Each [[connections]] entry names the signal "<producer_id>.<port>".
    // Every consumer must expose a port with the same port NAME; its view is
    // bound to the producer's slot address(es).
    std::unordered_map<std::string, Instance*> by_id;
    for (auto& inst : instances) by_id[inst.spec->id] = &inst;

    for (const auto& cs : spec.connections) {
        auto dot = cs.signal.find('.');
        if (dot == std::string::npos) {
            throw std::runtime_error("connection '" + cs.signal +
                                     "': signal must be named '<model_id>.<port_name>'");
        }
        std::string producer_id = cs.signal.substr(0, dot);
        std::string port_name = cs.signal.substr(dot + 1);

        auto pit = by_id.find(producer_id);
        if (pit == by_id.end()) {
            throw std::runtime_error("connection '" + cs.signal + "': unknown producer model '" +
                                     producer_id + "'");
        }
        Instance* producer = pit->second;

        const ModelType::Port* prod_port = nullptr;
        Instance::PortBinding* prod_bind = nullptr;
        for (auto& pb : producer->ports) {
            if (pb.port->matches(port_name)) {
                prod_port = pb.port;
                prod_bind = &pb;
                break;
            }
        }
        if (!prod_port) {
            throw std::runtime_error("connection '" + cs.signal + "': model '" + producer_id +
                                     "' (type " + producer->spec->type +
                                     ") has no port named '" + port_name + "'");
        }
        if (prod_port->kind != cs.kind) {
            throw std::runtime_error("connection '" + cs.signal + "': port '" + port_name +
                                     "' is a " + kind_name(prod_port->kind) +
                                     ", connection declares " + kind_name(cs.kind));
        }
        if (cs.kind == SignalKind::Signal || cs.kind == SignalKind::Latched) {
            if (cs.producers.size() != 1) {
                throw std::runtime_error("connection '" + cs.signal + "': " + kind_name(cs.kind) +
                                         " requires exactly 1 producer (got " +
                                         std::to_string(cs.producers.size()) + ")");
            }
            if (cs.producers[0] != producer_id) {
                throw std::runtime_error("connection '" + cs.signal +
                                         "': producer listed as '" + cs.producers[0] +
                                         "' but signal names '" + producer_id + "'");
            }
        }
        if (cs.consumers.empty()) {
            throw std::runtime_error("connection '" + cs.signal + "': no consumers listed");
        }

        // Consumer-side port name: use 'to' if specified, else the producer port name.
        std::string cons_port = cs.to_port.empty() ? port_name : cs.to_port;
        for (const auto& cid : cs.consumers) {
            auto cit = by_id.find(cid);
            if (cit == by_id.end()) {
                throw std::runtime_error("connection '" + cs.signal + "': unknown consumer model '" +
                                         cid + "'");
            }
            Instance* consumer = cit->second;
            Instance::PortBinding* cons_bind = nullptr;
            for (auto& pb : consumer->ports) {
                if (pb.port->matches(cons_port)) {
                    cons_bind = &pb;
                    break;
                }
            }
            if (!cons_bind) {
                throw std::runtime_error("connection '" + cs.signal + "': consumer '" + cid +
                                         "' (type " + consumer->spec->type +
                                         ") has no port named '" + cons_port + "'");
            }
            // Bind the producer + consumer views to a single shared slot.
            //
            // For Signal/Latched: the producer owns the slot; the consumer's
            // view points at it (consumer is read-only).
            //
            // For Accumulator: the CONSUMER owns the slot (it is the summing
            // sink). Every producer's port is re-bound to the consumer's slot
            // so that all producers' add() calls accumulate into the same
            // slot the consumer reads. This is essential when one consumer
            // has multiple producers (e.g. vehicle.total_mass fed by both
            // fuel_tank.total_mass_contrib and oxi_tank.total_mass_contrib):
            // binding the consumer to a single producer's slot would orphan
            // the other producers' contributions.
            if (cs.kind == SignalKind::Accumulator) {
                // Consumer's slot is canonical. Re-bind the producer's port
                // (and its PortBinding) to the consumer's slot address.
                prod_bind->cur = cons_bind->cur;
                prod_bind->nxt = cons_bind->nxt;
                prod_bind->port->bind(*producer->slot, prod_bind->cur, prod_bind->nxt);
                // The consumer keeps its own slot (already bound at config
                // time); no re-bind needed.
            } else {
                // Signal/Latched: consumer points at the producer's slot.
                cons_bind->cur = prod_bind->cur;
                cons_bind->nxt = prod_bind->nxt;
                cons_bind->port->bind(*consumer->slot, cons_bind->cur, cons_bind->nxt);
            }
            // Builder wiring (topology + contract validation).
            builder.wire(cs.signal, cs.kind, producer_id, cid);
        }
    }

    // ---- 5. Resolve topology (validates + sorts + reports) ------------------
    builder.resolve_topology();

    // ---- 6. Assemble core + functor -----------------------------------------
    std::vector<std::shared_ptr<ModelSlot>> slots;
    for (auto& inst : instances) {
        slots.push_back(std::move(inst.slot));
    }
    auto eval_order = builder.evaluation_order();
    auto core = std::make_unique<SimulationCore>(std::move(slots), std::move(eval_order), std::move(x));
    for (auto& s : core->slots()) {
        core->register_slot(s->id, s.get());
    }

    // Per-type hooks: accumulators, readers, proposers, latches.
    // Resolve each bag entry's slot through the core (the core now owns it).
    // We also record, for each model that declares a discrete 'rate' (Hz)
    // param, the index range of the latches it contributed, so the base-tick
    // resolver below can set the period on exactly the right latch(es).
    std::vector<LatchRef> built_latches;
    struct DiscreteRate { std::string id; std::size_t rate_hz; std::size_t latch_begin; std::size_t latch_end; };
    std::vector<DiscreteRate> discrete_rates_;
    for (std::size_t ei = 0; ei < bag->entries.size(); ++ei) {
        const auto& e = bag->entries[ei];
        auto mt_it = registry().find(e.type);
        if (mt_it == registry().end()) continue;
        const ModelType& mt = mt_it->second;
        ModelSlot* slot = find_slot(e.id, core->slots());
        if (!slot) continue; // defensive; can't happen after register_slot
        if (mt.accumulators) {
            std::vector<Accumulator<double>*> accs;
            mt.accumulators(*slot, accs);
            for (auto* a : accs) core->add_accumulator(*a);
        }
        // Reader/proposer names are namespaced by model id so that two
        // instances of the same type (e.g. two actuators) don't collide.
        const std::string id = e.id;
        if (mt.readers) {
            mt.readers(*slot, [&](const std::string& name, std::function<double()> r) {
                core->set_reader(id + "." + name, std::move(r));
            });
        }
        if (mt.proposers) {
            mt.proposers(*slot, [&](const std::string& name, std::function<void(double)> p) {
                core->set_proposer(id + "." + name, std::move(p));
            });
        }
        std::size_t latch_begin = built_latches.size();
        if (mt.collect_latches) {
            mt.collect_latches(*slot, built_latches);
        }
        // Guard: a reported period must be >= 1 (0 would make the driver's
        // k % period_ticks a division by zero).
        for (auto& lr : built_latches) {
            if (lr.period_ticks < 1) {
                throw std::runtime_error("model '" + e.id + "': latch period_ticks must be >= 1");
            }
        }
        // Multi-rate: a model may declare a discrete 'rate' (Hz) param that
        // sets the period of the latch(es) it just collected. Record the rate
        // + the latch index range; all periods are resolved together below via
        // the base-tick GCD so every discrete shares one clock.
        const ModelSpec* ms = nullptr;
        for (const auto& m : spec.models) if (m.id == e.id) { ms = &m; break; }
        if (ms) {
            auto rate_it = ms->params.find("rate");
            if (rate_it != ms->params.end() && rate_it->second > 0.0) {
                discrete_rates_.push_back({e.id,
                                           static_cast<std::size_t>(std::llround(rate_it->second)),
                                           latch_begin, built_latches.size()});
            }
        }
    }

    // ---- 5b. Resolve multi-rate base tick (divisible-only clock tree) ------
    // Every discrete subsystem (a model with a 'rate' param) must tick at an
    // integer divisor of one shared base tick. The base rate is the GCD of the
    // macro-window rate and all discrete rates; each subsystem's period_ticks
    // = base_hz / rate. The driver's base tick MUST equal base_hz (not
    // dt_macro), so that period_ticks matches the driver's k counter. When no
    // multi-rate subsystems exist, base_hz stays 0 and the driver steps at
    // 1/dt_macro (original single-rate behavior).
    std::size_t base_hz_out = 0;
    if (!discrete_rates_.empty()) {
        std::vector<std::size_t> rates;
        rates.reserve(discrete_rates_.size());
        for (const auto& d : discrete_rates_) rates.push_back(d.rate_hz);
        // The macro-window rate (1/dt_macro) is folded into the GCD so the
        // base tick also divides the window — this is what keeps a slower
        // sensor from dragging the base tick coarser than the FC's window.
        std::size_t window_hz =
            (spec.sim.dt_macro > 0.0)
                ? static_cast<std::size_t>(std::llround(1.0 / spec.sim.dt_macro))
                : 0;
        auto periods = VehicleFactory::resolve_periods(rates, window_hz, spec.sim.dt_max);
        for (const auto& d : discrete_rates_) {
            std::size_t p = periods.at(d.rate_hz);
            for (std::size_t i = d.latch_begin; i < d.latch_end; ++i) {
                // Set the period on BOTH the LatchRef (metadata) and the
                // Latched object itself (the driver reads l->period_ticks).
                built_latches[i].period_ticks = p;
                built_latches[i].latch->period_ticks = p;
            }
        }
        // The base rate is the GCD of the window rate and all discrete rates.
        std::vector<std::size_t> all_rates = rates;
        if (window_hz > 0) all_rates.push_back(window_hz);
        base_hz_out = VehicleFactory::resolve_base_hz(all_rates);
    }

    BuiltSystem out;
    out.base_hz = base_hz_out;
    out.core = std::move(core);
    out.sys = SystemFunctor(*out.core);
    out.builder = std::move(builder);
    out.models = bag;
    out.latches = std::move(built_latches);
    out.storage = store;
    return out;
}


// ---------------------------------------------------------------------------
// Multi-rate base-tick resolution (divisible-only clock tree)
// ---------------------------------------------------------------------------
//
// All discrete subsystems share ONE base tick. The base rate (Hz) is the LCM
// (least common multiple) of the subsystem rates AND the macro-window rate,
// so every subsystem's rate is an integer divisor of the base and its period
// is an integer number of base ticks (period_ticks = base_hz / rate >= 1).
// The base must be >= the fastest rate (otherwise that subsystem couldn't tick
// at its rate). LCM({100, 25}) = 100: the FC (100 Hz) ticks every base tick
// (period 1), the sensor (25 Hz) ticks every 4th (period 4). This keeps "is it
// tick time?" a pure integer branch (k % period_ticks == 0) — no floating-point
// drift, no epsilon comparisons.

std::size_t VehicleFactory::resolve_base_hz(const std::vector<std::size_t>& rates_hz) {
    std::size_t l = 1;
    for (std::size_t r : rates_hz) {
        if (r == 0) continue; // 0 = "not a discrete rate"; ignored
        l = l / std::gcd(l, r) * r; // LCM(a,b) = a*b/gcd(a,b); overflow-safe form
    }
    return l; // 1 if no positive rates (degenerate: base = 1 Hz)
}

std::map<std::size_t, std::size_t>
VehicleFactory::resolve_periods(const std::vector<std::size_t>& rates_hz, std::size_t window_hz,
                                double dt_max) {
    // Fold the macro-window rate into the LCM so the base tick is also a
    // multiple of the window rate. This guarantees the base >= window rate and
    // that the window is an integer number of base ticks.
    std::vector<std::size_t> all = rates_hz;
    if (window_hz > 0) all.push_back(window_hz);
    std::size_t base = resolve_base_hz(all);
    if (base == 0 || base == 1 && rates_hz.empty()) return {}; // no discrete subsystems
    // The base tick must not be finer than the integrator's max substep: a
    // base period smaller than dt_max would force the adaptive stepper to take
    // substeps smaller than it is allowed to, defeating the dt_max cap. This
    // is also what rejects absurdly fine rate combinations at build time.
    double base_period = 1.0 / static_cast<double>(base);
    if (base_period + 1e-12 < dt_max) {
        throw std::runtime_error("multi-rate: base tick " + std::to_string(base_period) +
                                 " s (base " + std::to_string(base) + " Hz) is finer than dt_max " +
                                 std::to_string(dt_max) + " s; choose related rates or raise dt_max");
    }
    std::map<std::size_t, std::size_t> out;
    for (std::size_t r : rates_hz) {
        if (r == 0) continue;
        if (base % r != 0) {
            // Cannot happen with LCM (base is a multiple of every input), but
            // guard against overflow producing a wrong LCM.
            throw std::runtime_error("multi-rate: rate " + std::to_string(r) +
                                     " Hz is not an integer divisor of the base " +
                                     std::to_string(base) + " Hz; rates must be integer-related");
        }
        out[r] = base / r; // integer by construction; >= 1
    }
    return out;
}

// ---------------------------------------------------------------------------
// Programmatic example (test oracle)
// ---------------------------------------------------------------------------
//
// This is the same vehicle the reference config config/vehicle.toml
// describes. It builds the identical topology by hand, so the two build
// paths (programmatic vs TOML) must produce equivalent systems.

BuiltSystem VehicleFactory::build_example_vehicle() {
    // Ensure the registry is populated (idempotent).
    if (model_types().empty()) {
        register_builtin_types();
    }
    // Build the equivalent spec in-memory and route it through the same
    // generic path — this guarantees the two paths cannot drift apart.
    VehicleSpec spec;
    spec.sim.t_end = 120.0;
    spec.sim.dt_macro = 0.01;
    spec.sim.dt_min = 1e-6;
    spec.sim.dt_max = 1e-3;
    spec.sim.eps_abs = 1e-9;
    spec.sim.eps_rel = 1e-9;
    spec.sim.telemetry_rate = 10.0;
    spec.sim.telemetry_file = "telemetry.csv";

    // Models.
    auto add = [&](const char* id, const char* type, const char* stage,
                   std::map<std::string, double> params = {}) {
        ModelSpec ms;
        ms.id = id;
        ms.type = type;
        ms.stage = stage;
        ms.params = std::move(params);
        spec.models.push_back(std::move(ms));
    };
    add("vehicle", "RigidBodyVehicle", "vehicle");
    add("fuel_tank", "PropellantTank", "stage1",
        {{"initial_mass", 22000.0}, {"volume", 22.0}, {"density", 1000.0}});
    add("oxi_tank", "PropellantTank", "stage1",
        {{"initial_mass", 22000.0}, {"volume", 20.0}, {"density", 1100.0}});
    add("engine1", "LiquidEngine", "stage1",
        {{"max_thrust", 1.0e6}, {"isp", 280.0}});
    add("gimbal_act", "FirstOrderActuator", "stage1", {{"tau", 0.05}});
    add("throttle_act", "FirstOrderActuator", "stage1", {{"tau", 0.1}, {"cmd0", 1.0}});
    add("p_sensor", "PressureSensor", "stage1");

    // Connections. Each names the producer signal "<id>.<port>" and lists
    // consumers. When the consumer's port has a DIFFERENT name than the
    // producer's port, set cs.to_port to the consumer's port name (this is
    // the port alias — e.g. the engine's force_z feeds the vehicle's
    // total_force_z accumulator).
    auto conn = [&](const char* signal, SignalKind kind, std::vector<std::string> producers,
                    std::vector<std::string> consumers, const char* to_port = nullptr) {
        ConnectionSpec cs;
        cs.signal = signal;
        cs.kind = kind;
        cs.producers = std::move(producers);
        cs.consumers = std::move(consumers);
        if (to_port) cs.to_port = to_port;
        spec.connections.push_back(std::move(cs));
    };
    // Engine -> vehicle force accumulators (force_* -> total_force_*).
    conn("engine1.force_x", SignalKind::Accumulator, {"engine1"}, {"vehicle"}, "total_force_x");
    conn("engine1.force_z", SignalKind::Accumulator, {"engine1"}, {"vehicle"}, "total_force_z");
    // Engine -> tank draw accumulators (fuel_draw/oxi_draw -> total_draw).
    conn("engine1.fuel_draw", SignalKind::Accumulator, {"engine1"}, {"fuel_tank"}, "total_draw");
    conn("engine1.oxi_draw", SignalKind::Accumulator, {"engine1"}, {"oxi_tank"}, "total_draw");
    // Tank mass -> vehicle total mass (total_mass_contrib -> total_mass).
    conn("fuel_tank.total_mass_contrib", SignalKind::Accumulator, {"fuel_tank"}, {"vehicle"}, "total_mass");
    conn("oxi_tank.total_mass_contrib", SignalKind::Accumulator, {"oxi_tank"}, {"vehicle"}, "total_mass");
    // Tank latched pressure -> engine (ullage_pressure_latched -> fuel/oxi_pressure).
    conn("fuel_tank.ullage_pressure_latched", SignalKind::Latched, {"fuel_tank"}, {"engine1"}, "fuel_pressure");
    conn("oxi_tank.ullage_pressure_latched", SignalKind::Latched, {"oxi_tank"}, {"engine1"}, "oxi_pressure");
    // Actuator position -> engine throttle (position -> throttle_pos).
    conn("throttle_act.position", SignalKind::Signal, {"throttle_act"}, {"engine1"}, "throttle_pos");
    // Tank pressure -> sensor source (ullage_pressure -> source).
    conn("fuel_tank.ullage_pressure", SignalKind::Signal, {"fuel_tank"}, {"p_sensor"}, "source");

    // Events.
    EventSpec ev;
    ev.id = "jettison_stage1";
    ev.type = "fixed_time";
    ev.stage = "stage1";
    ev.t = 60.0;
    spec.events.push_back(std::move(ev));

    return build_from_spec(spec);
}

} // namespace tdsim
