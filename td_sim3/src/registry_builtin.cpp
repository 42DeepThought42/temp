// td_sim3 — registry_builtin.cpp
//
// Registration of the built-in reference model types (rigid-body vehicle,
// propellant tank, liquid engine, first-order actuator, pressure sensor).
// Split out of factory.cpp so the generic build pipeline (factory.cpp) and
// the per-type registration blocks (this file) live in separate TUs. This
// file is where you add a new model type's register_<type>() block and
// call it from register_builtin_types() (see DEVELOPERS_GUIDE §4).

#include "tdsim/factory.hpp"
#include "tdsim/models.hpp" // built-in reference model types

#include <memory>

namespace tdsim {

// ---------------------------------------------------------------------------
// Built-in model types (reference set for this template)
// ---------------------------------------------------------------------------
//
// Pattern per type M (keep this small when adding new models):
//   ModelType mt;
//   mt.state_dim = M::DIM;
//   mt.construct = []() { /* make_unique<M> + ConcreteSlot<DIM, M> */ };
//   mt.ports = { {"port", kind, produced, bind_lambda}, ... };
//   mt.params = { {"name", setter_lambda}, ... };
//   + optional hooks: initial_state, post_configure, accumulators, readers,
//     proposers, collect_latches.
//
// NOTE on downcasts: every lambda is written for exactly one concrete type M,
// so static_cast<M*>(static_cast<void*>(b.model())) is safe by construction. The
// StatefulModel<0> base is a lifetime handle only.
//
// Lifetime: the ConcreteSlot is the SOLE owner of the concrete model
// (unique_ptr<M> inside it; StatefulModel<Dim> for different Dim are
// unrelated types, so there is no common pointer type — the slot erases it).
// The concrete model's StateDescriptor lives as a member of the model, so it
// is owned by the same unique_ptr. The construct lambda must therefore NOT do
//   auto m = std::make_unique<M>();
//   return std::make_unique<ConcreteSlot<...>>(std::move(m), &m->desc);
// — m would be destroyed when the lambda returns and the slot would dangle.
// The correct pattern (used by every register_* below):
//   auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
//   slot->m_model = std::make_unique<M>();
//   slot->desc = &slot->m_model->desc;   // stable: desc lives in the model
//   return slot;

namespace {

void register_vehicle() {
    using M = RigidBodyVehicle;
    ModelType mt;
    mt.state_dim = M::DIM;
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc = &slot->m_model->desc;
        return slot;
    };
    mt.ports = {
        {"total_force_x", {"force_x"}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->total_force_x).bind(cur, "");
         }},
        {"total_force_y", {"force_y"}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->total_force_y).bind(cur, "");
         }},
        {"total_force_z", {"force_z"}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->total_force_z).bind(cur, "");
         }},
        {"total_mass", {"mass", "total_mass_contrib"}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->total_mass).bind(cur, "");
         }},
        {"accel_x", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->accel_x).bind(cur, "");
         }},
        {"accel_y", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->accel_y).bind(cur, "");
         }},
        {"accel_z", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             (m->accel_z).bind(cur, "");
         }},
    };
    mt.params = {
        {"gravity", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model()));
            m->gravity = v;
        }},
    };
    mt.accumulators = [](ModelSlot& b, std::vector<Accumulator<double>*>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(&m->total_force_x);
        out.push_back(&m->total_force_y);
        out.push_back(&m->total_force_z);
        out.push_back(&m->total_mass);
    };
    mt.readers = [](ModelSlot& b,
                    const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("accel_x", [m]() { return m->accel_x.read(); });
        add("accel_y", [m]() { return m->accel_y.read(); });
        add("accel_z", [m]() { return m->accel_z.read(); });
    };
    VehicleFactory::register_model_type("RigidBodyVehicle", std::move(mt));
}

void register_tank() {
    using M = PropellantTank;
    ModelType mt;
    mt.state_dim = M::DIM;
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc = &slot->m_model->desc;
        return slot;
    };
    mt.ports = {
        {"mass", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->mass_out.bind(cur, "");
         }},
        {"ullage_pressure", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->ullage_pressure.bind(cur, "");
         }},
        {"ullage_pressure_latched", {}, SignalKind::Latched, true,
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->ullage_pressure_latched.bind(cur, nxt, "");
         }},
        {"total_draw", {"draw", "fuel_draw", "oxi_draw"}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->total_draw.bind(cur, "");
         }},
        {"total_mass_contrib", {"mass_contrib"}, SignalKind::Accumulator, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->total_mass_contrib.bind(cur, "");
         }},
    };
    mt.params = {
        {"volume", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->volume = v;
        }},
        {"density", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->density = v;
        }},
        {"initial_mass", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->initial_mass = v;
        }},
    };
    mt.initial_state = [](ModelSlot& b) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        return std::vector<double>{m->initial_mass};
    };
    // Latch init: derived from the initial fill fraction (runs AFTER params +
    // port binding, per the factory's post_configure call site).
    mt.post_configure = [](ModelSlot& b) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        double p0 = (m->volume > 1e-9)
                        ? std::clamp(m->initial_mass / (m->density * m->volume), 0.0, 1.0) * 1e5
                        : 0.0;
        m->ullage_pressure_latched.init(p0);
    };
    mt.accumulators = [](ModelSlot& b, std::vector<Accumulator<double>*>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(&m->total_draw);
        out.push_back(&m->total_mass_contrib);
    };
    mt.readers = [](ModelSlot& b,
                    const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        // Only log the algebraic ullage_pressure; the tank's mass is an ODE
        // state and already appears as an ODE telemetry column ("<id>.mass"),
        // so logging it again as a signal would duplicate the column.
        add("ullage_pressure", [m]() { return m->ullage_pressure.read(); });
    };
    mt.collect_latches = [](ModelSlot& b, std::vector<LatchRef>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(LatchRef{&m->ullage_pressure_latched, 1});
    };
    VehicleFactory::register_model_type("PropellantTank", std::move(mt));
}

void register_engine() {
    using M = LiquidEngine;
    ModelType mt;
    mt.state_dim = M::DIM;
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc = &slot->m_model->desc;
        return slot;
    };
    mt.ports = {
        {"fuel_pressure", {}, SignalKind::Latched, false,
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->fuel_pressure.bind(cur, nxt, "");
         }},
        {"oxi_pressure", {}, SignalKind::Latched, false,
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->oxi_pressure.bind(cur, nxt, "");
         }},
        {"throttle_pos", {"position"}, SignalKind::Signal, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->throttle_pos.bind(cur, "");
         }},
        {"thrust", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->thrust.bind(cur, "");
         }},
        {"force_x", {}, SignalKind::Accumulator, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->force_x.bind(cur, "");
         }},
        {"force_z", {}, SignalKind::Accumulator, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->force_z.bind(cur, "");
         }},
        {"fuel_draw", {}, SignalKind::Accumulator, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->fuel_draw.bind(cur, "");
         }},
        {"oxi_draw", {}, SignalKind::Accumulator, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->oxi_draw.bind(cur, "");
         }},
    };
    mt.params = {
        {"isp", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->isp = v;
        }},
        {"max_thrust", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->max_thrust = v;
        }},
        {"g0", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->g0 = v;
        }},
    };
    mt.accumulators = [](ModelSlot& b, std::vector<Accumulator<double>*>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(&m->force_x);
        out.push_back(&m->force_z);
        out.push_back(&m->fuel_draw);
        out.push_back(&m->oxi_draw);
    };
    mt.readers = [](ModelSlot& b,
                    const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("thrust", [m]() { return m->thrust.read(); });
    };
    VehicleFactory::register_model_type("LiquidEngine", std::move(mt));
}

void register_actuator() {
    using M = FirstOrderActuator;
    ModelType mt;
    mt.state_dim = M::DIM;
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc = &slot->m_model->desc;
        return slot;
    };
    mt.ports = {
        {"cmd", {}, SignalKind::Latched, false,
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->cmd.bind(cur, nxt, "");
         }},
        {"position", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->position.bind(cur, "");
         }},
    };
    mt.params = {
        {"tau", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->tau = v;
        }},
        {"x0", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->x0 = v;
        }},
        {"cmd0", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->cmd0 = v;
        }},
    };
    mt.initial_state = [](ModelSlot& b) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        return std::vector<double>{m->x0};
    };
    mt.post_configure = [](ModelSlot& b) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        m->cmd.init(m->cmd0);
    };
    mt.readers = [](ModelSlot& b,
                    const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("position", [m]() { return m->position.read(); });
    };
    mt.proposers = [](ModelSlot& b,
                      const std::function<void(const std::string&, std::function<void(double)>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("cmd", [m](double v) { m->cmd.propose(v); });
    };
    mt.collect_latches = [](ModelSlot& b, std::vector<LatchRef>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(LatchRef{&m->cmd, 1});
    };
    VehicleFactory::register_model_type("FirstOrderActuator", std::move(mt));
}

void register_sensor() {
    using M = PressureSensor;
    ModelType mt;
    mt.state_dim = M::DIM;
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc = &slot->m_model->desc;
        return slot;
    };
    mt.ports = {
        {"source", {"ullage_pressure"}, SignalKind::Signal, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->source.bind(cur, "");
         }},
        // The sensor's output is a Latched: a zero-order hold at the sensor's
        // sample rate. Consumers read the last committed (sampled) value.
        {"reading", {}, SignalKind::Latched, true,
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->reading.bind(cur, nxt, "");
         }},
    };
    mt.params = {
        {"offset", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->offset = v;
        }},
        {"rate", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->rate = v;
        }},
    };
    // Latch init: the sensor's first committed reading is the initial source
    // value (recomputed from the live source at t=0 by the factory's
    // post_configure, which runs after params + port binding).
    mt.post_configure = [](ModelSlot& b) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        m->reading.init(m->source.read() + m->offset);
    };
    mt.readers = [](ModelSlot& b,
                    const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("reading", [m]() { return m->reading.read(); });
    };
    mt.collect_latches = [](ModelSlot& b, std::vector<LatchRef>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        // period_ticks is filled in by the factory's base-tick resolver from
        // m->rate (Hz); default 1 = commit every base tick (continuous).
        out.push_back(LatchRef{&m->reading, 1});
    };
    VehicleFactory::register_model_type("PressureSensor", std::move(mt));
}

} // namespace

void VehicleFactory::register_builtin_types() {
    register_vehicle();
    register_tank();
    register_engine();
    register_actuator();
    register_sensor();
}

} // namespace tdsim
