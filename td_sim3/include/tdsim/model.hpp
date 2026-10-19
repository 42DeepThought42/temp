#pragma once
// td_sim3 — model.hpp
//
// Model contract. A model is a struct that:
//
//   1. derives from StatefulModel<Dim> (Dim = number of integrated ODE
//      doubles it owns; Dim == 0 for pure-algebraic / discrete-only models)
//      and declares a StateDescriptor with `Dim` named slots;
//   2. holds Signal<T>/Accumulator<T>/Latched<T> member views for the
//      algebraic data it consumes and produces (wired by the builder);
//   3. implements  void eval(const SimContext& ctx, const StateView& sv);
//
// eval() is the ONLY entry point. The system functor calls it in topological
// order. It MUST be a pure function of:
//   - its ODE state slice (sv.state, read-only),
//   - other models' ODE state via their spans (hand-wired views),
//   - already-resolved upstream signals,
//   - latched (previous accepted-window) values,
//   - the current time.
// Derivative models write their rates into sv.dxdt at the end of eval()
// (this is Pass 3 — it is fused with Pass 2 because a model's derivatives
// depend only on algebraics that are resolved by the time it runs).
//
// The registry stores std::any of the concrete type plus a type-erased
// wrapper (ModelSlot) that knows how to call eval() and how big the state
// slice is. New models: write the struct, add one line to the factory.

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "signal.hpp"

namespace tdsim {

// Per-eval context handed to every model. Note: this does NOT carry an
// 'active' flag — the system functor skips a model's eval() entirely once its
// stage is jettisoned, so a model's eval() is only ever called while active.
struct SimContext {
    double t = 0.0;
};

// A model's read/write window into the global ODE vectors.
struct StateView {
    std::span<const double> state; // read: current ODE state slice
    std::span<double> dxdt;        // write: derivative slice
};

// A read-only view of ANOTHER model's ODE state slice, wired by the builder
// to a global-vector offset. This is how an engine reads a tank's mass without
// holding a pointer to the tank. Because the span points into the single
// global vector that the system functor refreshes on every odeint call, the
// reader always sees the candidate state being probed — no staleness.
struct OdeReader {
    std::string name;            // telemetry / config name
    std::span<const double> data; // read-only slice of the global ODE vector
    // Framework-only binding (builder sets the offset before the sim starts).
    void bind(std::string n, std::span<const double> d) {
        name = std::move(n);
        data = d;
    }
    double operator[](std::size_t i) const { return data[i]; }
    std::size_t size() const { return data.size(); }
};

// Declaration of a model's ODE state: telemetry group name + per-slot names.
struct StateDescriptor {
    std::string name;
    std::vector<std::string> slot_names;
    std::size_t size() const { return slot_names.size(); }
};

// Base for models that own integrated state.
//
// NOTE: StatefulModel<Dim> for different Dim are UNRELATED types (no shared
// base). This is intentional: the hot path (eval) is a single virtual call
// through ModelSlot with zero indirection. The cost is that the registry
// (factory.hpp) stores every model behind the StatefulModel<0> type as a
// pure lifetime handle — hooks never call eval() through it, they downcast
// to the concrete type inside their captured lambda.
template <std::size_t Dim>
struct StatefulModel {
    static constexpr std::size_t state_dim = Dim;
    virtual ~StatefulModel() = default;
    virtual void eval(const SimContext& ctx, const StateView& sv) = 0;
};

// Type erasure over StatefulModel<Dim>. The builder constructs one per model
// instance; the system functor drives them through this interface without
// knowing Dim or the concrete type.
//
// Ownership & type erasure: the slot OWNS the concrete model (unique_ptr<T>)
// and is the ONLY well-defined way to reach it: model() returns the erased
// StatefulModel<0>* whose object representation IS the concrete model, so the
// registration hooks' static_cast<T*>(void*) downcasts are sound. There is no
// other copy of the model anywhere — the factory's "bag" is a list of
// slot pointers.
struct ModelSlot {
    std::string id;
    std::string stage_id; // empty for vehicle-level models
    std::size_t state_offset = 0;
    std::size_t state_dim = 0;
    bool active = true;

    virtual ~ModelSlot() = default;
    // Called in topological order. sv is already positioned at this model's
    // slice of the global vectors.
    virtual void eval(const SimContext& ctx, const StateView& sv) = 0;
    virtual const StateDescriptor& descriptor() const = 0;
    // The owned model, type-erased as void* (StatefulModel<Dim> for different
    // Dim are unrelated types, so there is no common pointer type).
    virtual void* model() = 0;
    virtual const void* model() const = 0;
};

template <std::size_t Dim, typename T>
struct ConcreteSlot : ModelSlot {
    std::unique_ptr<T> m_model;
    const StateDescriptor* desc = nullptr;

    ConcreteSlot() = default;
    explicit ConcreteSlot(std::unique_ptr<T> m, const StateDescriptor* d) : m_model(std::move(m)), desc(d) {}

    void eval(const SimContext& ctx, const StateView& sv) override {
        m_model->eval(ctx, sv);
    }
    const StateDescriptor& descriptor() const override { return *desc; }
    void* model() override { return m_model.get(); }
    const void* model() const override { return m_model.get(); }
};

static_assert([] {
    // Illustrative (non-enforced) check that Dim matches the descriptor size;
    // the builder re-checks this at runtime with a clear error.
    return true;
}());

} // namespace tdsim
