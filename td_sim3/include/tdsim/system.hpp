#pragma once
//
// system.hpp
//
// The odeint system functor. This is the single point where the three-pass
// evaluation pipeline runs. Per the architecture, the 3 passes are NOT in the
// outer driver loop — they live here, inside the functor, so they execute on
// every derivative call (including odeint's speculative trial steps).
//
// Design note on copyability: boost::numeric::odeint's integrate_adaptive
// takes the System BY VALUE, so the functor must be copyable. We therefore
// split it into:
//   - SimulationCore: owns the ModelSlots, the live ODE state (x_, dxdt_),
//     and the accessor tables. Non-copyable. Lives in the BuiltSystem.
//   - SystemFunctor: a thin, trivially-copyable wrapper holding a pointer to
//     the core. Every copy shares the same core, so odeint's internal copy
//     reads/writes the one live state. This is exactly what we want.
//
// Models are pure functions of (ODE spans, resolved upstream signals, latched
// values, time). They never mutate ODE state directly; only their assigned
// dxdt slice is written.
//

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "model.hpp"
#include "signal.hpp"
#include "topology.hpp"

namespace tdsim {

// ---------------------------------------------------------------------------
// SimulationCore — the non-copyable owner of all mutable simulation state.
// ---------------------------------------------------------------------------
class SimulationCore {
public:
    SimulationCore(std::vector<std::shared_ptr<ModelSlot>> slots,
                   std::vector<std::string> order, std::vector<double> x)
        : slots_(std::move(slots)), order_(std::move(order)), x_(std::move(x)), dxdt_(x_.size(), 0.0) {}

    SimulationCore(const SimulationCore&) = delete;
    SimulationCore& operator=(const SimulationCore&) = delete;

    // The 3-pass pipeline, executed on every derivative call.
    void compute_derivatives(const std::vector<double>& x, std::vector<double>& dxdt, double t) {
        x_ = x;
        std::fill(dxdt_.begin(), dxdt_.end(), 0.0);

        // Pass 1: clear accumulators.
        for (auto* acc : accumulators_) {
            acc->clear();
        }

        // Pass 2+3 (fused): evaluate in topological order. Each model reads
        // already-resolved upstream signals, latched values, and ODE spans,
        // then publishes its own signals and writes its dxdt slice.
        int evals = 0;
        for (const auto& model_id : order_) {
            auto it = slot_by_id_.find(model_id);
            if (it == slot_by_id_.end()) continue;
            ModelSlot* slot = it->second;
            if (!slot->active) continue; // jettisoned

            StateView sv{
                std::span<const double>(x_.data() + slot->state_offset, slot->state_dim),
                std::span<double>(dxdt_.data() + slot->state_offset, slot->state_dim)};
            SimContext ctx{t};
            slot->eval(ctx, sv);
            ++evals;
        }

        // Diagnostics: log the derivative-call count and flag non-finite
        // derivatives (the #1 cause of stepper rejections / dt collapse).
        ++derivative_calls_;
        bool nonfinite = false;
        for (double v : dxdt_) {
            if (!std::isfinite(v)) { nonfinite = true; break; }
        }
        if (nonfinite) {
            spdlog::error("[CORE] non-finite derivative at t={:.6f} (call #{}): dxdt={}",
                          t, derivative_calls_, print_vec(dxdt_));
        }
        if (derivative_calls_ % 1000 == 0) {
            spdlog::debug("[CORE] derivative call #{} at t={:.6f}, {} active models, "
                          "nonfinite={}", derivative_calls_, t, evals, nonfinite);
        }

        dxdt = dxdt_;
    }

    // For diagnostics: a compact one-line dump of a vector.
    static std::string print_vec(const std::vector<double>& v) {
        std::string s = "[";
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (i) s += ", ";
            s += std::to_string(v[i]);
        }
        s += "]";
        return s;
    }

    // Number of derivative calls (exposed for tests / diagnostics).
    std::size_t derivative_call_count() const { return derivative_calls_; }

    // Accessors.
    std::vector<double>& state() { return x_; }
    const std::vector<double>& state() const { return x_; }
    void set_state(const std::vector<double>& x) { x_ = x; }
    const std::vector<std::shared_ptr<ModelSlot>>& slots() const { return slots_; }

    // Pass-1 accumulator registration.
    void add_accumulator(Accumulator<double>& a) { accumulators_.push_back(&a); }

    // Map a model id to its slot.
    void register_slot(const std::string& id, ModelSlot* slot) { slot_by_id_[id] = slot; }

    // Accessor tables: signal name -> read / propose. The factory populates
    // these so the driver reads sensor signals and writes command latches by
    // name without knowing concrete model types. Signals registered via
    // set_reader are also the telemetry column set (the driver logs every
    // registered reader's name + value).
    void set_reader(const std::string& name, std::function<double()> r) { readers_[name] = std::move(r); }
    const std::unordered_map<std::string, std::function<double()>>& reader_names() const { return readers_; }
    void set_proposer(const std::string& name, std::function<void(double)> p) { proposers_[name] = std::move(p); }

    // Update the evaluation order (after a jettison re-sort).
    void set_evaluation_order(std::vector<std::string> order) { order_ = std::move(order); }

    double read_signal(const std::string& name) const {
        auto it = readers_.find(name);
        if (it == readers_.end()) throw std::runtime_error("No reader for signal '" + name + "'");
        return it->second();
    }
    void propose_signal(const std::string& name, double v) {
        auto it = proposers_.find(name);
        if (it == proposers_.end()) throw std::runtime_error("No proposer for signal '" + name + "'");
        it->second(v);
    }

private:
    std::vector<std::shared_ptr<ModelSlot>> slots_;
    std::vector<std::string> order_; // evaluation order (copied, jettison-updatable)
    std::vector<double> x_;
    std::vector<double> dxdt_;
    std::vector<Accumulator<double>*> accumulators_;
    std::unordered_map<std::string, ModelSlot*> slot_by_id_;
    std::unordered_map<std::string, std::function<double()>> readers_;
    std::unordered_map<std::string, std::function<void(double)>> proposers_;
    std::size_t derivative_calls_ = 0;
};

// ---------------------------------------------------------------------------
// SystemFunctor — copyable odeint system. All copies share the same core.
// ---------------------------------------------------------------------------
struct SystemFunctor {
    SystemFunctor() = default;
    explicit SystemFunctor(SimulationCore& core) : core_(&core) {}

    void operator()(const std::vector<double>& x, std::vector<double>& dxdt, double t) const {
        core_->compute_derivatives(x, dxdt, t);
    }

    // Convenience pass-throughs used by the driver.
    std::vector<double>& state() const { return core_->state(); }
    void set_state(const std::vector<double>& x) const { core_->set_state(x); }
    const std::vector<std::shared_ptr<ModelSlot>>& slots() const { return core_->slots(); }
    double read_signal(const std::string& name) const { return core_->read_signal(name); }
    void propose_signal(const std::string& name, double v) const { core_->propose_signal(name, v); }

    SimulationCore* core_ = nullptr;
};

} // namespace tdsim
