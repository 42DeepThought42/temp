#pragma once
// td_sim3 — topology.hpp
//
// The SimulationBuilder parses model + connection declarations (normally from
// the config parser), validates the signal contract, topologically sorts the
// producer→consumer graph, and emits the startup Engineering Data Flow &
// Topology Report. Any contract violation terminates before integration.
//
// Contract enforced (see SESSION_NOTES.md §3.2):
//   Signal<T>       exactly 1 producer (else hard error); DAG edge to each
//                   consumer; 0 producers also an error (declared but unwired).
//   Accumulator<T>  0+ producers (0 = warning, likely a config typo); edge from
//                   EACH producer to each consumer (consumer runs after all);
//                   producers are order-free among themselves.
//   Latched<T>      exactly 1 producer (else hard error); NO edge (this is the
//                   sanctioned algebraic-loop breaker).
//
// Jettison re-sort (cheap): remove Signal edges whose producer or consumer is
// inactivated; drop inactivated producers from Accumulators (no re-sort);
// Latched signals need no action.

#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "signal.hpp"

namespace tdsim {

struct Connection {
    std::string name;
    SignalKind kind = SignalKind::Signal;
    std::vector<std::string> producers; // exactly 1 for Signal/Latched, 0+ for Accumulator
    std::vector<std::string> consumers;
};

// An ODE-state reader wiring: which model's ODE slice a consumer reads.
struct OdeWire {
    std::string producer;
    std::string consumer;
};

// One entry in the startup report: a model and its resolved data flow.
struct ModelReport {
    std::string id;
    int pass2_rank = 0; // 1-based position in evaluation order
    std::vector<std::pair<std::string, std::string>> reads;    // (signal, producer)
    std::vector<std::pair<std::string, SignalKind>> writes;    // (signal, kind)
};

class SimulationBuilder {
public:
    // Register a model instance by id (and the stage it belongs to; empty for
    // vehicle-level models). Must be called before any wiring.
    void register_model(const std::string& id, const std::string& stage_id = "");

    // Wire a consumer to a producer for a named signal. The kind determines
    // the validation rules. Repeated calls for the same (signal, producer,
    // consumer) triple are deduplicated.
    void wire(const std::string& signal_name, SignalKind kind,
              const std::string& producer, const std::string& consumer);

    // Wire a model to read another model's ODE state slice. Creates a DAG edge
    // (producer must be integrated before consumer reads it — though ODE state
    // is read from the global vector, not computed in Pass 2, so this edge is
    // for determinism/reporting only). Returns the reader name for binding.
    void wire_ode(const std::string& reader_name, const std::string& producer,
                  const std::string& consumer);

    // Validate + sort. Returns the 0-based evaluation order (model ids).
    // Throws std::runtime_error on any hard contract violation.
    // Emits the Engineering Data Flow & Topology Report to stdout.
    std::vector<std::string> resolve_topology();

    // Jettison support: mark a stage (and all its models) inactive, then
    // re-resolve the surviving subgraph. Cheap per §3.2.
    void inactivate_stage(const std::string& stage_id);
    std::vector<std::string> resolve_topology_after_jettison();

    // Accessors for the system functor / driver.
    const std::unordered_map<std::string, Connection>& connections() const { return registry_; }
    const std::vector<std::string>& evaluation_order() const { return order_; }
    const std::unordered_map<std::string, bool>& active_models() const { return active_; }
    const std::vector<ModelReport>& report() const { return report_; }
    const std::unordered_map<std::string, OdeWire>& ode_wires() const { return ode_wires_; }

private:
    void build_edges();
    std::vector<std::string> kahn_sort() const;
    void emit_report() const;
    std::string kind_tag(SignalKind k) const;

    std::unordered_map<std::string, std::string> model_stage_; // id -> stage_id
    std::unordered_map<std::string, bool> active_;             // id -> active
    std::unordered_map<std::string, Connection> registry_;
    std::unordered_map<std::string, std::unordered_set<std::string>> edges_; // dep -> dependents (deduped)
    std::unordered_map<std::string, OdeWire> ode_wires_;
    std::vector<std::string> order_;
    mutable std::vector<ModelReport> report_; // rebuilt on each emit_report()
};

} // namespace tdsim
