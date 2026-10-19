#include "tdsim/topology.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

namespace tdsim {

namespace {
std::string join(const std::vector<std::string>& v) {
    std::string s;
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i) s += ", ";
        s += v[i];
    }
    return s;
}
} // namespace

void SimulationBuilder::register_model(const std::string& id, const std::string& stage_id) {
    if (model_stage_.count(id)) {
        throw std::runtime_error("Builder Error: duplicate model id '" + id + "'");
    }
    model_stage_[id] = stage_id;
    active_[id] = true;
    edges_[id] = {};
}

void SimulationBuilder::wire(const std::string& signal_name, SignalKind kind,
                             const std::string& producer, const std::string& consumer) {
    // Validate that both endpoints are registered models.
    if (!model_stage_.count(producer)) {
        throw std::runtime_error("Builder Error: signal '" + signal_name +
                                 "' references unknown producer model '" + producer + "'");
    }
    if (!model_stage_.count(consumer)) {
        throw std::runtime_error("Builder Error: signal '" + signal_name +
                                 "' references unknown consumer model '" + consumer + "'");
    }

    auto& conn = registry_[signal_name];
    conn.name = signal_name;
    if (conn.producers.empty() && conn.consumers.empty()) {
        conn.kind = kind; // first wire for this signal sets its kind
    } else if (conn.kind != kind) {
        throw std::runtime_error("Builder Error: signal '" + signal_name +
                                 "' kind mismatch (declared " + kind_tag(conn.kind) +
                                 ", wired as " + kind_tag(kind) + ")");
    }

    // Deduplicate producers and consumers.
    if (std::find(conn.producers.begin(), conn.producers.end(), producer) == conn.producers.end()) {
        conn.producers.push_back(producer);
    }
    if (std::find(conn.consumers.begin(), conn.consumers.end(), consumer) == conn.consumers.end()) {
        conn.consumers.push_back(consumer);
    }
}

void SimulationBuilder::wire_ode(const std::string& reader_name, const std::string& producer,
                                 const std::string& consumer) {
    if (!model_stage_.count(producer)) {
        throw std::runtime_error("Builder Error: ODE reader '" + reader_name +
                                 "' references unknown producer model '" + producer + "'");
    }
    if (!model_stage_.count(consumer)) {
        throw std::runtime_error("Builder Error: ODE reader '" + reader_name +
                                 "' references unknown consumer model '" + consumer + "'");
    }
    if (ode_wires_.count(reader_name)) {
        throw std::runtime_error("Builder Error: duplicate ODE reader name '" + reader_name + "'");
    }
    ode_wires_[reader_name] = OdeWire{producer, consumer};
    // Add a determinism edge: consumer runs after producer (reporting clarity).
    if (producer != consumer) {
        edges_[producer].insert(consumer);
    }
}

void SimulationBuilder::build_edges() {
    // Reset edges, then re-add both signal/accumulator edges and ODE edges.
    for (auto& [id, e] : edges_) {
        e.clear();
    }
    // Signal: edge producer -> each consumer.
    // Accumulator: edge each producer -> each consumer.
    // Latched: no edge.
    for (const auto& [name, conn] : registry_) {
        if (conn.kind == SignalKind::Latched) {
            continue;
        }
        for (const auto& producer : conn.producers) {
            for (const auto& consumer : conn.consumers) {
                if (producer == consumer) continue; // self-loop is a no-op, not an error
                edges_[producer].insert(consumer);
            }
        }
    }
    // ODE-state readers: edge producer -> consumer (determinism/reporting).
    for (const auto& [name, w] : ode_wires_) {
        if (w.producer != w.consumer) {
            edges_[w.producer].insert(w.consumer);
        }
    }
}

std::vector<std::string> SimulationBuilder::kahn_sort() const {
    // In-degree = number of active dependencies (producers that must run first).
    std::unordered_map<std::string, int> in_degree;
    for (const auto& [id, act] : active_) {
        if (act) in_degree[id] = 0;
    }
    // Only count edges between active models.
    std::unordered_map<std::string, std::vector<std::string>> adj;
    for (const auto& [id, act] : active_) {
        if (act) adj[id] = {};
    }
    for (const auto& [producer, consumers] : edges_) {
        auto pit = active_.find(producer);
        if (pit == active_.end() || !pit->second) continue;
        for (const auto& consumer : consumers) {
            auto cit = active_.find(consumer);
            if (cit == active_.end() || !cit->second) continue;
            adj[producer].push_back(consumer);
            in_degree[consumer]++;
        }
    }

    // Use a stable (deterministic) ordering: process in registration order among
    // zero-in-degree nodes for reproducible reports.
    std::vector<std::string> zero;
    for (const auto& [id, deg] : in_degree) {
        if (deg == 0) zero.push_back(id);
    }
    // Sort zero list by registration order (model_stage_ insertion order is
    // unordered; we sort by id for determinism).
    std::sort(zero.begin(), zero.end());

    std::vector<std::string> order;
    while (!zero.empty()) {
        std::string curr = zero.front();
        zero.erase(zero.begin());
        order.push_back(curr);
        for (const auto& nb : adj[curr]) {
            in_degree[nb]--;
            if (in_degree[nb] == 0) {
                zero.push_back(nb);
                std::sort(zero.begin(), zero.end()); // keep deterministic
            }
        }
    }

    // If any active model was not reached, there's a cycle among active models.
    size_t active_count = 0;
    for (const auto& [id, act] : active_) {
        if (act) active_count++;
    }
    if (order.size() != active_count) {
        // Identify the models in the cycle for a useful error message.
        std::vector<std::string> stuck;
        for (const auto& [id, act] : active_) {
            if (act && std::find(order.begin(), order.end(), id) == order.end()) {
                stuck.push_back(id);
            }
        }
        std::sort(stuck.begin(), stuck.end());
        throw std::runtime_error("Topology Fatal: algebraic loop detected among models: [" +
                                 join(stuck) +
                                 "]. Break the loop with an explicit Latched<T> signal.");
    }
    return order;
}

std::vector<std::string> SimulationBuilder::resolve_topology() {
    build_edges();

    // Validate producer counts.
    for (const auto& [name, conn] : registry_) {
        if (conn.kind == SignalKind::Signal) {
            if (conn.producers.empty()) {
                throw std::runtime_error("Topology Error: Signal '" + name +
                                         "' has no producer (declared but never wired).");
            }
            if (conn.producers.size() > 1) {
                throw std::runtime_error("Topology Error: Signal '" + name +
                                         "' has multiple producers: [" + join(conn.producers) +
                                         "]. Signal<T> allows exactly one.");
            }
        } else if (conn.kind == SignalKind::Accumulator) {
            if (conn.producers.empty()) {
                std::cout << "[WARNING] Accumulator '" << name
                          << "' has 0 producers. Likely a config typo; consumers will read 0.\n";
            }
        } else if (conn.kind == SignalKind::Latched) {
            if (conn.producers.empty()) {
                throw std::runtime_error("Topology Error: Latched signal '" + name +
                                         "' has no producer.");
            }
            if (conn.producers.size() > 1) {
                throw std::runtime_error("Topology Error: Latched signal '" + name +
                                         "' has multiple producers: [" + join(conn.producers) +
                                         "]. Latched<T> allows exactly one.");
            }
        }
    }

    order_ = kahn_sort();
    emit_report();
    return order_;
}

void SimulationBuilder::inactivate_stage(const std::string& stage_id) {
    for (auto& [id, stage] : model_stage_) {
        if (stage == stage_id) {
            active_[id] = false;
        }
    }
}

std::vector<std::string> SimulationBuilder::resolve_topology_after_jettison() {
    // Rebuild edges over the active subgraph only (build_edges + kahn_sort both
    // respect active_). No per-signal bookkeeping needed: Signal edges to
    // inactivated endpoints are simply not added, Accumulator producers that are
    // inactive contribute no edge, Latched is unaffected.
    build_edges();
    order_ = kahn_sort();
    emit_report();
    return order_;
}

std::string SimulationBuilder::kind_tag(SignalKind k) const {
    switch (k) {
        case SignalKind::Signal: return "Signal";
        case SignalKind::Accumulator: return "Accumulator";
        case SignalKind::Latched: return "Latched";
    }
    return "?";
}

void SimulationBuilder::emit_report() const {
    std::cout << "\n======================================================================\n";
    std::cout << "        ENGINEERING DATA FLOW & TOPOLOGY REGISTERED REPORT        \n";
    std::cout << "======================================================================\n";

    // Build a quick lookup: model id -> report entry index.
    std::unordered_map<std::string, int> idx;
    report_.clear();
    report_.reserve(order_.size());
    for (int i = 0; i < static_cast<int>(order_.size()); i++) {
        report_.push_back(ModelReport{order_[i], i + 1, {}, {}});
        idx[order_[i]] = i;
    }

    // Populate reads/writes from the registry.
    for (const auto& [name, conn] : registry_) {
        for (const auto& consumer : conn.consumers) {
            auto it = idx.find(consumer);
            if (it == idx.end()) continue; // inactivated
            for (const auto& producer : conn.producers) {
                report_[it->second].reads.emplace_back(name, producer);
            }
        }
        for (const auto& producer : conn.producers) {
            auto it = idx.find(producer);
            if (it == idx.end()) continue;
            report_[it->second].writes.emplace_back(name, conn.kind);
        }
    }

    for (const auto& r : report_) {
        std::cout << "Pass 2 Order [" << r.pass2_rank << "]: Model [" << r.id << "]\n";
        std::cout << "  -> READS (Consumed):\n";
        if (r.reads.empty()) std::cout << "     (none)\n";
        for (const auto& [sig, prod] : r.reads) {
            std::cout << "     * " << sig << " (From: " << prod << ")\n";
        }
        std::cout << "  <- WRITES (Produced):\n";
        if (r.writes.empty()) std::cout << "     (none)\n";
        for (const auto& [sig, kind] : r.writes) {
            std::cout << "     * " << sig << " [" << kind_tag(kind) << "]\n";
        }
        std::cout << "----------------------------------------------------------------------\n";
    }
    std::cout << "======================================================================\n\n";
}

} // namespace tdsim
