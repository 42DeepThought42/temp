// td_sim3 — integration tests for the registry-driven build pipeline
// (factory.cpp): spec validation, the multi-producer Accumulator binding
// (regression for the orphaned-producer bug, SESSION_NOTES §9a.4), and the
// reference-vehicle oracle (TOML path vs programmatic path must not drift).

#include <doctest/doctest.h>

#include "tdsim/factory.hpp"
#include "tdsim/models.hpp"
#include "tdsim/simulator.hpp"
#include "tdsim/spec.hpp"

#include "registry_setup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace tdsim;

namespace {

SimParams test_params() {
    SimParams p;
    p.t_end = 1.0;
    p.dt_macro = 0.01;
    p.telemetry_rate = 0.0; // disabled: tests never write CSV files
    p.telemetry_file = "";
    return p;
}

ModelSpec model(const std::string& id, const std::string& type,
                std::map<std::string, double> params = {},
                const std::string& stage = "") {
    ModelSpec ms;
    ms.id = id;
    ms.type = type;
    ms.stage = stage;
    ms.params = std::move(params);
    return ms;
}

ConnectionSpec conn(const std::string& signal, SignalKind kind,
                    std::vector<std::string> producers, std::vector<std::string> consumers,
                    const std::string& to_port = "") {
    ConnectionSpec cs;
    cs.signal = signal;
    cs.kind = kind;
    cs.producers = std::move(producers);
    cs.consumers = std::move(consumers);
    cs.to_port = to_port;
    return cs;
}

// Run one derivative evaluation of a built system and return a copy of dxdt.
std::vector<double> one_eval(BuiltSystem& bs, const std::vector<double>& x, double t = 0.0) {
    std::vector<double> dxdt(x.size(), 0.0);
    bs.sys(x, dxdt, t);
    return dxdt;
}

std::size_t index_of(const std::vector<std::string>& v, const std::string& s) {
    return static_cast<std::size_t>(std::find(v.begin(), v.end(), s) - v.begin());
}

// Global ODE offset of a model's state slice (for indexing dxdt).
std::size_t state_offset_of(const std::string& id, BuiltSystem& bs) {
    auto* s = find_slot(id, bs.core->slots());
    REQUIRE(s != nullptr);
    return s->state_offset;
}

} // namespace

TEST_CASE("build_from_spec: unknown model type is rejected") {
    ensure_registry();
    VehicleSpec spec;
    spec.sim = test_params();
    spec.models.push_back(model("mystery", "NoSuchType"));
    CHECK_THROWS_AS(VehicleFactory::build_from_spec(spec), std::runtime_error);
}

TEST_CASE("build_from_spec: unknown model parameter is rejected") {
    ensure_registry();
    VehicleSpec spec;
    spec.sim = test_params();
    spec.models.push_back(model("t", "PropellantTank", {{"bogus_key", 1.0}}));
    bool threw = false;
    std::string msg;
    try {
        VehicleFactory::build_from_spec(spec);
    } catch (const std::runtime_error& e) {
        threw = true;
        msg = e.what();
    }
    CHECK(threw);
    CHECK(msg.find("unknown parameter") != std::string::npos);
    CHECK(msg.find("bogus_key") != std::string::npos);
}

TEST_CASE("build_from_spec: connection to a nonexistent port is rejected") {
    ensure_registry();
    VehicleSpec spec;
    spec.sim = test_params();
    spec.models.push_back(model("t", "PropellantTank", {{"initial_mass", 1.0}}));
    spec.connections.push_back(conn("t.nonexistent_port", SignalKind::Signal, {"t"}, {"t"}));
    CHECK_THROWS_AS(VehicleFactory::build_from_spec(spec), std::runtime_error);
}

TEST_CASE("build_from_spec: connection kind mismatch is rejected") {
    ensure_registry();
    VehicleSpec spec;
    spec.sim = test_params();
    spec.models.push_back(model("t", "PropellantTank", {{"initial_mass", 1.0}}));
    // ullage_pressure is a Signal port; wiring it as a Latched is an error.
    spec.connections.push_back(conn("t.ullage_pressure", SignalKind::Latched, {"t"}, {"t"}));
    CHECK_THROWS_AS(VehicleFactory::build_from_spec(spec), std::runtime_error);
}

TEST_CASE("--check contract: a valid spec builds & resolves topology without integrating") {
    // Mirrors what `./td_sim --check <config>` does at the library level:
    // load -> register -> build_from_spec (which resolves the signal topology
    // and emits the data-flow report) -> STOP, never constructing a Simulator.
    // The contract is: a well-formed spec yields a BuiltSystem with a fully
    // resolved evaluation order and a sane ODE state vector, and the build
    // path itself has no side effects that require integration to be valid.
    ensure_registry();

    // Build the reference spec (same one the TOML oracle uses) via the
    // programmatic oracle so this test is self-contained (no file I/O).
    auto bs = VehicleFactory::build_example_vehicle();

    // Topology resolved: every registered model appears exactly once in the
    // evaluation order, in a dependency-respecting permutation.
    auto& order = bs.builder.evaluation_order();
    REQUIRE(!order.empty());
    CHECK(order.size() == bs.models->entries.size());
    // No duplicates.
    auto sorted = order;
    std::sort(sorted.begin(), sorted.end());
    CHECK(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());

    // The ODE state vector is sized for the sum of all model dims and is
    // finite at t=0 (no integration has occurred, but the state is seeded).
    auto x = bs.sys.state();
    REQUIRE(!x.empty());
    for (double v : x) CHECK(std::isfinite(v));

    // A single derivative evaluation (what --check does NOT do, but proves
    // the built system is well-formed enough to evaluate) is finite and has
    // the same dimension as the state.
    std::vector<double> dxdt(x.size(), 0.0);
    bs.sys(x, dxdt, 0.0);
    REQUIRE(dxdt.size() == x.size());
    for (double v : dxdt) CHECK(std::isfinite(v));
}

TEST_CASE("--check contract: a topology-invalid spec fails at BUILD time (never integrates)") {
    // The whole point of --check is that a broken wiring is caught before any
    // integration. This locks in that an algebraic loop (a<->b via Signal) is
    // rejected inside build_from_spec, i.e. the failure happens on the build
    // path, not during a run.
    ensure_registry();
    VehicleSpec spec;
    spec.sim = test_params();
    // Two tanks wired into each other as a Signal loop: a.topology cycle.
    spec.models.push_back(model("a", "PressureSensor"));
    spec.models.push_back(model("b", "PressureSensor"));
    // 'reading' is a produced Signal; 'source' is a consumed Signal. Wire
    // a.reading -> b.source and b.reading -> a.source to form a cycle.
    spec.connections.push_back(conn("a.reading", SignalKind::Signal, {"a"}, {"b"}, "source"));
    spec.connections.push_back(conn("b.reading", SignalKind::Signal, {"b"}, {"a"}, "source"));
    CHECK_THROWS_AS(VehicleFactory::build_from_spec(spec), std::runtime_error);
}

TEST_CASE("REGRESSION: two producers -> one Accumulator, both contributions land") {
    ensure_registry();
    // Locks in the fix from SESSION_NOTES §9a.4: the CONSUMER's accumulator
    // slot is canonical and every producer is re-bound to it. Before the fix,
    // the first producer kept its own slot and its contribution was silently
    // dropped (total mass saw only one tank, thrust vanished, vehicle fell).
    VehicleSpec spec;
    spec.sim = test_params();
    spec.models.push_back(model("vehicle", "RigidBodyVehicle"));
    spec.models.push_back(model("tank_a", "PropellantTank", {{"initial_mass", 1000.0},
                                                             {"volume", 1.0}, {"density", 1000.0}}));
    spec.models.push_back(model("tank_b", "PropellantTank", {{"initial_mass", 2000.0},
                                                             {"volume", 1.0}, {"density", 1000.0}}));
    spec.connections.push_back(conn("tank_a.total_mass_contrib", SignalKind::Accumulator,
                                    {"tank_a"}, {"vehicle"}, "total_mass"));
    spec.connections.push_back(conn("tank_b.total_mass_contrib", SignalKind::Accumulator,
                                    {"tank_b"}, {"vehicle"}, "total_mass"));

    auto bs = VehicleFactory::build_from_spec(spec);

    // Order from the builder: both tanks run before the vehicle.
    auto& order = bs.builder.evaluation_order();
    REQUIRE(order.size() == 3);
    CHECK(index_of(order, "tank_a") < index_of(order, "vehicle"));
    CHECK(index_of(order, "tank_b") < index_of(order, "vehicle"));

    // One derivative evaluation: the vehicle's total_mass reader must see the
    // SUM of both tanks (3000), and gravity must scale with that total.
    std::vector<double> x = bs.sys.state();
    auto dxdt = one_eval(bs, x);

    auto slot_v = find_slot("vehicle", bs.core->slots());
    auto* vehicle = static_cast<RigidBodyVehicle*>(slot_v->model());
    CHECK(vehicle->total_mass.read() == doctest::Approx(3000.0));
    CHECK(vehicle->accel_z.read() == doctest::Approx(-vehicle->gravity));

    // d(vz)/dt must equal -g (no force, full mass): both tanks contributed.
    // vehicle state layout: [px,py,pz,vx,vy,vz], offset 0 in the global vector.
    CHECK(dxdt[5] == doctest::Approx(-vehicle->gravity));

    // The tanks' own mass derivatives are 0 (no engine drawing).
    CHECK(dxdt[state_offset_of("tank_a", bs)] == doctest::Approx(0.0));
    CHECK(dxdt[state_offset_of("tank_b", bs)] == doctest::Approx(0.0));
}

TEST_CASE("REGRESSION: TOML path and programmatic path produce identical physics") {
    ensure_registry();
    // The reference vehicle exists twice: config/vehicle.toml (the single
    // source of truth) and build_example_vehicle() (the test oracle, which
    // builds the same spec in memory). Any drift between the two must fail
    // here on the telemetry numbers.
    const char* cfg = "../config/vehicle.toml";
    std::ifstream probe(cfg);
    if (!probe.is_open()) {
        cfg = "config/vehicle.toml"; // running from the source tree
    }

    auto loaded = load_spec(cfg);
    // Disable telemetry for the TOML build; run it, then compare.
    std::string toml_out = "telemetry_toml.csv";
    std::string prog_out = "telemetry_prog.csv";
    loaded.sim.telemetry_file = toml_out;
    loaded.sim.telemetry_rate = 10.0;
    loaded.sim.t_end = 120.0;

    // Run both simulations in nested scopes so their telemetry files are
    // fully flushed (ofstream closed) before we read them back below.
    {
        auto bs_toml = VehicleFactory::build_from_spec(loaded);
        Simulator sim_toml(std::move(bs_toml), loaded.sim);
        sim_toml.run();
    }
    {
        auto params_prog = loaded.sim;
        params_prog.telemetry_file = prog_out;
        auto bs_prog = VehicleFactory::build_example_vehicle();
        Simulator sim_prog(std::move(bs_prog), params_prog);
        sim_prog.run();
    }

    // Compare the two runs row by row as strict strings. Column order is now
    // deterministic (Simulator::build_telemetry_columns sorts the reader
    // names), so identical physics produces byte-identical rows. This also
    // pins the schema (header) and the exact value formatting.
    auto read_lines = [](const std::string& f) {
        std::ifstream in(f);
        std::vector<std::string> rows;
        std::string l;
        while (std::getline(in, l)) rows.push_back(l);
        return rows;
    };
    auto ra = read_lines(toml_out);
    auto rb = read_lines(prog_out);
    REQUIRE(ra.size() > 100); // 120 s @ 10 Hz + final row
    REQUIRE(ra.size() == rb.size());
    for (std::size_t i = 0; i < ra.size(); ++i) {
        CHECK(ra[i] == rb[i]);
    }
    std::remove(toml_out.c_str());
    std::remove(prog_out.c_str());
}
