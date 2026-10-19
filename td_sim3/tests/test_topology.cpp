// td_sim3 — unit tests for the SimulationBuilder contract (topology.hpp):
// signal-kind validation, topological ordering, cycle detection, edge
// deduplication, and jettison re-sort.

#include <doctest/doctest.h>

#include <algorithm>

#include "tdsim/topology.hpp"

using namespace tdsim;

namespace {

bool contains(const std::vector<std::string>& v, const std::string& s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

std::size_t index_of(const std::vector<std::string>& v, const std::string& s) {
    return static_cast<std::size_t>(std::find(v.begin(), v.end(), s) - v.begin());
}

} // namespace

TEST_CASE("duplicate model ids are rejected") {
    SimulationBuilder b;
    b.register_model("a");
    CHECK_THROWS_AS(b.register_model("a"), std::runtime_error);
}

TEST_CASE("wiring to an unknown model is rejected") {
    SimulationBuilder b;
    b.register_model("a");
    b.register_model("b");
    CHECK_THROWS_AS(b.wire("s", SignalKind::Signal, "ghost", "b"), std::runtime_error);
    CHECK_THROWS_AS(b.wire("s", SignalKind::Signal, "a", "ghost"), std::runtime_error);
}

TEST_CASE("Signal with exactly one producer resolves; order respects the edge") {
    SimulationBuilder b;
    b.register_model("producer");
    b.register_model("consumer");
    b.wire("p.out", SignalKind::Signal, "producer", "consumer");

    auto order = b.resolve_topology();
    REQUIRE(order.size() == 2);
    CHECK(index_of(order, "producer") < index_of(order, "consumer"));
}

TEST_CASE("Signal with two producers is a hard error") {
    SimulationBuilder b;
    b.register_model("p1");
    b.register_model("p2");
    b.register_model("c");
    b.wire("s", SignalKind::Signal, "p1", "c");
    b.wire("s", SignalKind::Signal, "p2", "c");
    CHECK_THROWS_AS(b.resolve_topology(), std::runtime_error);
}

TEST_CASE("Latched with two producers is a hard error") {
    SimulationBuilder b;
    b.register_model("p1");
    b.register_model("p2");
    b.register_model("c");
    b.wire("l", SignalKind::Latched, "p1", "c");
    b.wire("l", SignalKind::Latched, "p2", "c");
    CHECK_THROWS_AS(b.resolve_topology(), std::runtime_error);
}

TEST_CASE("0-producer Accumulator is a warning, not an error") {
    SimulationBuilder b;
    b.register_model("c");
    b.wire("a", SignalKind::Accumulator, "c", "c"); // self-wire registers the connection
    auto order = b.resolve_topology(); // must NOT throw
    CHECK(order.size() == 1);
}

TEST_CASE("Accumulator: consumer runs after ALL producers") {
    SimulationBuilder b;
    b.register_model("pa");
    b.register_model("pb");
    b.register_model("sink");
    b.wire("a", SignalKind::Accumulator, "pa", "sink");
    b.wire("a", SignalKind::Accumulator, "pb", "sink");

    auto order = b.resolve_topology();
    REQUIRE(order.size() == 3);
    CHECK(index_of(order, "pa") < index_of(order, "sink"));
    CHECK(index_of(order, "pb") < index_of(order, "sink"));
    // Producers are order-free among themselves.
    CHECK(contains(order, "pa"));
    CHECK(contains(order, "pb"));
}

TEST_CASE("Latched creates NO topology edge (algebraic-loop breaker)") {
    SimulationBuilder b;
    // a -> b only via a Latched connection: no ordering constraint may be
    // imposed, otherwise the latch cannot break an instantaneous loop.
    b.register_model("a");
    b.register_model("b");
    b.wire("l", SignalKind::Latched, "a", "b");

    auto order = b.resolve_topology();
    REQUIRE(order.size() == 2);
    // Either order is legal; just assert nothing threw and both are present.
    CHECK(contains(order, "a"));
    CHECK(contains(order, "b"));
}

TEST_CASE("algebraic cycle (Signal loop) is detected and named") {
    SimulationBuilder b;
    b.register_model("a");
    b.register_model("b");
    b.wire("a.out", SignalKind::Signal, "a", "b");
    b.wire("b.out", SignalKind::Signal, "b", "a");

    bool threw = false;
    std::string err;
    try {
        b.resolve_topology();
    } catch (const std::runtime_error& e) {
        threw = true;
        err = e.what();
    }
    CHECK(threw);
    CHECK(err.find("algebraic loop") != std::string::npos);
    CHECK(err.find("a") != std::string::npos);
    CHECK(err.find("b") != std::string::npos);
}

TEST_CASE("duplicate edges are deduplicated (no false cycle)") {
    // Wiring the same (signal, producer, consumer) triple twice must not
    // double-decrement Kahn's in-degree and falsely report a cycle.
    SimulationBuilder b;
    b.register_model("a");
    b.register_model("b");
    b.wire("s", SignalKind::Signal, "a", "b");
    b.wire("s", SignalKind::Signal, "a", "b");

    auto order = b.resolve_topology();
    REQUIRE(order.size() == 2);
    CHECK(index_of(order, "a") < index_of(order, "b"));
}

TEST_CASE("jettison: stage models are dropped, survivors re-resolved") {
    SimulationBuilder b;
    b.register_model("vehicle", "vehicle");
    b.register_model("tank", "stage1");
    b.register_model("engine", "stage1");
    b.wire("tank.mass", SignalKind::Signal, "tank", "vehicle");
    b.wire("engine.force", SignalKind::Accumulator, "engine", "vehicle");

    auto pre = b.resolve_topology();
    REQUIRE(pre.size() == 3);

    b.inactivate_stage("stage1");
    auto post = b.resolve_topology_after_jettison();
    REQUIRE(post.size() == 1);
    CHECK(post[0] == "vehicle");
}

TEST_CASE("jettison of a cycle participant resolves the loop") {
    SimulationBuilder b;
    b.register_model("a", "s1");
    b.register_model("b", "s1");
    b.register_model("c", "s2");
    // a <-> b cycle, both jettisoned; c is independent.
    b.wire("a.out", SignalKind::Signal, "a", "b");
    b.wire("b.out", SignalKind::Signal, "b", "a");
    b.wire("c.out", SignalKind::Signal, "c", "a"); // c feeds into the cycle

    b.inactivate_stage("s1");
    auto post = b.resolve_topology_after_jettison();
    REQUIRE(post.size() == 1);
    CHECK(post[0] == "c");
}
