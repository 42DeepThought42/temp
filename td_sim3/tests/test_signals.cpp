// td_sim3 — unit tests for the signal layer (signal.hpp):
// Signal / Accumulator / Latched semantics and the SlotStore bound check.

#include <doctest/doctest.h>

#include "tdsim/signal.hpp"

using namespace tdsim;



TEST_CASE("Signal: publish once, every reader sees the value") {
    Signal<double> writer, r1, r2;
    double slot = 0.0;
    writer.bind(&slot, "sig");
    r1.bind(&slot, "sig");
    r2.bind(&slot, "sig");

    writer.publish(42.5);
    CHECK(r1.read() == 42.5);
    CHECK(r2.read() == 42.5);
    writer.publish(-1.0);
    CHECK(r1.read() == -1.0);
}

TEST_CASE("Accumulator: add() is commutative summation; clear() resets") {
    // In the real factory the shared slot is zero-initialized (SlotStore
    // value-initializes its backing array), so a fresh accumulator reads 0.
    double slot = 0.0;
    Accumulator<double> a, p1, p2, p3;
    a.bind(&slot, "acc");
    p1.bind(&slot, "acc");
    p2.bind(&slot, "acc");
    p3.bind(&slot, "acc");

    CHECK(a.read() == 0.0); // fresh accumulator is zero, not garbage

    p1.add(1.0);
    p3.add(0.5); // order of add() calls is irrelevant
    p2.add(2.0);
    CHECK(a.read() == doctest::Approx(3.5));

    // clear() forces the slot back to zero even mid-window; the next add()
    // then starts from 0, not from the previous sum.
    a.clear();
    CHECK(a.read() == 0.0);
    p1.add(7.0);
    CHECK(a.read() == doctest::Approx(7.0));
}

TEST_CASE("Latched: read() is frozen at the committed value until commit()") {
    Latched<double> l;
    double cur = 0.0, nxt = 0.0;
    l.bind(&cur, &nxt, "latch");

    l.init(5.0);
    CHECK(l.read() == 5.0);

    // propose() writes 'next' only: consumers never see it.
    l.propose(9.0);
    CHECK(l.read() == 5.0);
    l.propose(11.0); // latest proposal wins
    CHECK(l.read() == 5.0);

    // commit() promotes next -> current, once per macro window.
    l.commit();
    CHECK(l.read() == 11.0);

    // A window with no proposal leaves the committed value in place
    // (init() seeded next == current, so this is what the driver sees).
    l.commit();
    CHECK(l.read() == 11.0);
}

TEST_CASE("SlotStore: throws on exhaustion instead of overflowing") {
    SlotStore store{3};
    REQUIRE_NOTHROW(store.slot(0));
    REQUIRE_NOTHROW(store.slot(2));
    CHECK_THROWS_AS(store.slot(3), std::runtime_error);
    // The store is still usable within its capacity afterwards.
    CHECK(store.slot(1) != nullptr);
}
