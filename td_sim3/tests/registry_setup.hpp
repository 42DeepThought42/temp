// td_sim3 — tests/registry_setup.hpp
//
// Ensure the model-type registry is populated before the first test that
// calls VehicleFactory::build_from_spec() runs. doctest orders test cases by
// translation unit, so a static initializer in tests/main.cpp alone would not
// be guaranteed to run first; every test TU that builds a system calls
// ensure_registry() (idempotent) at the start of the relevant cases.

#pragma once

#include "tdsim/factory.hpp"

inline void ensure_registry() {
    static int done = [] {
        tdsim::VehicleFactory::register_builtin_types();
        return 0;
    }();
    (void)done;
}
