// td_sim3 — doctest entry point for the tdsim test suite.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "registry_setup.hpp"

TEST_CASE("registry: built-in model types are registered") {
    ensure_registry();
    auto& types = tdsim::VehicleFactory::model_types();
    CHECK(types.count("RigidBodyVehicle") == 1);
    CHECK(types.count("PropellantTank") == 1);
    CHECK(types.count("LiquidEngine") == 1);
    CHECK(types.count("FirstOrderActuator") == 1);
    CHECK(types.count("PressureSensor") == 1);
}
