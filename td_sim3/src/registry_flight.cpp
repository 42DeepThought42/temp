// td_sim3 — registry_flight.cpp
//
// Registration of the built-in flight software (VehicleFlightSoftware), split
// out of the example so concrete flight-software knowledge lives in the
// library, not the executable. This mirrors src/registry_builtin.cpp for model
// types (see DEVELOPERS_GUIDE §4 for the model-type analogue). The example
// (examples/td_sim.cpp) no longer knows which flight software exist — it only
// looks them up by name through FlightSoftwareRegistry.

#include "tdsim/flight_software.hpp"

#include <map>
#include <memory>
#include <string>

namespace tdsim {

namespace {

// The backing store, keyed by name. A std::map (not unordered_map) so that
// --list-types prints software in a stable, sorted order.
std::map<std::string, std::function<std::unique_ptr<IFlightSoftware>()>>&
registry_storage() {
    static std::map<std::string, std::function<std::unique_ptr<IFlightSoftware>()>> r;
    return r;
}

} // namespace

void FlightSoftwareRegistry::register_software(
    const std::string& name,
    std::function<std::unique_ptr<IFlightSoftware>()> make) {
    registry_storage()[name] = std::move(make);
}

void FlightSoftwareRegistry::register_builtin_software() {
    // VehicleFlightSoftware: the single "one brain, many loops" software for
    // the reference vehicle (throttle hold + PID gimbal stabilization).
    register_software(
        "VehicleFlightSoftware",
        []() -> std::unique_ptr<IFlightSoftware> {
            return std::make_unique<VehicleFlightSoftware>();
        });
}

const std::function<std::unique_ptr<IFlightSoftware>()>*
FlightSoftwareRegistry::find(const std::string& name) {
    auto it = registry_storage().find(name);
    if (it == registry_storage().end()) return nullptr;
    return &it->second;
}

const std::map<std::string, std::function<std::unique_ptr<IFlightSoftware>()>>&
FlightSoftwareRegistry::software() {
    return registry_storage();
}

} // namespace tdsim
