#pragma once
// td_sim3 — simparams.hpp
//
// Integrator + telemetry parameters. This is a leaf header (no project
// includes) so it can be shared between the spec layer and the simulator
// without an include cycle.

#include <string>

namespace tdsim {

struct SimParams {
    double t_end = 0.0;
    double dt_macro = 0.01;      // discrete tick period (100 Hz default)
    double dt_min = 1e-6;        // odeint min step (driver-enforced floor)
    double dt_max = 1e-3;        // odeint max step (kept << dt_macro)
    double eps_abs = 1e-9;
    double eps_rel = 1e-9;
    double telemetry_rate = 100.0; // Hz
    std::string telemetry_file = "telemetry.csv";
};

} // namespace tdsim
