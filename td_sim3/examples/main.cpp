// td_sim3 — example driver.
//
// Builds the reference example vehicle, attaches a single 100 Hz flight
// computer (one VehicleFlightSoftware instance running both the throttle-hold
// and PID gimbal loops), and a stage-1 jettison event at a fixed time.
//
// One flight computer per vehicle, one flight software instance per vehicle.
// The software internally runs multiple control loops (throttle + gimbal) and
// proposes commands to both actuators in a single tick.

#include <iostream>

#include "tdsim/factory.hpp"
#include "tdsim/simulator.hpp"

int main() {
    auto built = tdsim::VehicleFactory::build_example_vehicle();

    tdsim::SimParams params;
    params.t_end = 120.0;
    params.dt_macro = 0.01;     // 100 Hz discrete tick
    params.dt_min = 1e-6;
    params.dt_max = 1e-3;
    params.eps_abs = 1e-9;
    params.eps_rel = 1e-9;
    params.telemetry_rate = 10.0; // 10 Hz log (keep the file small)
    params.telemetry_file = "telemetry.csv";

    tdsim::Simulator sim(std::move(built), params);

    // Single flight computer running the vehicle's flight software. The
    // VehicleFlightSoftware internally manages both the throttle (open-loop
    // hold at full thrust) and the gimbal (PID hold at 0). It reads the
    // gimbal position sensor and proposes commands to both actuators.
    auto fs = std::make_unique<tdsim::VehicleFlightSoftware>();
    fs->throttle_setpoint = 1.0;  // full thrust for the demo
    fs->gimbal_kp = 10.0;
    fs->gimbal_ki = 0.0;
    fs->gimbal_kd = 0.0;
    fs->gimbal_setpoint = 0.0;    // aligned
    fs->dt = params.dt_macro;

    sim.add_flight_computer(
        "flightcomp", std::move(fs),
        /*sensor_signals=*/{"gimbal_act.position"},
        /*command_signals=*/{"throttle_act.cmd", "gimbal_act.cmd"});

    // Jettison stage 1 at a fixed time (t = 60 s) to exercise the
    // mid-run discontinuity path: the stepper is torn down, stage-1 models
    // are deactivated, the topology re-resolves, and the vehicle coasts
    // ballistically on stage 2. (A real vehicle would trigger on tank-empty;
    // the fixed-time trigger keeps the demo deterministic.)
    sim.add_jettison_event(tdsim::JettisonEvent{
        "stage1",
        [](double t, const tdsim::SystemFunctor&) { return t >= 60.0; },
        false});

    sim.run();
    return 0;
}
