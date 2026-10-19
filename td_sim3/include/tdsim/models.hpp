#pragma once
// td_sim3 — models.hpp
//
// Built-in reference models. Each is a small struct deriving
// StatefulModel<Dim> with declared signal views. These double as the
// developer's template for adding new models: write the struct, declare the
// signals it provides/consumes, implement eval(), then register the type in
// the factory registry (see register_builtin_types() in factory.cpp for the
// registration pattern: ports, params, initial state, accumulators, readers,
// proposers, latches).
//
// The FirstOrderActuator is the canonical "discrete command -> continuous
// effect" reference: it consumes a Latched command and provides a Signal
// output, with a first-order lag  x_dot = (cmd - x) / tau.

#include <algorithm>
#include <cmath>
#include <string>

#include "model.hpp"
#include "signal.hpp"

namespace tdsim {

// ---------------------------------------------------------------------------
// RigidBodyVehicle — 6-DOF point-mass translational dynamics (reduced: 3-DOF
// translational for the first template; attitude can be added later).
// State: [px, py, pz, vx, vy, vz]. Acceleration is algebraic (from total
// force / total mass).
//   Provided ports: total_force_x/y/z (Accumulator), total_mass (Accumulator),
//                   accel_x/y/z (Signal)
// ---------------------------------------------------------------------------
struct RigidBodyVehicle : StatefulModel<6> {
    static constexpr std::size_t DIM = 6;
    StateDescriptor desc{"vehicle", {"px", "py", "pz", "vx", "vy", "vz"}};

    // Provided.
    Accumulator<double> total_force_x;
    Accumulator<double> total_force_y;
    Accumulator<double> total_force_z;
    Accumulator<double> total_mass;
    Signal<double> accel_x;
    Signal<double> accel_y;
    Signal<double> accel_z;

    double gravity = 9.81; // acts in -z (up = +z)

    void eval(const SimContext& ctx, const StateView& sv) override {
        (void)ctx;
        // Read accumulated forces and mass (all engines/tanks have run earlier
        // in topological order, per the builder's edges).
        double fx = total_force_x.read();
        double fy = total_force_y.read();
        double fz = total_force_z.read() - gravity * total_mass.read();
        double m = total_mass.read();

        // Publish acceleration (algebraic).
        double ax = (m > 1e-9) ? fx / m : 0.0;
        double ay = (m > 1e-9) ? fy / m : 0.0;
        double az = (m > 1e-9) ? fz / m : 0.0;
        accel_x.publish(ax);
        accel_y.publish(ay);
        accel_z.publish(az);

        // Derivatives: d(pos)/dt = vel, d(vel)/dt = accel.
        sv.dxdt[0] = sv.state[3]; // px' = vx
        sv.dxdt[1] = sv.state[4]; // py' = vy
        sv.dxdt[2] = sv.state[5]; // pz' = vz
        sv.dxdt[3] = ax;          // vx' = ax
        sv.dxdt[4] = ay;          // vy' = ay
        sv.dxdt[5] = az;          // vz' = az
    }
};

// ---------------------------------------------------------------------------
// PropellantTank — integrates propellant mass; provides mass + ullage pressure
// (algebraic, from mass/volume). Consumes total_draw (Accumulator of mdot from
// engines) to compute its mass derivative.
//   State slots: mass   (Dim=1)
//   Provided ports: mass_out (Signal), ullage_pressure (Signal),
//                   ullage_pressure_latched (Latched),
//                   total_draw (Accumulator, consumed),
//                   total_mass_contrib (Accumulator, produced -> vehicle)
// ---------------------------------------------------------------------------
struct PropellantTank : StatefulModel<1> {
    static constexpr std::size_t DIM = 1;
    StateDescriptor desc{"tank", {"mass"}};

    // Provided.
    Signal<double> mass_out;             // current propellant mass (algebraic mirror)
    Signal<double> ullage_pressure;      // live pressure, for sensors
    Latched<double> ullage_pressure_latched; // for engines: breaks the
    // engine<->tank instantaneous pressure/draw algebraic loop (one-step delay)
    // Consumed: engines accumulate their mdot here.
    Accumulator<double> total_draw;   // sum of mdot from all drawing engines
    // The tank contributes its current mass to the vehicle's total_mass
    // accumulator (wired by the factory to the vehicle's accumulator slot).
    Accumulator<double> total_mass_contrib;

    double volume = 1.0;
    double density = 1000.0; // kg/m^3 (for pressure model)
    double initial_mass = 0.0;

    // The latch's initial committed value is set by the factory from the
    // initial fill fraction (see register_builtin_types in factory.cpp).

    void eval(const SimContext& ctx, const StateView& sv) override {
        (void)ctx;
        double m = sv.state[0];
        mass_out.publish(m);
        // Simple ullage pressure model: proportional to fill fraction.
        double fill = (volume > 1e-9) ? std::clamp(m / (density * volume), 0.0, 1.0) : 0.0;
        ullage_pressure.publish(fill * 1e5); // up to 1 bar
        ullage_pressure_latched.propose(fill * 1e5); // engines read this (latched)
        // Contribute this tank's mass to the vehicle total.
        total_mass_contrib.add(m);

        // dm/dt = -total_draw (engines summed).
        sv.dxdt[0] = -total_draw.read();
    }
};

// ---------------------------------------------------------------------------
// LiquidEngine — consumes fuel/oxi tank pressures + throttle command (Latched),
// provides thrust (adds to vehicle force accumulator) and mdot (adds to each
// tank's total_draw accumulator).
//   State: none (Dim=0) — pure algebraic.
//   Consumed ports: fuel_pressure (Latched), oxi_pressure (Latched),
//                   throttle_pos (Signal)
//   Provided ports: thrust (Signal), force_x/force_z (Accumulator),
//                   fuel_draw/oxi_draw (Accumulator)
// ---------------------------------------------------------------------------
struct LiquidEngine : StatefulModel<0> {
    static constexpr std::size_t DIM = 0;
    StateDescriptor desc{"engine", {}};

    // Consumed.
    // Pressure inputs are LATCHED: the engine reads the tank's last-committed
    // pressure, not the instantaneous one. This breaks the engine<->tank
    // algebraic loop (engine mdot -> tank mass -> tank pressure -> engine).
    Latched<double> fuel_pressure;
    Latched<double> oxi_pressure;
    Signal<double> throttle_pos; // continuous throttle from the actuator (0..1)

    // Provided.
    Signal<double> thrust;
    // The engine adds to these accumulators (wired to the vehicle + tanks).
    Accumulator<double> force_x; // (vehicle total_force_x)
    Accumulator<double> force_z; // (vehicle total_force_z)
    Accumulator<double> fuel_draw;  // (fuel tank total_draw)
    Accumulator<double> oxi_draw;   // (oxi tank total_draw)

    double isp = 280.0;     // specific impulse
    double max_thrust = 1.0; // at full throttle, full pressure
    double g0 = 9.81;

    void eval(const SimContext& ctx, const StateView& sv) override {
        (void)sv; // no ODE state
        (void)ctx;
        double throttle = throttle_pos.read();
        // Pressure-based flow factor (simplified): flow scales with sqrt of
        // available pressure, zero if either propellant is exhausted.
        double pf = std::max(0.0, fuel_pressure.read());
        double po = std::max(0.0, oxi_pressure.read());
        double pmin = std::min(pf, po);
        double flow_factor = std::sqrt(std::max(0.0, pmin) / 1e5); // normalized
        double T = throttle * flow_factor * max_thrust;

        thrust.publish(T);
        // Mass flow rate from thrust and Isp.
        double mdot = T / (isp * g0);
        // Split mdot between fuel and oxi (1:1 for simplicity).
        double half = 0.5 * mdot;
        fuel_draw.add(half);
        oxi_draw.add(half);
        // Add thrust to vehicle force accumulators (pointing +z for template).
        force_z.add(T);
        force_x.add(0.0);
    }
};

// ---------------------------------------------------------------------------
// FirstOrderActuator — the reference "discrete command -> continuous effect"
// model. Consumes a Latched command, integrates a first-order lag, and provides
// the actual (smoothed) actuator position as a Signal.
//   State slots: x   (Dim=1)
//   x_dot = (cmd - x) / tau
//   Consumed ports: cmd (Latched)
//   Provided ports: position (Signal)
// ---------------------------------------------------------------------------
struct FirstOrderActuator : StatefulModel<1> {
    static constexpr std::size_t DIM = 1;
    StateDescriptor desc{"actuator", {"x"}};

    // Consumed.
    Latched<double> cmd; // discrete command (1-step delayed)
    // Provided.
    Signal<double> position; // smoothed output

    double tau = 0.05; // time constant (seconds)
    double x0 = 0.0;   // initial position
    double cmd0 = 0.0; // initial committed command (latch init)

    void eval(const SimContext& ctx, const StateView& sv) override {
        (void)ctx;
        double x = sv.state[0];
        double c = cmd.read();
        position.publish(x);
        sv.dxdt[0] = (c - x) / tau;
    }
};

// ---------------------------------------------------------------------------
// PressureSensor — pure algebraic (Dim=0). Reads a source pressure signal and
// re-publishes it (with optional noise/offset) under a sensor name. This is the
// "sensors sense state of any model" pattern: it consumes a Signal.
//   Consumed ports: source (Signal)
//   Provided ports: reading (Latched) — the sensor's output is a zero-order
//     hold at its sample rate. The producer propose()s a fresh value every
//     Pass 2 (re-derived from the live source each derivative call); the
//     driver commits only on the sensor's tick (period_ticks), so the held
//     value is the quantity sampled at the last sensor tick. period is set by
//     the factory from the 'rate' param (Hz) via the base-tick GCD resolver.
// ---------------------------------------------------------------------------
struct PressureSensor : StatefulModel<0> {
    static constexpr std::size_t DIM = 0;
    StateDescriptor desc{"sensor", {}};

    Signal<double> source;   // consumed
    Latched<double> reading; // provided (held at the sensor's sample rate)

    double offset = 0.0;
    double rate = 0.0; // Hz; 0 = continuous (commit every base tick)

    void eval(const SimContext& ctx, const StateView& sv) override {
        (void)ctx;
        (void)sv;
        reading.propose(source.read() + offset);
    }
};

} // namespace tdsim
