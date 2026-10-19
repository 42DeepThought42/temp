# Architecture Specification: Topological Multi-Pass Registry (TMPR)

This architecture is optimized for mixed-domain (continuous/discrete) engineering simulations where the physical topology changes at runtime.

---

## Core Pillars & Mechanisms

### 1. Structural Pattern: Flat Component Registry
*   **What it is:** A flat registry (`SimulationRegistry`) that manages models via polymorphic base pointers and maps their positions to a single, resizeable `std::vector<double>` continuous state vector.
*   **How it differs from OOP:** Traditional architectures build a deeply nested composite tree (e.g., Vehicle owns Stage owns Tank). TMPR flattens this. Models register their continuous state needs at runtime, and the registry handles index offset maps.
*   **Why:** Allows you to seamlessly resize the raw continuous array and recalculate indices when a model (like an engine or booster stage) is jettisoned mid-flight.

### 2. Execution Pattern: 3-Pass Topological Derivative Pipeline
*   **What it is:** A strict three-phase horizontal sweep across all active models during every derivative call (`operator()` in `boost::numeric::odeint`):
    1.  **Clear:** Resets all global or inter-model algebraic accumulator signals (e.g., total forces, mass flow rates).
    2.  **Evaluate:** Executes algebraic ports sequentially via a topologically sorted graph (Tanks read -> Sensors read -> Engines calculate thrust -> Vehicle calculates acceleration).
    3.  **Integrate:** Models read the final resolved algebraic signals and populate the `dxdt` derivative vector.
*   **Why:** Eliminates circular dependencies and causal loops. Models never call each other dynamically; they rely on a directed acyclic data flow sorted at simulation startup.

### 3. Domain Boundary Pattern: Dual-Domain Temporal Isolator
*   **What it is:** A strict separation between **Continuous Vector States** (integrated speculatively by `odeint`'s variable step sizes) and **Discrete Domain States** (Flight Software variables updated exclusively at a fixed macro rate, e.g., 100 Hz).
*   **Why:** It keeps the Flight Software completely hidden from `odeint`’s intermediate trial steps, preventing discrete state changes from corrupting the continuous mathematical step logic.

### 4. Telemetry Pattern: Asynchronous Double-Buffered Logger
*   **What it is:** At fixed macro boundaries (100 Hz), a raw memory copy of all telemetry-tagged model variables is pushed into a lock-free queue. A dedicated background thread (`std::jthread`) consumes this queue to format and write the data to disk.
*   **Why:** Prevents disk I/O bottlenecks from lagging the high-frequency integration solver loop.

---

## Pipeline & Execution Diagrams

### 1. The 3-Pass Derivative Pipeline (Per-Substep Flow)
During every intermediate evaluation that `boost::numeric::odeint` performs, the system steps through this linear, unidirectional sequence. There are no backward loops or side-channel queries between models.

[odeint calls Wrapper(x, dxdt, t)]
│
├──► PASS 1: CLEAR ACCUMULATORS
│    └── Loop all active models ──► Reset algebraic registers (Forces, Mass Flows, Torques = 0)
│
├──► PASS 2: TOPOLOGICAL ALGEBRAIC EVALUATION (Data flows downstream via sorted Graph)
│    ├── 1. Tanks       ──► Read x vector ──► Output Current Mass & Pressures
│    ├── 2. Sensors     ──► Read x vector ──► Output Pressures, Accel (State or Current Algebraic)
│    ├── 3. Engines     ──► Read Tank/Sensor Ports ──► Output Thrust Forces & Fuel Burn Rates (mdot)
│    └── 4. Vehicle     ──► Accumulate All Thrust/Mass ──► Compute Global Accelerations
│
├──► PASS 3: DERIVATIVE POPULATION
│    └── Loop all active models ──► Write rates of change (dx/dt) into dxdt vector
│                                   (e.g., Tank writes -mdot, Vehicle writes Velocity/Accel)
│
[Return to odeint]

### 2. The Multi-Rate Coordination Loop (Macro vs. Micro Time)
This diagram illustrates how the continuous integration domain is nested inside the discrete execution domain, preventing state corruption.

Discrete Time T (e.g., 0.00s, 0.01s, 0.02s)
│
├──► [DISCRETE STEP] Latch Sensor Hardware Snapshots
├──► [DISCRETE STEP] Run Flight Software Algorithms (100 Hz Fixed Rate)
├──► [DISCRETE STEP] Update Command Actuator Registers (Valves, Gimbal Angles)
│
└──► [CONTINUOUS STEP WINDOW] (Time: T ──► T + 0.01s)
│
├──► odeint Speculative Step 1 (t = T + 0.002s) ──► Runs 3-Pass Pipeline
├──► odeint Speculative Step 2 (t = T + 0.007s) ──► Runs 3-Pass Pipeline
└──► odeint Converged Boundary  (t = T + 0.010s) ──► End of continuous window
│
├──► [EVENT STEP] Check Jettison Triggers ──► (If triggered, Resize Vector & Re-index Mapping)
├──► [TELEMETRY] Push Flat Binary Copy to Log Queue
│
Advance to next Discrete Time T + dt...

---

## 3. Model Communication Data Contract
To keep models decoupled, they utilize a Port-to-Port structural layout. Instead of an Engine directly calling a Tank object, it connects to an intermediate data slot. An LLM can easily generate new models using this rigid schema:

```cpp
// Explicit, flat data structures acting as the "wiring board" between models
struct AlgebraicPorts {
    // Tank Outputs / Engine Inputs
    double tank_ullage_pressure = 0.0;
    double tank_mass_flow_rate_demand = 0.0;
    
    // Engine Outputs / Vehicle Inputs
    double engine_produced_thrust = 0.0;
    double engine_fuel_burn_rate = 0.0;
    
    // Vehicle Outputs / Sensor Inputs
    double vehicle_total_mass = 0.0;
    double vehicle_axial_acceleration = 0.0;
};
```




