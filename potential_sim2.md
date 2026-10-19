## Detailed Architecture: C++20 Time-Domain Rocket Simulation

### 1. State Management: The Flat Vector Bridge
To satisfy the requirements of `boost::numeric::odeint`, the entire simulation state must be represented as a contiguous block of memory (typically a `std::vector<double>`). However, a rocket is naturally hierarchical.

*   **The State Manager:** This central registry handles the allocation of the global state vector. During the initialization phase, as components (Tanks, Engines, etc.) are created, the State Manager assigns them a specific starting index and a count of variables (e.g., a Tank might take 3 slots for mass, pressure, and temperature).
*   **State Views via `std::span`:** To avoid components having to perform manual index math (which is error-prone), the State Manager provides components with a `std::span<double>`. This allows a model to treat its piece of the global vector as a local array while maintaining the flat memory layout required by the solver.
*   **Derivative Contributions:** Components do not update the state directly. Instead, they write to a corresponding **derivative vector** ($\dot{x}$). If three different engines draw from one tank, each engine adds its mass-flow rate ($\dot{m}$) to the tank's mass derivative slot. The ODE solver then integrates these summed derivatives to find the next state.

### 2. The Execution Pipeline (The "Heartbeat")
The simulation must synchronize continuous physics (differential equations) with discrete logic (Flight Software). This is achieved through a strictly sequenced loop.

**The Sequence of Operations:**
1.  **The Discrete Scheduler:** A timer tracks the "simulated time." When the time crosses a 100Hz boundary (every 0.01s), the `FlightComputer` is triggered.
2.  **The Sense-Think-Act Cycle:**
    *   **Sense:** Sensors read values from the current state vector and write them to a **Sensor Buffer**.
    *   **Think:** The `FlightSoftware` (via an `IFlightSoftware` interface) processes the Sensor Buffer and determines the necessary control actions.
    *   **Act:** The computer writes these actions (e.g., valve positions, gimbal angles) into a **Command Buffer**.
3.  **The System Functor (`odeint` Integration):** The solver calls a "System Functor." This object iterates through all active components. Each component reads the **current state** and the **current commands** to calculate its derivative.
4.  **Integration Step:** `odeint` uses the calculated derivatives to advance the global state vector to $t + \Delta t$.
5.  **Telemetry Sampling:** At a user-defined rate, the global state and command buffers are snapshotted and written to the output file.

### 3. Inter-Model Communication & "Talking"
To avoid circular dependencies and "pointer soup," the architecture uses a **Connection Map** rather than direct object references.

*   **The Connection Map:** This is a runtime lookup table. For example, `Engine_1` is mapped to `FuelTank_A` and `OxidizerTank_B`. 
*   **Decoupled Interaction:** When `Engine_1` calculates its physics, it doesn't call a method on `FuelTank_A`. Instead:
    1.  It looks up the index for `FuelTank_A` via the map.
    2.  It reads the current pressure from that index in the state vector.
    3.  It calculates the required mass flow $\dot{m}$.
    4.  It adds $-\dot{m}$ to the derivative slot assigned to `FuelTank_A`.
*   **Ownership Resolution:** Global properties (like total vehicle acceleration) are handled by a "Vehicle Dynamics" component. This component sums the thrust from all active engines and divides by the total mass (the sum of all active tank masses) to compute the final acceleration.

### 4. Runtime Configuration & Flexibility
Because the vehicle build-up is unknown until runtime, the simulation employs a **Factory Pattern**.

*   **Configuration Parsing:** An input file (JSON/YAML) defines the hierarchy: `Vehicle` $\rightarrow$ `Stage` $\rightarrow$ `Component`.
*   **Dependency Injection:** The Factory instantiates the components and then "wires" them together. It assigns the State Manager indices and populates the Connection Map based on the names provided in the config file.
*   **Flight Software Abstraction:** By using an `IFlightSoftware` base class, the user can switch between a "Simple PID Controller" model and a "Full Flight Code" model in the config file without recompiling the simulation engine.

### 5. Handling Jettison Events and Discontinuities
Jettisoning a stage creates a "step change" in the system (e.g., a sudden drop in total mass). Most ODE solvers assume the system is smooth; a sudden jump can cause the solver to crash or take infinitesimally small steps to compensate.

*   **The Discontinuity Trigger:** When a jettison event is detected, the simulation pauses the integrator.
*   **State Modification:** The State Manager marks the jettisoned components as "Inactive." Their contributions to the derivatives are zeroed out, and their mass is subtracted from the vehicle total.
*   **Stepper Reset:** The `odeint` stepper's internal history (used for error estimation and step-size control) is now invalid. The simulation **must destroy and re-instantiate the stepper** (or call a reset method) before resuming integration from the new state.

### Technical Summary Table

| Concept | Implementation Detail | Purpose |
| :--- | :--- | :--- |
| **State Storage** | `std::vector<double>` $\rightarrow$ `std::span` | Performance and `odeint` compatibility |
| **Communication** | ID-based Connection Map | Prevents circular dependencies/tight coupling |
| **Discrete Logic** | Accumulator-based 100Hz trigger | Accurate Flight Software timing |
| **Interaction** | Summation in the derivative vector | Handles many-to-many (Engine $\leftrightarrow$ Tank) |
| **Software** | `IFlightSoftware` Interface | Allows interchangeable flight logic |
| **Events** | Integration Pause $\rightarrow$ State Update $\rightarrow$ Reset | Handles mass discontinuities (Jettison) |
