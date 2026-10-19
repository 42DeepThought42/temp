## Architecture Proposal: High-Fidelity Rocket Simulation

This architecture focuses on the strict separation of **continuous state** (physics), **discrete logic** (flight software), and **inter-model communication** (the data bus). By decoupling these, we satisfy the constraints of `boost::numeric::odeint` while allowing for a dynamic, runtime-configurable vehicle assembly.

---

### 1. State Management & ODE Integration
To prevent the corruption of the ODE stepper's internal state and ensure numerical stability, the simulation treats the physics state as a monolithic mathematical entity rather than a collection of object properties.

*   **The Global State Vector:** All integrable variables (e.g., mass, position, velocity, pressure) are stored in a single flat `std::vector<double>`. This is the only data structure passed to `boost::numeric::odeint`.
*   **State Views (`std::span`):** Instead of models owning their state, the orchestrator assigns each model a `std::span<double>` pointing to its specific slice of the global state vector. 
*   **The Derivative Function:** The system function passed to the ODE solver does not "update" the model. It asks each model to calculate its contribution to the derivatives ($\dot{y}$). Models read from their state span and the Data Bus and write their calculated rates of change into a corresponding derivative span.
*   **Strict Immutability:** During the integration step, models are forbidden from modifying the state vector or any parameters in the Data Bus. They only compute slopes.

### 2. Communication Architecture: The Data Bus
To resolve circular dependencies and "shaky ownership," the simulation employs a **Blackboard Pattern**. Models never call each other directly.

*   **The Registry:** A centralized key-value store (using hashed IDs or strings) that holds non-integrable data (e.g., commanded thrust, current valve positions, sensed pressures).
*   **Unidirectional Flow:** 
    *   **Model $\rightarrow$ Bus:** An engine calculates the required mass flow rate and publishes it to the bus.
    *   **Bus $\rightarrow$ Model:** A tank reads the required mass flow rate from the bus to calculate its mass derivative.
*   **Decoupling:** This removes the need for an engine to "know" which tank it is connected to at the code level. The connection is defined in the input file by assigning them to the same Registry Key.

### 3. Hybrid Time-Domain Execution
The simulation uses an orchestration loop to manage the intersection of continuous physics and discrete software ticks.

*   **The Orchestration Loop:**
    1.  **Integration Phase:** `odeint` advances the physics state from $t$ to $t + \Delta t$.
    2.  **Event Check:** The system checks for "discrete events" (e.g., stage jettison, timer expiration).
    3.  **Software Tick:** If $t$ has crossed a 100 Hz boundary (every 10ms), the integrator is paused. The **Flight Computer** model is executed:
        *   It reads sensor values from the Data Bus.
        *   It runs the **Flight Software** logic.
        *   It writes new commands (e.g., "Gimbal to 2 degrees") back to the Data Bus.
    4.  **Parameter Update:** Models update their internal constants based on the new Bus commands before the next integration phase begins.

### 4. Vehicle Hierarchy & Lifecycle
While the vehicle is conceptually a hierarchy, it is executed as a flat list for performance and simplicity.

*   **Composite Structure:** A `Vehicle` contains `Stages`, which contain `Components` (Engines, Tanks, etc.). This hierarchy is used for organization and initial setup.
*   **Execution List:** At runtime, the orchestrator flattens all active components into a linear list. When the ODE solver runs, it iterates through this flat list to collect derivatives.
*   **Dynamic Jettisoning:** When a stage is jettisoned, the orchestrator removes those components from the active execution list. The state vector indices for those components are marked as inactive or handled via a masking array to maintain the index mapping for remaining components.

### 5. Flight Software Interface
To allow multiple software models to run on the same hardware model, the architecture uses a strategy pattern reinforced by C++20 Concepts.

*   **`FlightSoftware` Concept:** Defines the requirements for any software model (e.g., must have an `update(inputs, outputs)` method).
*   **Abstract Interface:** A base class `IFlightSoftware` allows the `FlightComputer` model to hold a `std::unique_ptr<IFlightSoftware>`, enabling the software to be swapped at runtime via the input file.

### 6. Telemetry & Configuration
*   **The Telemetry Buffer:** Each model implements a `capture_telemetry()` method. At a defined sampling rate, the orchestrator calls this method on all active models and pushes the results into a high-performance ring buffer. A separate thread handles the I/O to write this buffer to a file, ensuring telemetry doesn't block the physics solver.
*   **Runtime Factory:** A `VehicleFactory` reads the input file (JSON/XML/YAML). It:
    1.  Instantiates the requested models.
    2.  Allocates the necessary space in the global state vector.
    3.  Maps the "connections" from the input file to shared keys in the Data Bus.
    4.  Assembles the flat execution list for the orchestrator.
