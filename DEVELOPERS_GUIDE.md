# td_sim3 — Developer's Guide

A time-domain rocket/vehicle simulation template in C++20 built on `boost::numeric::odeint`.
The vehicle (models, parameters, signal wiring, events, flight software, integrator settings)
is **described in a TOML file and discovered at runtime** — nothing about a specific vehicle is
baked into the core library. This guide assumes you are coming back to the project occasionally to
either **run/make a new simulation** or **add a new model type**.

---

## 1. The 30-second version

```bash
cd td_sim3
cmake -S . -B build && cmake --build build        # build (C++20, Boost + spdlog + toml++)
./build/td_sim --list-types                       # what model types & flight software exist
./build/td_sim --check config/vehicle.toml        # validate a config without running
./build/td_sim config/vehicle.toml                # run the reference 120 s flight
```

Output: a startup **Engineering Data Flow & Topology Report** (the resolved evaluation order +
who reads/writes what), spdlog lines, and a `telemetry.csv` next to the binary.
The reference vehicle burns 0–60 s, jettisons stage 1 at t=60 s, and coasts ballistically to
t=120 s. Expected physics at t=120: `vz ≈ 976.03 m/s`, `pz ≈ 85,538 m`, fuel/oxi frozen at
`12,445.8 kg` after jettison. If you don't see that, something regressed.

There are two executables:

| Binary | Purpose |
|---|---|
| `td_sim` | The real path: loads a TOML config, builds the vehicle, runs it. |
| `td_sim_example` | Programmatic build of the *same* reference vehicle (routes through the same `build_from_spec` with an in-memory spec). It exists as a test oracle — its physics must match `td_sim` exactly. |

---

## 2. How the architecture works (just enough to not break it)

### 2.1 State
- One flat `std::vector<double>` is the **ODE state** (only truly integrated variables: tank
  masses, vehicle pos/vel, actuator positions). One flat `dxdt` alongside it.
- Models get `std::span` views into that vector at build time. **Never do index math in model
  code** — the framework positions the spans.
- Everything algebraic (pressure, thrust, mass flows, acceleration) is **not integrated**; it is
  recomputed on every derivative call.

### 2.2 Signals — the only channel between models (`include/tdsim/signal.hpp`)

| Type | Producers | Model API | Topology rule | Use for |
|---|---|---|---|---|
| `Signal<T>` | exactly 1 | `publish()` / `read()` | DAG edge producer→consumer; consumer runs after producer | one-to-many instantaneous data (sensor → computer, actuator position → engine) |
| `Accumulator<T>` | 0+ (0 is a *warning*) | `add()` (+=) / `read()`; framework clears it every call | consumer runs after **all** producers | many-to-many summation (all engines → vehicle force; all tanks → vehicle mass; all engines → one tank's draw) |
| `Latched<T>` | exactly 1 | `propose()` / `read()` (reads last **committed** value); framework `init()` + `commit()` per macro window | **no edge** — this is the sanctioned algebraic-loop breaker | any instantaneous feedback loop (engine reads tank pressure → 1-step delay), and all discrete flight-computer commands |

**Which one to use — decision rules:**

1. **Does the consumer need a SUM of several producers?** (many engines → one total force,
   many tanks → one total mass) → `Accumulator`. This is the only type that adds.
2. **Otherwise, does the data path close a loop?** (the consumer, directly or through a
   chain of Signals, feeds back to the producer) → `Latched`. It is the *only* legal loop
   breaker. If you're unsure whether there's a loop, wire it as a `Signal` first and let
   `resolve_topology` tell you — a cycle is a hard build error that names the participants,
   then latch one edge on that cycle.
3. **Otherwise** (one producer, fan-out, no loop — sensor → computer, actuator position →
   engine) → `Signal`.

Two clarifications that the table above leaves implicit:

- **Single-producer `Accumulator` is allowed and normal.** The reference vehicle wires
  `engine1.force_z → vehicle` as an Accumulator even though it has one producer today, because
  the port is *designed* to accept more engines later and the consumer owns the sum slot. If a
  port will ever take >1 producer, make it an Accumulator from the start; if it is
  strictly one-to-one forever, a `Signal` is the simpler choice. Either builds fine at N=1.
- **Latching is not free — it is a 1-step (≤ `dt_macro`, 10 ms) delay in physics.** Use it on
  the *slowest / least sensitivity-critical* edge of the cycle (e.g. tank pressure → engine,
  not a fast gimbal feedback edge) and know you're trading 10 ms of lag for a solvable DAG.
  Discrete flight-computer commands are latched for a different reason (they must stay frozen
  over the whole macro window — see `SESSION_NOTES.md` §8), not because of a loop.

Distinct C++ interfaces mean wiring mistakes are compile-time or build-time errors, not runtime
asserts. A `Latched<T>` is the *only* legal way to break an instantaneous loop; a topological
cycle without one is a hard build error.

### 2.3 The three-pass pipeline (runs **inside** the odeint functor, not the driver loop)

On **every** derivative call — including odeint's speculative trial steps:

1. **Pass 1:** framework zeros every registered `Accumulator`.
2. **Pass 2+3 (fused):** each model's `eval(ctx, sv)` runs in **topological order**. It reads its
   ODE span, upstream signals, and latched (previous-window) values; publishes/adds its own
   signals; and (if it has state) writes its own `dxdt` slice.

Models are **pure functions** of (ODE state, resolved signals, latches, t). They never mutate ODE
state directly. That purity is what makes odeint's rejected trial steps harmless.

### 2.4 Time orchestration (the driver, per 10 ms macro window)

Per window of `dt_macro` (default 0.01 s = 100 Hz):

1. **Discrete tick at window start:** flight computer runs its `IFlightSoftware::update(t,
   sensors)` — sensors are read from registered readers; commands are `propose()`d onto command
   latches.
2. **Adaptive integration** across the window via `odeint::integrate_adaptive` with a **fresh
   stepper built per window** (RK54 Dopri5). It clamps the final step to land exactly on the
   window end; the final time/step count come from the **observer** callback (start_time/dt are
   passed by value — do not read stale locals).
3. **Window end:** `commit()` all latches (`next → current`). A command proposed at `t` therefore
   affects the window `(t, t+dt]`.
4. **Event check** (after the commit, on a refreshed copy of the live state). A jettison marks
   the stage's models inactive (state slots are kept, so spans stay valid), re-sorts the
   surviving subgraph, and the next window's fresh stepper discards all adaptive history —
   that's what makes the discontinuity safe.
5. Telemetry snapshot at the configured rate.

**Hard rules (do not regress, see `SESSION_NOTES.md` §6–§8 for the full history):**
- `x_live_` (driver-owned) is the single source of truth for ODE state; the core's `x_` is a
  mirror refreshed on every derivative call and before event triggers.
- Latch commit is at window **end**, not start.
- Latched `read()` returns `current_` only; there is no `discard()` hook and there is no
  t-tracking in the functor. The 3-part contract (read-from-current-only / temporal freezing /
  clean window boundaries) is closed — don't reopen it.
- If you ever hand-roll stepping: use the **3-arg** `try_step(sys, x, t, dt)`. The 5-arg form is
  broken on Boost 1.92 and rejects every step.

---

## 3. Task: make a new simulation

A "simulation" = a vehicle TOML config + (optionally) new model types/flight software. For a
vehicle built from existing types you only touch the config.

### 3.1 Config anatomy (`config/vehicle.toml` is the reference — copy it)

```toml
[sim_params]                       # required: t_end; rest have defaults
t_end        = 120.0
dt_macro     = 0.01               # discrete tick period (100 Hz)
dt_max       = 1e-3               # adaptive substep ceiling (must be <= dt_macro)
eps_abs      = 1e-9
eps_rel      = 1e-9
telemetry_rate = 10.0             # Hz; 0 disables telemetry
telemetry_file = "telemetry.csv"

[models.<id>]                     # one table per model instance
type   = "PropellantTank"         # registered type name (see --list-types)
stage  = "stage1"                 # "vehicle" for airframe; stages are jettison units
# ... type-specific numeric params (unknown keys are hard errors with file:line)

[[connections]]                   # one entry per signal
signal    = "engine1.force_z"     # "<producer_id>.<producer_port>"
kind      = "Accumulator"         # Signal | Accumulator | Latched
producers = ["engine1"]           # 1 for Signal/Latched, N for Accumulator
consumers = ["vehicle"]
to        = "total_force_z"       # optional: consumer-side port name (port alias)

[events.<id>]                     # discrete events
type  = "fixed_time"              # only supported trigger today
stage = "stage1"
t     = 60.0

[flight_computer]                 # exactly one per vehicle (omit for open-loop sims)
model    = "flightcomp"
software = "VehicleFlightSoftware" # registered software name
sensors  = ["gimbal_act.position"] # "<model>.<port>" names (reader table)
commands = ["throttle_act.cmd", "gimbal_act.cmd"]  # proposer names
[flight_computer.params]          # passed to the software's set_param(); unknown names error
throttle_setpoint = 1.0
```

Validation is strict at load time (unknown sections/keys/ports/types → error with file:line).
`--check` runs load + build + topology resolution without integrating — use it to iterate on
configs quickly.

### 3.2 Wiring gotchas

- **Port names need not match across the wire.** The consumer's port is named via `to` (or must
  match the producer port name if `to` is omitted). E.g. an engine's `force_z` feeds the
  vehicle's `total_force_z` accumulator. Ports can also declare **aliases** in the registry
  (e.g. the vehicle's `total_mass` port also matches `mass` and `total_mass_contrib`).
- **Accumulators: the consumer owns the slot.** If one consumer has multiple producers (two tanks
  → vehicle total mass), the factory re-binds *all producer views* to the consumer's slot. You
  just declare each producer→consumer connection; don't try to wire a "shared sum" yourself.
- **Break every instantaneous loop with `Latched`.** Engine→tank→engine (thrust/flow → mass →
  pressure → thrust) would be a cycle; the tank's `ullage_pressure_latched` (1-step delay) is
  what makes it a DAG. If `resolve_topology` reports a cycle, add a latch somewhere on it.
- **`[flight_computer].sensors` / `commands` are namespaced reader/proposer names**, not signal
  names: `"<model_id>.<registered_name>"` (e.g. `gimbal_act.position`, `throttle_act.cmd`).
- **One flight computer, one software instance, per vehicle.** The software may internally run
  many control loops (throttle + gimbal + mode logic) and command many actuators in one tick.
  Do not model "multiple flight software" as multiple instances.

### 3.3 Sanity checklist after a run

- `vz` rises monotonically during burn; **exactly constant** after jettison (coast).
- Tank masses drain monotonically; frozen after their stage jettisons.
- No NaN in `telemetry.csv` (the core logs `spdlog::error` immediately on non-finite derivatives).
- The topology report's Pass 2 order is a valid DAG; a consumer appears after all its producers.
- `td_sim_example` still matches `td_sim` to ~4e-5 absolute (same physics, different build path).

---

## 4. Task: add a new model type

This is the design's primary extensibility path, and it is deliberately a **three-step ritual**:

### Step 1 — write the model struct (in `include/tdsim/models.hpp`, or its own header)

```cpp
struct MyModel : StatefulModel<2> {          // Dim = # of integrated doubles (0 = algebraic-only)
    static constexpr std::size_t DIM = 2;
    StateDescriptor desc{"mytype", {"a", "b"}};   // telemetry group name + slot names

    // Declared signal views — members, wired by the builder before the sim starts.
    Signal<double>      out_signal;         // I publish this
    Accumulator<double> in_sum;             // I read this (cleared every derivative call)
    Latched<double>     cmd_in;             // I read the last-committed command

    double some_param = 0.0;                // settable from [models.<id>] params

    void eval(const SimContext& ctx, const StateView& sv) override {
        // sv.state / sv.dxdt are MY slice ONLY (positioned by the framework).
        double c = cmd_in.read();           // frozen over the whole macro window
        out_signal.publish( /* ... */ );
        sv.dxdt[0] = sv.state[0] + c;       // Pass 3: write my derivatives last
        sv.dxdt[1] = -sv.state[1];
    }
};
```

Rules:
- `eval()` is the **only** entry point and must be pure: function of its ODE slice, upstream
  signals, latches, `ctx.t`, and its own (immutable) parameters. No mutation of shared state.
- Read-only access to *other* models' ODE state: declare an `OdeReader` member and call
  `SimulationBuilder::wire_ode(...)` — but for the common cases, signals/accumulators are the
  intended channels.
- `StatefulModel<Dim>` for different `Dim` are **unrelated types** (no shared base). That's
  intentional (keeps the hot path a single virtual call); it just means the factory's construct
  pattern below is the only way to type-erase them.
- If the model is a *consumer* of an accumulator it must be listed in that connection's
  `consumers`; if it *feeds* one, in `producers`. The framework clears accumulators in Pass 1,
  so a consumer that runs after all producers (enforced by topology) sees the full sum.

### Step 2 — register the type (one block in `src/factory.cpp`)

Add `register_mytype()` next to the existing `register_tank()` / `register_engine()` (which are
the patterns to copy verbatim) and call it from `register_builtin_types()`. The full surface:

```cpp
void register_mytype() {
    using M = MyModel;
    ModelType mt;
    mt.state_dim = M::DIM;

    // Lifetime pattern — the slot is the SOLE owner of the concrete model.
    // Do NOT do `auto m = make_unique<M>(); return make_unique<ConcreteSlot<...>>(move(m), &m->desc);`
    // — m dies at lambda exit and desc dangles.
    mt.construct = []() {
        auto slot = std::make_unique<ConcreteSlot<M::DIM, M>>();
        slot->m_model = std::make_unique<M>();
        slot->desc    = &slot->m_model->desc;   // stable: lives inside the owned model
        return slot;
    };

    // Every signal port the type exposes. name = config-facing port name;
    // aliases = alternate config-facing names; produced = this model is the writer side.
    mt.ports = {
        {"out_signal", {}, SignalKind::Signal, true,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->out_signal.bind(cur, "");
         }},
        {"in_sum", {}, SignalKind::Accumulator, false,
         [](ModelSlot& b, double* cur, double*) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->in_sum.bind(cur, "");
         }},
        {"cmd_in", {}, SignalKind::Latched, false,      // Latched ports need cur AND nxt
         [](ModelSlot& b, double* cur, double* nxt) {
             auto* m = static_cast<M*>(static_cast<void*>(b.model()));
             m->cmd_in.bind(cur, nxt, "");
         }},
    };

    // Numeric params accepted in [models.<id>]; unknown config keys are rejected via this map.
    mt.params = {
        {"some_param", [](ModelSlot& b, double v) {
            auto* m = static_cast<M*>(static_cast<void*>(b.model())); m->some_param = v;
        }},
    };

    // Optional hooks (leave unset = default):
    mt.initial_state  = [](ModelSlot& b) {          // default: all zeros
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        return std::vector<double>{ /* a0, b0 */ };
    };
    mt.post_configure = [](ModelSlot& b) {          // runs AFTER params + port binding;
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        m->cmd_in.init(0.0);                        // use for latch init from derived values
    };
    mt.accumulators   = [](ModelSlot& b, std::vector<Accumulator<double>*>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(&m->in_sum);                  // every accumulator this instance clears in Pass 1
    };
    mt.readers        = [](ModelSlot& b,
        const std::function<void(const std::string&, std::function<double()>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("out_signal", [m]() { return m->out_signal.read(); });
        // NOTE: registered readers ARE the telemetry columns (namespaced "<model_id>.<name>").
        // Don't re-log an ODE state as a reader — it already appears as "<id>.<slot_name>".
    };
    mt.proposers      = [](ModelSlot& b,
        const std::function<void(const std::string&, std::function<void(double)>)>& add) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        add("cmd_in", [m](double v) { m->cmd_in.propose(v); });
        // proposer names ("<id>.cmd_in") are what [flight_computer].commands references.
    };
    mt.collect_latches = [](ModelSlot& b, std::vector<Latched<double>*>& out) {
        auto* m = static_cast<M*>(static_cast<void*>(b.model()));
        out.push_back(&m->cmd_in);                  // latches the driver commits each window
    };
    VehicleFactory::register_model_type("MyModel", std::move(mt));
}
```

Notes:
- Every lambda is written for exactly one concrete type, so the
  `static_cast<M*>(static_cast<void*>(b.model()))` downcast is safe by construction.
- Forgetting `collect_latches` on a latch that carries a flight-computer command means the
  command never commits (stays at its init value). Forgetting `accumulators` means a stale sum
  leaks into the next derivative call.
- New event trigger types: add the parse branch in `src/config.cpp` (`parse_events`) and the
  dispatch in `examples/td_sim.cpp` (which currently only accepts `fixed_time`).

### Step 3 — wire it in the TOML

```toml
[models.my_model]
type = "MyModel"
stage = "stage1"
some_param = 42.0

[[connections]]
signal = "other.producer_port"
kind   = "Accumulator"
producers = ["other", "other2"]
consumers = ["my_model"]
to       = "in_sum"

[[connections]]
signal = "my_model.out_signal"
kind   = "Signal"
producers = ["my_model"]
consumers = ["somedownstream"]
```

Then: `./build/td_sim --list-types` shows the new type and its ports/params; `--check` validates
wiring; run it and confirm the topology report shows your model in a sensible position.

### Adding a new flight software (same shape, smaller)

1. Derive `IFlightSoftware` (see `VehicleFlightSoftware` in
   `include/tdsim/flight_software.hpp`): implement `update(t, sensors)` (pure; returns a
   `map<name, double>` of commands), `name()`, and — if it takes config params — `set_param()`
   (throw on unknown names; the loader's strictness depends on it).
2. Register it in the `software_registry()` map in `examples/td_sim.cpp`.
3. Point `[flight_computer].software` at it; list its `sensors`/`commands` (namespaced reader/
   proposer names) and params in the TOML.
Remember the software only sees the sensor snapshot you list and `t` — it has no access to the
continuous state vector.

---

## 5. Repository map

```
td_sim3/
├── CMakeLists.txt           # C++20; tdsim (static lib) + td_sim + td_sim_example
├── config/vehicle.toml      # THE reference vehicle (TOML is the single source of truth)
├── include/tdsim/
│   ├── simparams.hpp        # SimParams (integrator/telemetry knobs); leaf header
│   ├── signal.hpp           # Signal / Accumulator / Latched + SlotStore
│   ├── model.hpp            # SimContext, StateView, OdeReader, StatefulModel<Dim>,
│   │                        #   ModelSlot / ConcreteSlot (type erasure; slot owns the model)
│   ├── topology.hpp         # SimulationBuilder: contract validation, Kahn sort, DAG edges,
│   │                        #   jettison re-sort, Engineering Data Flow Report
│   ├── system.hpp           # SimulationCore (3-pass pipeline, reader/proposer tables) +
│   │                        #   SystemFunctor (copyable odeint wrapper sharing the core)
│   ├── spec.hpp             # VehicleSpec (typed IR: models/connections/events/FC) + load_spec()
│   ├── telemetry.hpp        # TelemetryLogger: named columns → CSV
│   ├── flight_software.hpp  # IFlightSoftware + VehicleFlightSoftware (example: throttle + PID)
│   ├── models.hpp           # reference models: RigidBodyVehicle, PropellantTank, LiquidEngine,
│   │                        #   FirstOrderActuator, PressureSensor  ← templates for new models
│   └── factory.hpp          # ModelType registry record, VehicleFactory (build_from_spec)
├── src/
│   ├── config.cpp           # TOML → VehicleSpec, strict file:line validation
│   ├── factory.cpp          # registry + generic 7-step build pipeline + register_builtin_types
│   │                        #   ← the place to add new model-type registrations
│   ├── topology.cpp         # validation / sort / report / jettison re-sort impl
│   ├── simulator.cpp        # driver loop: tick → integrate_adaptive → commit → events → telemetry
│   └── telemetry.cpp        # CSV open/record/close
└── examples/
    ├── td_sim.cpp           # CLI: load TOML, register types, run; --check / --list-types
    └── main.cpp             # td_sim_example: programmatic build (test oracle)
```

`SESSION_NOTES.md` (repo root) is the design contract + decisions log + a catalogue of
integration bugs already hit and fixed (esp. §6 odeint/Boost 1.92 gotchas and §8 the closed
3-part latch contract). Read §6–§8 before touching the driver, integrator, or signal layer.

---

## 6. Build, debug, and verification

- **Build:** `cmake -S . -B build && cmake --build build` (Release by default; `build-dbg/` is a
  pre-existing Debug tree). Dependencies: system **Boost** (header-only odeint), **spdlog**,
  **toml++** (`find_package(tomlplusplus)`). CMake 4.4 prints a harmless FindBoost policy
  warning — ignore it.
- **Debugging:**
  - Topology/wiring problems fail **at build time** with model/param/port names in the error.
  - Config typos fail **at load time** with `file:line` context.
  - Runtime NaN/Inf derivatives log `spdlog::error [CORE] non-finite derivative ...` immediately.
  - Per-window step counts log at `spdlog::debug`; note this spdlog build does **not** honor
    `SPDLOG_ACTIVE_LEVEL` — set the level in code to see debug lines.
  - A step-size collapse (pathological RHS) throws via odeint's `failed_step_checker` instead of
    hanging.
- **Verification ritual after any change:**
  1. `./build/td_sim --check config/vehicle.toml` (config + topology OK)
  2. `./build/td_sim config/vehicle.toml` (runs; check final `vz`/`pz`/masses against §1)
  3. `./build/td_sim_example` (must match the TOML run's physics to ~4e-5)
  4. Grep `telemetry.csv` for NaN; eyebank the burn/coast transition at the event time.
- **Tests:** no test suite yet (the commented block at the bottom of `CMakeLists.txt` is the
  placeholder). The `td_sim_example` vs `td_sim` parity check above is the de-facto regression
  test.

---

## 7. Known open items (as of this writing)

- `tests/` directory: still empty (cycle detection, accumulator warning, latch commit-per-window,
  mini integration smoke test).
- Only `fixed_time` event triggers exist; a `tank_empty` trigger would read state via the
  `SystemFunctor&` the trigger receives (the driver refreshes state before triggers run).
- `RigidBodyVehicle` is 3-DOF translational (px/py/pz + vx/vy/vz); 6-DOF attitude is not there.
- Flight-software params not handled by `set_param()` are swallowed by the CLI's try/catch
  (currently only `dt_macro`/PID gains/setpoints are routed) — route new params explicitly.
- Telemetry is synchronous CSV (fine ≤ ~1 kHz); async writer is a documented option, not needed.
