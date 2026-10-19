# td_sim3 — Session Notes (for future sessions)

**Purpose of this file:** If a session dies mid-implementation, a future session can read this and pick up without re-deriving the design. Last updated: after the multi-rate latch cadence session — see §11 (Multi-rate discrete subsystems) for the LCM base-tick design.

**Habit (user-mandated):** commit finished, verified chunks to git as work progresses, so a dead session loses at most one chunk. Baseline commit of the broken TOML session state is in git history.

## 11. MULTI-RATE DISCRETE SUBSYSTEMS (25 Hz sensor next to 100 Hz FC) — DONE

**Goal:** support discrete components at different rates (e.g. a 25 Hz digital sensor
next to a 100 Hz flight computer) without a general RTOS scheduler.

**Decision: divisible-only clock tree on a shared base tick (LCM), NOT a
priority/deadline scheduler and NOT variable macro windows.**

The sim runs on ONE base tick. Every discrete subsystem (FC, multi-rate sensor
latch) AND the macro window must tick at an integer divisor of the base. The
base rate is the **LCM** (least common multiple) of all discrete rates and the
macro-window rate: `base_hz = LCM(window_hz, all_rates)`. Each subsystem's
`period_ticks = base_hz / rate` (an integer ≥ 1 by construction). "Is it tick
time?" is a pure integer branch `k % period_ticks == 0` — no floating-point
drift, no epsilon tests. The driver's base tick period is `1/base_hz` (finer
than or equal to `dt_macro`); when no multi-rate subsystems exist, `base_hz=0`
and the driver steps at `1/dt_macro` (original single-rate behavior, byte-
identical).

**Why LCM, not GCD:** the base must be ≥ the fastest rate (otherwise that
subsystem couldn't tick at its rate). LCM({100, 25}) = 100: the FC (100 Hz)
ticks every base tick (period 1), the sensor (25 Hz) ticks every 4th (period
4). GCD would give 25 Hz, which is coarser than the FC's 100 Hz — impossible.

**Implementation (3 commits):**
1. `Latched<T>::period_ticks` (default 1) + driver commit gate `k % period == 0`
   + `collect_latches` hook returns `LatchRef{latch, period_ticks}`. All existing
   latches report period 1 → reference telemetry byte-identical.
2. `VehicleFactory::resolve_base_hz` (LCM) + `resolve_periods` (per-rate
   `period_ticks`, dt_max validation). `PressureSensor.reading` converted from
   `Signal` to `Latched` (a sensor output is a ZOH at its sample rate); new
   `rate` param (Hz) sets the latch period. `FlightComputer` gains `period_ticks`
   + `dt`; driver ticks the FC only when `k % fc.period_ticks == 0`.
3. Tests: LCM resolver, dt_max rejection, incommensurate-rate handling, Latched
   ZOH hold pattern, period-1 passthrough.

**Key invariants preserved:**
- `SystemFunctor` remains a pure function of `(x, t)`; no tick indices or
  schedules inside the functor. Scheduling lives in the driver.
- Discrete state frozen across each integration window (unchanged).
- `Latched<T>` remains the discrete-to-continuous bridge; `period_ticks` is
  the only new field.
- Jettison handling unchanged (latches need no special action).
- Reference vehicle telemetry byte-identical (md5 `7151b346…` verified before
  and after all three commits).

**Open / future:**
- Telemetry rate is capped at the base rate (can't sample faster than the
  driver's tick). A 100 Hz telemetry + 25 Hz sensor → base = 100 Hz, telemetry
  at 100 Hz is fine. A 1000 Hz sensor would push the base to 1000 Hz and make
  the sim 10× more expensive — the dt_max validation catches pathological
  cases.
- If a consumer needs sample-age (e.g. differentiating a slow sensor), that's
  an opt-in per-consumer feature, not a bus-wide change.
- Variable-window scheduling (stepping `integrate_adaptive` to the next
  discrete tick) remains a deferred contingency for incommensurate rates; the
  divisible-only contract rejects those at build time instead.

A best-practices review (RAII/SOLID/etc.) was done over the whole tree and the top 3 findings were fixed. No physics/behavior change — verified identical reference output (vz=976.0322, pz=85537.79, tanks frozen 12445.84 kg @ t=120, jettison t=60, 36,000 steps / 12,000 windows, no NaN; TOML path vs programmatic oracle match on all 20 telemetry columns).

1. **TelemetryLogger is now RAII + exception-safe.** The `open()`/`close()` public API is gone. It now opens the CSV in its constructor `(columns, path)` and flushes+closes in the destructor, so the file is flushed on EVERY exit path — including when `odeint::integrate_adaptive` throws (`failed_step_checker`) and unwinds out of `Simulator::run()`. Previously a throw skipped `close()` and the final row. The logger is now explicitly move-only (owns the file handle). `Simulator::build_telemetry_columns()` only constructs it when telemetry is enabled (empty file / rate<=0 → default-constructed no-file, `record()` is a guarded no-op); the driver no longer tracks a `telemetry_on` flag or calls `close()`.
2. **`Simulator` is now explicitly non-copyable AND non-movable** (all four special members `= delete` with a comment). Before, only the copy was implicitly deleted (it holds a `SystemFunctor&`); the move status was implicit/undocumented.
3. **Dead code / misleading API removed:** (a) dropped the `#include "tdsim/models.hpp" // TEMP DEBUG` from `src/simulator.cpp` (concrete-type dependency in the driver, no remaining use); (b) removed `SimContext::active` — the core skips a jettisoned model's `eval()` entirely, so no model could ever see `false` and it was always constructed `true`. `SimContext` is now just `{ double t }`; the construction site in `system.hpp` updated to `SimContext ctx{t}`.

**Still OPEN from the review (items 4+ not yet done):** move the flight-software registry out of `examples/td_sim.cpp` into the library (mirror the `VehicleFactory` registry pattern) so `--list-types` and registration have one home; `SlotStore` has no bound check on its magic 1024 capacity (latent heap overflow if a config declares >1024 port-slots); `emit_report()` retains `report_` (mutable, never consumed) and writes to stdout; `OdeReader`/`wire_ode` currently have no model-side consumers; `models.hpp` still includes `flight_software.hpp` with no apparent use; telemetry **column order is non-deterministic** (reader table is an `unordered_map`) — sort reader names when building columns if stable/diffable CSV is wanted.

## 9. RESUME: where we are right now (read this first)

**Goal:** make the TOML config the single source of truth for the example vehicle, and un-bake the example from the core, because this is a TEMPLATE project:
- The core simulation routines (signal/model/topology/system/simulator) must contain NO vehicle knowledge. They don't today — vehicle knowledge lives only in `models.hpp`, `factory.cpp` (registry), and `flight_software.hpp`. Keep it that way.
- Future models will be added by the user: write the `StatefulModel<Dim>` struct, add one `register_model_type()` block, wire it in TOML. The registry design already supports this; don't redesign it.
- The `VehicleFactory::build_example_vehicle()` programmatic path and `examples/main.cpp` (compile-time params) are the "baked-in" part. Plan: delete them; the reference vehicle lives only in the example TOML config (moved to `examples/`), and the TOML path is the single build path.

**What the previous (failed) session left behind — the tree DOES NOT COMPILE:**
1. `src/factory.cpp`: every `mt.construct` lambda does `std::unique_ptr<StatefulModel<0>>(std::move(m))` where `m` is `unique_ptr<Concrete>`. This fails: `StatefulModel<Dim>` for different `Dim` are UNRELATED types, so there is no implicit `unique_ptr` conversion. Fix: declare a small local `struct Holder : StatefulModel<0> { T* p; ~Holder() { delete p; } };` per type and wrap the raw pointer (or make construct return `unique_ptr<ModelSlot>` + raw lifetime handled by the bag). Keep it simple and local.
2. `examples/td_sim.cpp`: includes nonexistent `tdsim/config.hpp` (real header is `tdsim/spec.hpp`, `load_spec` lives in `src/config.cpp` — add the declaration to spec.hpp or a thin config.hpp), references nonexistent `FlightSoftware` base (it's `IFlightSoftware`), calls nonexistent `sim.add_event(...)` (real API: `add_jettison_event(JettisonEvent{stage, trigger, fired})`), and sets up Boost.Log (drop it; the project uses spdlog only).
3. `IFlightSoftware` needs `virtual void set_param(const std::string&, double) {}` (default no-op; `VehicleFlightSoftware` overrides) so the CLI can apply `[flight_computer.params]` generically. Also `VehicleFlightSoftware`'s `dt` must be set by the driver from `params_.dt_macro` (it's needed for the PID derivative term and currently has no config path other than public member).
4. `JettisonEvent` (simulator.hpp) has no stage-id field on the trigger side... actually it DOES (`stage_id`). The CLI must fill it from `EventSpec.stage`.
5. `new_notes.md` proposes pushing commit/discard t-tracking into SystemFunctor. This was ALREADY REJECTED in §8 (solidified 3-part contract: read-from-current-only, temporal freezing, clean window boundaries). Do NOT implement new_notes.md.

**Working definition of done:** `./build/td_sim examples/<vehicle.toml>` runs the 120 s flight, jettisons stage1 at t=60 s, writes telemetry.csv with sane physics (vz → ~976 m/s at burnout, constant in coast; tanks drain monotonically; no NaN), and `--list-types` works. Core headers remain vehicle-agnostic.

**Build:** `cd td_sim3 && cmake -S . -B build && cmake --build build`. Note: CMake 4.4 warns about FindBoost policy CMP0167 — harmless, ignore (odeint is header-only; `Boost::boost` is an INTERFACE alias).

---

## 9a. RESUME: where we are right now (UPDATED — the TOML path now WORKS)

**Status: DONE.** The TOML config is the single source of truth. `./build/td_sim config/vehicle.toml` runs the 120 s flight, jettisons stage1 at t=60 s, and writes telemetry.csv with correct physics. The programmatic path (`td_sim_example`) also works (it routes through `build_from_spec` with an in-memory spec).

**Physics (verified, t=120 s):** vz = 976.03 m/s, pz = 85,538 m, fuel = oxi = 12,446 kg (frozen after jettison). Burnout at t=60: vz = 976 m/s, thrust ≈ 752 kN (pressure-flow factor × thrust). After jettison: vz constant, pz linear, fuel frozen — correct ballistic coast.

**Bugs found & fixed this session (in order):**
1. **`build_from_spec` was missing model wiring** (no `build_from_spec` existed — only `build_example_vehicle` which had hardcoded params). Added `build_from_spec` that takes a `VehicleSpec` and runs the full 7-step pipeline. The `build_example_vehicle` now builds an in-memory spec and routes through `build_from_spec` (guarantees the two paths can't drift).
2. **`ConcreteSlot` lifetime bug:** the `mt.construct` lambdas did `std::make_unique<ConcreteSlot<...>>(std::move(m))` then returned a pointer to `m->desc`, but `m` was destroyed at end of scope → dangling descriptor. Fixed: construct the `ConcreteSlot` first (it owns the model via `unique_ptr<M>`), then set `slot->desc = &slot->m_model->desc` (pointer into the owned model, stable for the slot's lifetime). Added a default constructor to `ConcreteSlot` for the two-stage init.
3. **`ModelBag` no longer holds model/slot pointers** — it's now metadata-only (id/type/stage/dim). The `ConcreteSlot` is the sole owner of the concrete model. `find_slot(bag, id)` resolves a bag entry to its core slot. This satisfies plan item #1 (ConcreteSlot sole owner).
4. **Accumulator multi-producer wiring bug (THE big one):** when multiple producers feed one consumer Accumulator (e.g. `fuel_tank.total_mass_contrib` + `oxi_tank.total_mass_contrib` → `vehicle.total_mass`), the old code bound the consumer's slot to the LAST producer's slot, orphaning earlier producers (each producer kept its own slot). The engine's `force_z.add(T)` went into the engine's own slot, but the vehicle read from a different slot → `total_force_z = 0` → no thrust → vz ≈ 2540 (gravity-only fall). Fixed: for Accumulator connections, the CONSUMER's slot is canonical; all producer ports are re-bound to point at the consumer's slot so `add()` calls from all producers accumulate into the same slot the consumer reads. For `Signal`/`Latched`, the producer owns the slot (existing behavior).
5. **`build_example_vehicle` missing `to_port`:** the hardcoded connections didn't set `cs.to_port`, so port names had to match exactly between producer and consumer. But the port names DON'T match (e.g. engine's `force_z` → vehicle's `total_force_z`). Added `to_port` to every connection. The TOML config already had `to_port` (the `to` field), so only the programmatic path was affected.
6. **Telemetry column naming:** ODE columns used `desc.name` (type name, not unique — two tanks both produce "tank.mass"). Changed to `slot->id` (unique). Removed the redundant `mass` signal reader from the tank (it's an ODE state, already logged as an ODE column). Net: unique column names, no duplicates.
7. **`VehicleFlightSoftware::dt` config path:** added `dt_macro` to `set_param()` so the TOML `[flight_computer.params]` can override it; the driver seeds `dt` from `params_.dt_macro` before applying config params.

**Key files modified:**
- `src/factory.cpp` — added `build_from_spec`, fixed `ConcreteSlot` lifetime, fixed Accumulator binding, added `to_port` to `build_example_vehicle` connections, removed redundant mass reader.
- `include/tdsim/factory.hpp` — `ModelBag` is metadata-only; added `find_slot()`; `build_example_vehicle` now routes through `build_from_spec`.
- `include/tdsim/model.hpp` — `ConcreteSlot` default constructor.
- `include/tdsim/flight_software.hpp` — `dt_macro` param in `set_param()`.
- `src/simulator.cpp` — ODE telemetry columns use `slot->id`.
- `examples/td_sim.cpp` — `build_flight_software` takes `dt_macro`.

**Verified:** both `td_sim` (TOML) and `td_sim_example` (programmatic) produce IDENTICAL physics (vz=976.03, pz=85538, fuel=12446 at t=120). 36,000 adaptive steps over 12,000 macro windows. No NaN. `--list-types` works.

**Remaining (from §7):**
- The `--check` flag was NOT implemented (the plan mentioned it but it's not in the current td_sim.cpp). If needed, add a `--check` that validates the spec without running (topology validation only).
- 6-DOF vehicle (plan item #4).

**Test suite (done, doctest):** `tests/` now has `main.cpp`, `test_signals.cpp`, `test_topology.cpp`, `test_factory.cpp` + `registry_setup.hpp`. 23 test cases / 3675 assertions, all green via `ctest` (target `tdsim_tests`); CMake `enable_testing()` section is active.
- `test_signals.cpp`: Signal / Accumulator / Latched semantics + the `SlotStore` bound-check throw.
- `test_topology.cpp`: duplicate model id, unknown wire endpoint, Signal/Latched two-producer errors, 0-producer Accumulator is a warning, consumer-after-all-producers ordering, Latched creates no edge (loop breaker), algebraic-cycle detection (names the models), duplicate-edge dedup (no false cycle), jettison re-sort (incl. a cycle resolved by jettison).
- `test_factory.cpp`: spec validation (unknown type / unknown param / nonexistent port / kind mismatch), the **multi-producer Accumulator regression** (two tanks → `total_mass` reads the sum 3000, `d(vz)/dt = -g` with full mass — locks in the §9a.4 fix), and the **TOML-vs-programmatic oracle** (runs `config/vehicle.toml` and `build_example_vehicle()` to 120 s, compares every telemetry row).
- **Two non-obvious gotchas hit while writing the oracle test:** (1) telemetry column order is nondeterministic (`unordered_map` — see §10 / open items), so the two CSVs are compared per row as `time + multiset(field values)`, not as strings; tighten to strict comparison once columns are sorted. (2) the `TelemetryWriter`'s `ofstream` is buffered — read the CSV only AFTER the `Simulator` is destroyed (file closed), else you see a partially-flushed file. Both simulations therefore run in nested scopes.
- **Registry ordering gotcha:** `build_from_spec` needs `VehicleFactory::register_builtin_types()` to have run first. doctest orders test cases by translation unit, so a static init in `tests/main.cpp` alone is NOT guaranteed to run before `test_factory.cpp` cases — hence the idempotent `ensure_registry()` in `tests/registry_setup.hpp`, called at the top of every factory test case.
- The TOML config's `throttle_setpoint` param is read but the VehicleFlightSoftware's `set_param` doesn't handle it — only `dt_macro` is handled. The `throttle_setpoint` and `gimbal_*` params from the TOML are silently ignored (the try/catch in `build_flight_software` swallows them). This is acceptable for now (the example uses hardcoded values in the driver), but a proper implementation would route them through `set_param`.

## 1. What we're building

Initial C++20 template for a time-domain rocket simulation (`boost::numeric::odeint`), per `requirements.md`. Key requirement pressures:

- Vehicle topology **unknown until runtime** (built from input file: models + params + signal wiring + events + telemetry config).
- Models must "talk" (many-to-many engines↔tanks, sensors read anything, vehicle acceleration is a global) **without circular dependency loops or corrupting odeint's stepper state**.
- **One flight computer per vehicle, running one flight software instance.** The software may internally run multiple control loops (throttle, gimbal, mode logic) and command multiple actuators in a single tick. Multiple flight software *versions* may exist (swappable via config), but a given vehicle contains exactly one.
- Flight computers + flight software run at **fixed discrete rate (100 Hz)**; must be invisible to odeint's speculative trial steps.
- Jettison events (stages/engines) = mid-run discontinuities.
- Easy telemetry of everything to file.
- **Extensibility is a first-class goal**: adding a new model kind must be small and intuitive for a developer.

## 2. Input documents

- `requirements.md` — the requirements (rocket model example list, odeint, 100 Hz, config file, jettison).
- `potential_sim1.md` — "Topological Multi-Pass Registry": best ideas = 3-pass pipeline (Clear → topo Evaluate → Populate dxdt), algebraic vs differential split, dual-domain temporal isolator. Flaw: hard-coded global `AlgebraicPorts` struct (anti-extensible).
- `potential_sim2.md` — "Flat Vector Bridge": best ideas = `std::span` views, derivative **summation** for many-to-many, **stepper reset/rebuild on jettison** (only doc that got this right), ID-based connection map. Flaw: models doing index lookups into the raw vector.
- `potential_sim3.md` — OOP Vehicle/Stage/Component: best ideas = the 5-layer conceptual model (ODE state / computed / discrete / commands / telemetry), runtime factory. **Flawed**: `restoreODEState(x)` mutates model members every functor call; raw `Tank*` pointers; `std::any` state; fixed-dt `do_step` loop.
- `potential_sim4.md` — Data bus/blackboard: best ideas = named-signal decoupling, ring-buffer async telemetry, concepts + strategy for `IFlightSoftware`. Flaw: untyped string-keyed store, no ordering story.

**Requirement clarification (from user, post-implementation):** `requirements.md` says "Flight Computer can run many different flight software models." Corrected interpretation: a vehicle contains **one** flight computer running **one** flight software instance at a time. The flight software may internally run multiple control loops (throttle, gimbal, mode logic) and command multiple actuators. "Multiple flight software" means multiple *versions* exist (selectable via config), not multiple instances per vehicle. Do NOT model this as multiple `IFlightSoftware` instances per vehicle.

**Synthesis decided** (agreed with user across 4 rounds): sim3 layers + sim2 flat-state/spans/summation + sim1 3-pass pipeline & latching + sim4 named signals & concepts. Plus three user-mandated refinements, then the user supplied a formal spec (their message starting "1. Unified Architectural Specification") which is now the contract.

## 3. THE AGREED ARCHITECTURE (the contract — do not redesign without asking)

### 3.1 State
- One flat `std::vector<double>` ODE state (only truly integrated vars: tank masses, vehicle pos/vel/attitude, etc.) + one flat `dxdt`.
- Models get `std::span` views at build time; **no index math in model code, ever**.
- Algebraic quantities (pressure, thrust, accel, mass flows) are NOT integrated — recomputed every functor call via the 3-pass pipeline.

### 3.2 Signals (see `include/tdsim/signal.hpp`) — the only model↔model channel
| Type | Producers | API | Topology effect | Jettison re-sort | Telemetry tag |
|---|---|---|---|---|---|
| `Signal<T>` | exactly 1 (else hard build error) | `publish()` / `read()` | DAG edge producer→each consumer | remove edge if either endpoint inactive; re-sort surviving subgraph | `[Instantaneous]` |
| `Accumulator<T>` | 0+ (0 = **warning**, not error) | `add()` (+=) / `read()` / framework `clear()` | consumer must run after **ALL** producers (edge from each producer); producers order-free among themselves | drop inactive producers; **no** re-sort | `[Accumulated]` |
| `Latched<T>` | exactly 1 | `propose()` / `read()` (reads last **committed** value) / framework `init()`, `commit()` | **no edge** — this is the sanctioned algebraic-loop breaker (1-step delay) | nothing | `[Latched (1-Step Delay)]` |

Distinct interfaces = misuse is a compile error, not a runtime assert.

### 3.3 Execution (per odeint functor call)
- **Pass 1**: framework zeros every `Accumulator` slot.
- **Pass 2+3 fused**: models run in topological order; each `eval()` reads ODE spans + resolved upstream signals + latched values, `publish`/`add`s its provided signals, and (if it has state) writes its own `sv.dxdt` slice.
- **The 3 passes live INSIDE the system functor** (`operator()(x, dxdt, t)`) — odeint calls it many times per accepted step (RK stages, error control, rejections); running passes once per driver-loop iteration is the bug the user's §4 draft had and we fixed.
- Models are pure functions of (x, algebraics, latches, t). Nothing mutable → speculative steps can't corrupt anything.

### 3.4 Time orchestration (driver, per macro window of 10 ms) — AS IMPLEMENTED
1. Discrete tick (exactly at window start `t`): run `IFlightSoftware` (reads last-**committed** sensor latches) → `propose()` command latches.
2. Adaptive stepping **bounded by the window end**, driven by **`integrate_adaptive`** (see §6.1 — this is the correct API; an earlier session wrongly concluded it was broken due to a by-value-argument misdiagnosis). It clamps `dt` internally so the final step lands exactly on `t_window_end` (no overshoot) and has a built-in `failed_step_checker` that throws if the step size collapses. The final time + step count are captured via the **observer** callback (start_time/dt are passed by value).
3. On window completion (at `t = t_window_end`): `commit()` all latches (current ← next). The commit happens at the **end** of the window, so a command proposed at `t` lands in `current` for the window `(t, t+dt]`. **Rejected internal steps need no discard hook** because `Latched::read()` never sees `next`.
4. Event check (jettison triggers) runs **after** the commit: on trigger → mark stage + its models inactive (keep state slots; spans stay valid) → **destroy & rebuild the stepper** (adaptive history invalid — mandatory; the stepper is rebuilt at the start of every window anyway) → drop inactive producers from accumulators, remove dead Signal edges, re-sort surviving subgraph → resume.
5. Telemetry snapshot at user-chosen rate.

**State source of truth:** the driver owns `x_live_` (`std::vector<double>`), the vector the integrator writes in place. The core's `x_` mirrors it: it is refreshed on every derivative call AND refreshed from `x_live_` before event triggers run (triggers read `SystemFunctor::state()`, which is backed by the core's `x_`; without the refresh, triggers see a stale first-probe state).

### 3.5 Telemetry
Names come for free: every ODE slot and every signal is named (needed for config wiring anyway). At sample rate: (t, all ODE names, all algebraic names, all latch values) → flat binary/CSV row. Optional ring-buffer + background writer thread (from sim4) if throughput demands.

### 3.6 Adding a new model (the ritual — must stay this small)
1. Write a struct deriving `StatefulModel<Dim>` (or a plain struct if Dim==0 — note: see §5 open item), declare its `StateDescriptor` (named slots), declare its `Signal`/`Accumulator`/`Latched` member views, implement `eval(ctx, sv)`.
2. Register the type name in the factory (one line in a `map<string, factory_fn>`).
3. Instantiate + wire in the vehicle config file.
Framework enforces all the invariants; the model author can't break the sim by accident.

### 3.7 Config (YAML/JSON) sections
`sim_params` (t_end, dt_macro, solver, telemetry rate/file) · `models:` (type, id, stage, params) · `connections:` (signal name → producer model, consumer models) · `events:` (jettison triggers) · `flight_software:` (which of the registered flight software versions this vehicle runs + its sensor/command signal names — exactly one per vehicle).

## 4. Files in the tree (all DONE — builds clean, example runs correctly)

```
td_sim3/
├── CMakeLists.txt             DONE — C++20, find_package(Boost) + find_package(spdlog REQUIRED);
│                                targets: tdsim (static lib) + td_sim_example (exe).
│                                Boost linked via Boost::boost (headers). spdlog::spdlog linked.
├── include/tdsim/
│   ├── signal.hpp             DONE — Signal<T>/Accumulator<T>/Latched<T> + SlotStore (double-backed).
│   ├── model.hpp              DONE — SimContext, StateView, StateDescriptor,
│   │                                StatefulModel<Dim>, ModelSlot (type erasure), ConcreteSlot<Dim,T>.
│   ├── topology.hpp           DONE — Connection, SimulationBuilder (registry, Kahn sort, validation
│   │                                ledger, Engineering Data Flow & Topology Report, jettison re-sort).
│   ├── system.hpp             DONE — SystemFunctor (odeint system fn wrapping Pass 1/2/3),
│   │                                SimulationCore (owns slots + live ODE state), reader registry.
│   ├── flight_software.hpp    DONE — IFlightSoftware interface + VehicleFlightSoftware (the
│   │                                example vehicle's single flight software: throttle hold +
│   │                                PID gimbal, one instance commanding both actuators).
│   ├── models.hpp             DONE — RigidBodyVehicle (1-D here: pz/vz + total_mass/total_force_z
│   │                                Accumulators), PropellantTank, LiquidEngine, PressureSensor,
│   │                                FirstOrderActuator (gimbal + throttle), and the ModelBag.
│   ├── telemetry.hpp          DONE — TelemetryLogger: name-table → CSV header + rows; sync write.
│   ├── factory.hpp            DONE — VehicleFactory: model registry (type-name → ctor), slot allocation,
│   │                                wiring, ODE init, example-vehicle builder.
│   └── simulator.hpp          DONE — Simulator: driver loop (§3.4), flight-computer registry,
│                                  jettison-event registry, telemetry hookup, x_live_ ownership.
├── src/
│   ├── topology.cpp           DONE — builder validation + report + jettison re-sort impl.
│   ├── system.cpp             (logic is header-only; .cpp is a small TU for linkage if needed)
│   ├── simulator.cpp          DONE — driver loop, try_step integration, jettison handling, spdlog.
│   ├── telemetry.cpp          DONE — CSV open/record/close.
│   └── factory.cpp            DONE — concrete model params + example vehicle assembly.
├── examples/
│   └── main.cpp               DONE — builds the example vehicle, adds ONE flight computer running
│                                VehicleFlightSoftware (throttle-hold + PID gimbal in one instance)
│                                + a fixed-time (t=60 s) stage-1 jettison, runs sim.
└── (config/, tests/ not yet created — see §6 open items)
```

**Build:** `cd td_sim3 && cmake -S . -B build && cmake --build build`. **Run:** `./build/td_sim_example`.
**Current status: example runs correctly end-to-end** (120 s flight, jettison at t=60 s, ballistic coast after). See §6 for the integration bugs that were found and fixed to get here.

## 5. Bugs identified in the user's spec drafts (fix when implementing — do NOT port them)

From user's `SimulationBuilder` draft:
- `wire_accumulator` stores sentinel string `"[Multi-Producer Accumulator]"` in `producer_model` field → breaks report + jettison rules. **Fix: `std::vector<std::string> producers` on `Connection`; "exactly 1" check = `producers.size()==1`.**
- No edge dedup → duplicate dependencies → double in-degree decrement in Kahn → **false cycle detection**. Dedup on insert.
- Kahn's is fine otherwise; keep the cycle-error message ("enforce explicit Latched<T> decoupling").

From user's `SimulationDriver` draft:
- **Passes 1/2/3 were in the driver while-loop, not the functor** → stale algebraics on 2nd..nth odeint functor calls. Fixed by design (§3.3).
- `Latched::discard()` (`next = current`) is a no-op after any `commit()`; and rejection isn't visible to the driver anyway. **Fix: read-from-`current`-only + commit-per-accepted-window; no discard.** (Implemented in `signal.hpp`.)
- 100 Hz tick missing from the driver loop.
- Mock `call_odeint_stepper` returns constant 0.01 — replaced with real `odeint::integrate_adaptive` (see §6.1 for the correct usage, and §6.2 for the 5-arg `try_step` gotcha to avoid if you ever hand-roll).

## 6. INTEGRATION BUGS found & fixed (the hard-won knowledge — read before touching the integrator)

The code was *structurally* complete but the example produced garbage (empty telemetry, `Final t = 349` past `t_end=120`, jettison at the wrong time). Three independent Boost 1.92 / orchestration bugs caused it. **Do not regress these.**

### 6.1 `integrate_adaptive` is the CORRECT API — the earlier 't=349/1875' was MY misuse, not a Boost bug
- **Corrected (after re-testing):** `integrate_adaptive(stepper, sys, x&, t0, tend, dt)` works perfectly on Boost 1.92. Verified against analytic solutions (`exp(-t)` to 5 digits) and on a discontinuous RHS (`x'=sign(x)`). It drives the 3-arg `try_step` internally, clamps `dt` so the final step lands exactly on `end_time` (no overshoot), and has a built-in `failed_step_checker` that throws after 500 consecutive rejections (so a pathological RHS can't hang the sim).
- **What actually went wrong:** the public signature passes `start_time` and `dt` **BY VALUE** — they are NOT updated in place. My first test read a stale local `t` after the call, which produced the bogus 't=1875'/'t=349' numbers. The real final time and step count must be captured via the **observer** callback (or the returned step count).
- **Fix (final):** use `integrate_adaptive` with an observer lambda that records the observed time + step count per window. This is *better* than a hand-rolled `try_step` loop: less code, the overshoot clamp is built in, and the `failed_step_checker` gives a clean `runtime_error` instead of an infinite loop. (A hand-rolled 3-arg `try_step` loop still works as a fallback and is documented in §6.2, but `integrate_adaptive` is preferred.)
- **`dt_min` is no longer enforced by the integrator.** The adaptive path relies on the stepper's error control + `failed_step_checker`. `params_.dt_min` is kept in `SimParams` for a potential future fixed-step solver; it is currently unused on the adaptive path.

### 6.2 The 5-arg `try_step(sys, x, dx, t, dt)` FAILS on EVERY step on Boost 1.92
- Symptom (when driving `try_step` by hand): the sim appeared "stuck" — every trial step rejected, `dt` collapsed, the 10 ms window never completed (a 120 s run took >10 min, effectively hung). Reproduced in isolation: even a trivial smooth ODE (`dy/dt=-y`) rejected all 1000 steps with the 5-arg form, for BOTH `runge_kutta_fehlberg78` and `runge_kutta_cash_karp54`.
- **Fix (if hand-rolling):** use the **3-arg form `try_step(sys, x, t, dt)`** (it manages its own internal `dxdt`). Verified: 3 steps over a 10 ms window, `t` lands exactly on the boundary. The 5-arg overload is disabled only when `StateIn == time_type`, so it *looks* valid for `std::vector<double>` but is broken on this Boost.
- **However:** since §6.1, we no longer hand-roll `try_step` at all — `integrate_adaptive` uses the 3-arg form internally and is the preferred path. This note is retained because (a) it explains a past symptom and (b) if you ever drop down to a manual loop, you must use the 3-arg form. **Most important gotcha: if you call `try_step` by hand, use 3 args, not 5.**

### 6.3 Event triggers read a stale ODE state
- Symptom: jettison fired at the wrong time. `SystemFunctor::state()` reads the **core's** `x_`, which is only refreshed when the functor is called as the ODE system. The integrator writes a separate live vector, so between windows (where triggers run) the core's `x_` held a stale first-probe value.
- **Fix:** the driver owns `x_live_` (the integrator's vector, single source of truth). `check_events` calls `built_.core->set_state(x_live_)` **before** any trigger runs, so `sys.state()` is current. Triggers that need to read the state should rely on this refresh (the example's fixed-time trigger doesn't read state, but a tank-empty trigger would).

### 6.4 Latched command timing (commit at window END, not start)
- Symptom: with commit at the window *start*, a jettison could leave the throttle frozen on a pre-jettison value (the "defer a whole window" hack was a wrong patch for a real timing bug).
- **Fix:** `commit_latches()` runs at the **end** of each window (after the integrate step, before the event check). A command proposed at `t` lands in `current` for the window `(t, t+dt]`. No window-skipping needed.

### 6.5 Example was physically incoherent (drove thrust to zero)
- Symptom: tanks went negative, vehicle free-fell. The original example had a PID on a *pressure* setpoint (50 kPa) but initial pressure was 100 kPa, so the controller anti-throttled to 0 → zero thrust.
- **Fix:** `examples/main.cpp` now uses (a) an open-loop **throttle hold at 100 %** and (b) a **PID gimbal hold**, with a fixed-time (t=60 s) stage-1 jettison. Tank sizes in `factory.cpp` tuned for a ~60 s burn (22,000 kg each; measured mdot ≈ 362 kg/s at full thrust — see telemetry). Verified: burn 0–60 s (thrust 1e6 N, vz 0→1291.5 m/s, pz 0→32.3 km), then ballistic coast (vz *exactly constant* at 1291.496 m/s, az=0, fuel frozen). **The physics is correct.**

### 6.6 spdlog notes
- `find_package(spdlog REQUIRED)` + `spdlog::spdlog` linked in CMake.
- This spdlog build does **not** honor the `SPDLOG_ACTIVE_LEVEL` env var (debug lines stay suppressed unless the level is set in code). `spdlog::error` (non-finite-derivative guard) always shows.
- The core (`system.hpp`) logs `spdlog::error` immediately if any derivative is NaN/Inf — the #1 cause of stepper collapse — plus `spdlog::debug` per-1000-call counts. The driver logs per-window step/rejection counts at `debug`.

## 7. Open items / next steps (in order)

DONE (this session): signal.hpp, model.hpp, topology.hpp + builder, system.hpp + functor, flight_software.hpp + PID, telemetry.hpp, models.hpp (vehicle/tank/engine/sensor/actuator), factory.hpp + factory.cpp, simulator.hpp + simulator.cpp, CMakeLists.txt, examples/main.cpp — **all building and the example running correctly (see §6).**

REMAINING (not started):
1. ~~**Config-driven vehicle (YAML/JSON).**~~ DONE — `src/toml_spec.cpp` + `include/tdsim/spec.hpp`, `config/vehicle.toml`; `build_from_spec` routes through the factory; TOML and programmatic paths verified identical (see §10 + the oracle test in `tests/test_factory.cpp`).
2. ~~**`tests/`**~~ DONE (doctest) — `tests/{main,test_signals,test_topology,test_factory}.cpp` + `registry_setup.hpp`, 23 cases / 3675 assertions green via `ctest` (target `tdsim_tests`). Covers signal/latch/accumulator semantics, SlotStore bound check, the full topology contract (two-producer errors, 0-producer warning, cycle detection + naming, Latched-no-edge, jettison re-sort), spec validation, the multi-producer Accumulator regression (§9a.4), and the TOML-vs-programmatic end-to-end oracle. See §10 for the two non-obvious gotchas (nondeterministic column order; buffered CSV flush) and the doctest registry-ordering gotcha.
3. **6-DOF vehicle** — the current `RigidBodyVehicle` is 1-D (pz/vz). Extend to 6-DOF (pos/vel + attitude + angular rates) when the vehicle model needs it. Good first stress test of the `StatefulModel<Dim>` / `std::span` machinery.

DONE (telemetry determinism): `Simulator::build_telemetry_columns` now sorts ALL columns (ODE + algebraic) by name before handing them to the `TelemetryLogger`. Previously the ODE columns followed `sys_.slots()` order (= topological evaluation order, which varies with model declaration order in the spec) and the algebraic columns came from an `unordered_map` — so `telemetry.csv` column order was nondeterministic across runs/platforms/hash seeds/equivalent-but-reordered specs. Now the schema is fully sorted and stable, so the file is diffable. The oracle test in `tests/test_factory.cpp` was tightened from "per-row multiset" to strict `ra[i] == rb[i]` string comparison, which now passes.

DONE (flight-software registry out of the example): added `FlightSoftwareRegistry` to `include/tdsim/flight_software.hpp` (static `register_software` / `register_builtin_software` / `find` / `software()`, mirroring `VehicleFactory`) + `src/registry_flight.cpp` (registers `VehicleFlightSoftware`, in a `std::map` so `--list-types` order is stable). `examples/td_sim.cpp` no longer holds the `software_registry()` local or references `VehicleFlightSoftware` directly — it calls `FlightSoftwareRegistry::register_builtin_software()` then `find(name)`. Verified: physics byte-identical (vz=976.0322, pz=85537.79), `--check` and `--list-types` still work, test suite green.

DONE (`--check`): the `--check` CLI flag was ALREADY implemented in `examples/td_sim.cpp` (it was listed as open in stale notes). `./td_sim --check <config>` loads the spec, registers types, builds the system (which runs the topology resolution + data-flow report), prints `[check] OK`, and exits WITHOUT running the Simulator. The only gap was a regression test, now added to `tests/test_factory.cpp`: (1) `"--check contract: a valid spec builds & resolves topology without integrating"` — builds the reference vehicle, asserts the evaluation order is a duplicate-free permutation of all models, the ODE state vector is sized & finite at t=0, and a single derivative eval is finite (proving the build path is well-formed without integrating); (2) `"--check contract: a topology-invalid spec fails at BUILD time"` — a two-`PressureSensor` Signal loop (`a.reading->b.source`, `b.reading->a.source`) throws inside `build_from_spec`, locking in that a broken wiring is caught on the build path (never reaching integration). Test suite now 25 cases / 1295 assertions.
4. **Optional async telemetry writer** (ring-buffer + background thread, from sim4) if throughput ever demands it — not needed at the current 10–100 Hz sample rates.
5. Update this file as work completes.

## 8. Decisions log (so future-me doesn't relitigate)

- Latch commit granularity = **per accepted macro window**, not per accepted micro step (simpler, rejection-proof). Revisit only if a model needs finer latch resolution.
- Jettison keeps state slots allocated (masking) so spans stay valid; inactive models' `eval` still runs? **No** — skip inactive models in Pass 2/3 (cheaper) but keep their slots so downstream spans remain valid. Inactive model's provided `Signal`s: leave last value; builder report notes it.
- `SlotStore` is double-backed (`std::vector<double>` + pointer cast) to keep one contiguous block per type; every signal T we support is `double`. If non-double signals ever appear, switch to `std::aligned_storage`-style per-type stores.
- No virtual base for models; `StatefulModel<Dim>` + `ConcreteSlot` erasure instead (avoids virtual-call-in-hot-loop and keeps `eval` inline-able).
- `std::any` NOT used for model state (rejected sim3's approach): state lives only in the flat vectors; models hold views.
- Telemetry default = inline synchronous write at ≤ ~1 kHz; async thread is an option flag, not the default.
- **Stepper = `runge_kutta_fehlberg78` driven by `integrate_adaptive`** (the correct high-level API — see §6.1). It uses the 3-arg `try_step` internally, clamps to the window end, and has a `failed_step_checker`. Capture the final time/step count via the **observer** (start_time & dt are passed by value — a past misdiagnosis came from reading a stale local). Do NOT hand-roll a 5-arg `try_step` (broken on Boost 1.92, §6.2); if you ever must hand-roll, use the 3-arg form. `dt_min` is unused on the adaptive path.
- **`x_live_` is the single source of truth for ODE state** (owned by the driver); the core's `x_` is a mirror refreshed on derivative calls and before event triggers (§3.4, §6.3). Don't reintroduce two independent state vectors.
- **Commit latches at window END, not start** (§6.4). The "defer a whole window after jettison" approach was a wrong patch — don't resurrect it.
- **SOLIDIFIED IN 3 PARTS (closed contract — do not reopen).** A second-opinion review proposed pushing commit/discard logic into the SystemFunctor via time-tracking (`last_evaluated_t` + discard-on-backward-t). Rejected. The invariants are now locked:
  1. **Latched<T> read isolation:** `read()` returns `current_`. Period. `next_` is a pure staging write-slot. Consumers never see `next_`, so a rejected speculative step is a non-event by construction. Do NOT add a `discard()` hook — it is a no-op after commit and the t-tracking detector that would drive it is wrong (RK stages evaluate at `t + a_i*dt` with `a_i > 0`, so backward-`t` never fires in-step; the "step advancement" trigger fires on every intermediate stage and commits prematurely).
  2. **Temporal freezing:** all discrete command inputs and algebraic-loop breakers are static over the entire macro window `(T, T + dt_macro]`. odeint's micro-stepping sees a completely frozen discrete environment. Do NOT make latched values change mid-window or mid-step.
  3. **Clean window boundaries:** the driver owns the timeline. It updates `x_live_`, runs the adaptive block (`integrate_adaptive`), and only updates `current_` at the macro checkpoint (window end) — before processing jettison events. The functor is a pure derivative function of `(x, t)` and holds no timeline state.
- **Stepper = `runge_kutta_dopri5` (RK54), NOT `runge_kutta_fehlberg78` (RK78).** (Supersedes the earlier RK78 decision above.) Benchmark on the example vehicle, FS active, 120 s + jettison, identical tolerances: RK78 and RK54 produce **identical step counts (48,000) and matching physics to 4.5e-5 absolute** (just the two error-controllers' estimates at the same tol), but RK54 is **~2.0–2.2× faster** (27.5 ms vs 56.2 ms). Reason: `dt_max=1 ms` pins every step at the ceiling, so order buys nothing and the cost is pure RHS-eval count (6 vs 7 per step) on a light algebraic RHS. RK78's order bonus only pays off on expensive smooth ODEs with large steps — the opposite regime. Revisit ONLY if we drop `dt_max` and start taking large steps on a smooth problem.
