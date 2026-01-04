# Lifecycle Parity – ros2_rust Adapter

This document tracks lifecycle parity for the **ros2_rust transport adapter**
(`rosrustext_ros2_rust` using `rclrs`).

Canonical reference:
- `docs/spec/lifecycle.md` (normative)

This file answers:
> “Given the ROS2 lifecycle spec, what does the ros2_rust adapter provide?”

---

## Dependency source

- Primary: `rclrs` (crate) from crates.io  
  (upstream repository: `ros2-rust/ros2_rust`, multi-crate workspace)
- Pin to git **only** if Jazzy-required APIs or correctness bugs force it  
  (exact commit must be documented)

---

## Services (ROS-facing)

| Service | ROS Type | Status | Notes |
|------|---------|--------|------|
| `/<node>/change_state` | `lifecycle_msgs/srv/ChangeState` | ✅ Implemented (Slice-3 / WIP mapping) | Native service server. Minimal synchronous state update; emits one transition event per successful primary transition. |
| `/<node>/get_state` | `lifecycle_msgs/srv/GetState` | ✅ Implemented (Slice-3 / WIP mapping) | Native service server. |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | ✅ Implemented (Slice-3 / WIP mapping) | Native service server. |
| `/<node>/get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | ✅ Implemented (Slice-3 / WIP mapping) | Native service server. |
| `/<node>/get_transition_graph` | `rosrustext_interfaces/srv/GetTransitionGraph` | ✅ Implemented (feature `transition_graph`) | Custom introspection service (Jazzy lacks `lifecycle_msgs/GetTransitionGraph`). |
| `create` | internal | ❌ Omitted | Wrapper-only concern. |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern. |

**Custom introspection policy (Jazzy):**  
`lifecycle_msgs` in Jazzy does not include `GetTransitionGraph`. rosrustext
provides `rosrustext_interfaces/srv/GetTransitionGraph` **only** behind the
`transition_graph` feature. Default builds keep the standard Jazzy surface
and do not require the custom interface package.

**Design constraint:**  
`change_state` must not block the executor thread. Whether the transition
completes before the response is returned is **adapter-defined**, as permitted
by the ROS2 service contract (“able to initiate transition”).

---

## Topics

| Topic | ROS Type | Status | Notes |
|------|---------|--------|------|
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅ Implemented (Slice-3) | Native publisher. Emits exactly one event per successful primary transition. Jazzy `timestamp` is `uint64` nanoseconds. |
| `/bond` | `bond/msg/Status` | ✅ Implemented (feature `bond`) | Adapter-owned publisher + heartbeat timer for Nav2 lifecycle manager compatibility. **QoS is normative (see below).** |

---

## Semantics (core truth projected through adapter)

| Aspect | Status | Notes |
|------|--------|------|
| Busy-state rejection | ✅ Core-provided | Adapter must reject deterministically without mutating state. |
| Activation gating | ✅ Core-provided | `ActivationGate` owned by lifecycle node. |
| Publish suppression when inactive | ✅ Core-provided | Silent drop (no per-message warnings). |
| Timer suppression when inactive | ✅ Core-provided | Implemented via cancellation or guarded callbacks. |
| Shutdown from any state | ✅ Core-provided | Best-effort path to Finalized. |
| ErrorProcessing handling | ✅ Core-provided | Adapter maps outcomes to ROS-visible transitions + events. |
| Fatal error policy | ✅ Core-provided | Adapter enforces Finalized per core policy. |

---

## Callback execution model (transport-specific)

The ros2_rust adapter is a **native `rclrs` node**.

- Lifecycle callbacks (core): synchronous hooks.
- Service handlers: must **not block the executor thread**.
- Application owns the executor/spin loop (same model as rclcpp).
- No adapter-owned background spinner.

**Implication:**  
The adapter must support:
- transition work occurring off-thread or incrementally
- transition completion observed in executor context
- service response sent **after** completion *or* after initiation (both allowed)

Tokio is **not required** and should not be assumed.

---

### Bond QoS (normative)

Nav2 lifecycle manager expects `/bond` with the following QoS (not optional in practice):

- Reliability: **Reliable**
- Durability: **TransientLocal**
- History: **KeepLast(1)**
- Depth: **1**

This QoS is part of lifecycle parity (adapter responsibility), not an application tuning knob.

## Lifecycle node ownership & lifetime model (normative)

The ros2_rust lifecycle adapter **must mirror rclcpp/rclpy lifecycle ownership semantics**.

### Required behavior

* A `LifecycleNode` is the **primary node abstraction** exposed to application code.
* Lifecycle-internal ROS entities are **owned by the lifecycle node**, not the application:

  * Lifecycle services (`get_state`, `change_state`, introspection)
  * Transition event publisher
  * Bond publisher / heartbeat timer
* Application code **must not** be required to:

  * store lifecycle service handles
  * keep lifecycle timers alive
  * reason about lifecycle service lifetimes

### Rationale

This matches ROS 2 expectations:

* In `rclcpp_lifecycle::LifecycleNode`, lifecycle services exist *because the node exists*.
* Applications do not “hold” lifecycle services — they *use* the lifecycle node.
* Losing lifecycle services due to dropped handles is **not acceptable**.

### Rust-specific constraint

Because `rclrs` uses RAII lifetimes:

* The adapter **must internally retain** service / timer / publisher handles
* This retention is an **implementation detail**, invisible to application code

### Consequence for adapter API

* Lifecycle service registration functions:

  * **do not return handles**
  * return `Result<()>`
* Application-facing node structs should only retain:

  * the `LifecycleNode`
  * application-owned publishers / timers / subscriptions

This requirement is **part of lifecycle parity** and not an optional convenience.

## Implementation notes (Jazzy, rclrs 0.6)

- Lifecycle-owned entities are enabled by default:
  - `LifecycleNode::try_new` / `try_with_gate` call `enable_defaults`
  - `enable_defaults` wires transition_event + lifecycle services + bond (feature-gated)
  - Code: `crates/rosrustext_ros2_rust/src/lifecycle/node.rs`
- RAII retention:
  - `LifecycleNode::keep_internal` stores services/publishers/timers in `internals`
  - Code: `crates/rosrustext_ros2_rust/src/lifecycle/node.rs`
- Activation gating:
  - `ManagedPublisher::publish` drops when inactive
  - `create_timer_repeating_gated` checks gate inside timer callback
  - Code: `crates/rosrustext_ros2_rust/src/lifecycle/managed_publisher.rs`,
    `crates/rosrustext_ros2_rust/src/lifecycle/node.rs`,
    `crates/rosrustext_ros2_rust/src/lifecycle/managed_timer.rs`
- Transition events:
  - Emitted once per successful primary transition in `enable_change_state_service`
  - Jazzy timestamp uses `u64` nanoseconds in `make_transition_event`
  - Code: `crates/rosrustext_ros2_rust/src/lifecycle/node.rs`
- Bond (feature `bond`):
  - Enabled in `enable_bond`, invoked by `enable_defaults`
  - Heartbeat period 1s, timeout 4s; active only in Active state
  - Code: `crates/rosrustext_ros2_rust/src/lifecycle/node.rs`,
    `crates/rosrustext_ros2_rust/src/lifecycle/bond_agent.rs`

### Interface package location

- Canonical package: `interfaces/rosrustext_interfaces`
- dev_ws links it into `rosrustext_dev_ws/src/rosrustext_interfaces`
  (created by `rosrustext_dev_ws/scripts/source_env.sh`)

## Transport-specific constraints (rclrs)

- Executor: application-provided (single-threaded baseline).
- Service response timing: must prove deferred response is possible without blocking.
- Discovery & remapping: standard ROS 2 behavior.
- CLI compatibility: `ros2 lifecycle`, `ros2 node list`, etc. must work without proxy nodes.
- rclrs 0.6 service callbacks must use **concrete** Request/Response signatures:
  - `move |req: <pkg>::srv::<Service>_Request| -> <Service>_Response`
  - Using `ServiceIDL` Request/Response types caused `ros2 service call` to hang.

## Validation (Jazzy)

- `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal configure` → “Transitioning successful”
- `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal activate` → “Transitioning successful”
- `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal deactivate` → “Transitioning successful”
- `ros2 service call /ros2_rust_lifecycle_gate_minimal/get_state lifecycle_msgs/srv/GetState "{}"`
  - returns `id=2 label='Inactive'` after configure
  - returns `id=3 label='Active'` after activate
- CLI smoke: `scripts/test/ros2_rust/lifecycle/test_lifecycle_cli.sh`
- `ros2 topic echo /ros2_rust_lifecycle_gate_minimal/transition_event lifecycle_msgs/msg/TransitionEvent --once`
- Bond smoke: `scripts/test/ros2_rust/lifecycle/test_bond.sh`
  - observes `active=true` after activate
  - observes `active=false` after deactivate
- Nav2 lifecycle manager smoke: `scripts/test/ros2_rust/lifecycle/test_nav2_lifecycle_manager.sh`
- Transition graph smoke: `scripts/test/ros2_rust/lifecycle/test_transition_graph.sh`
- Full suite: `scripts/test/ros2_rust/run_all_tests.sh` (runs all four tests)

Note: Nav2 may log an "incompatible QoS" warning for `/bond` when another
subscriber with mismatched QoS is present; this is expected and does not
invalidate the test.

---

## Known gaps / risks (must be proven)

- Deferred service response correctness in `rclrs` (long-running transitions)
- Executor-safe transition completion signaling
- Timer cancellation vs guarded execution trade-offs
- Minimal parameter surface expectations (if any)

---

## Test layers

- Core unit tests (Rust): `cargo test -p rosrustext_core`
- Adapter integration tests (Rust): `cargo test -p rosrustext_ros2_rust`
- System tests (ROS CLI + managers): reuse scripts with native adapter (no proxy)

---

## Definition of Done (ros2_rust lifecycle)

Lifecycle parity is complete when a Rust node using `rosrustext_ros2_rust` can be:

- Driven by `ros2 lifecycle get/set`
- Managed by Python lifecycle managers
- Managed by `nav2_lifecycle_manager` (bond enabled)

…with no semantic drift from `docs/spec/lifecycle.md`.

### Application-owned vs lifecycle-owned entities (normative)

* Lifecycle-owned ROS entities are retained internally by `LifecycleNode`:

  * lifecycle services (`get_state`, `change_state`, introspection)
  * `transition_event` publisher
  * bond publisher (when feature `bond` is enabled)

* Application-owned entities are **not retained**:

  * publishers
  * timers
  * subscriptions

**Rationale:**
This matches `rclcpp_lifecycle::LifecycleNode` semantics exactly.
The application must retain handles to keep entities alive.

This is **required for parity**, not an ergonomic choice.
