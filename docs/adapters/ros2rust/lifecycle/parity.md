# Lifecycle Parity – ros2_rust Adapter

This document tracks lifecycle parity for the **ros2_rust transport adapter**
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/lifecycle.md` (normative)

This file answers:

> “Given the ROS2 lifecycle spec, what does the ros2_rust adapter provide?”

---

## Dependency source

* Primary: `rclrs` (crate) from crates.io
  (upstream repository: `ros2-rust/ros2_rust`, multi-crate workspace)
* Pin to git **only** if Jazzy-required APIs or correctness bugs force it
  (exact commit must be documented)

---

## Services (ROS-facing)

| Service                             | ROS Type                                       | Status                                     | Notes                                                                                                               |
| ----------------------------------- | ---------------------------------------------- | ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| `/<node>/change_state`              | `lifecycle_msgs/srv/ChangeState`               | ✅ Implemented (Slice-3 / WIP mapping)      | Native service server. Minimal synchronous state update; emits transition events on successful primary transitions. |
| `/<node>/get_state`                 | `lifecycle_msgs/srv/GetState`                  | ✅ Implemented (Slice-3 / WIP mapping)      | Native service server.                                                                                              |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions`   | ✅ Implemented (Slice-3 / WIP mapping)      | Native service server.                                                                                              |
| `/<node>/get_available_states`      | `lifecycle_msgs/srv/GetAvailableStates`        | ✅ Implemented (Slice-3 / WIP mapping)      | Native service server.                                                                                              |
| `/<node>/get_transition_graph`      | `rosrustext_interfaces/srv/GetTransitionGraph` | ✅ Implemented (feature `transition_graph`) | Custom introspection service. Jazzy lacks `lifecycle_msgs/GetTransitionGraph`.                                      |
| `create`                            | internal                                       | ❌ Omitted                                  | Wrapper-only concern.                                                                                               |
| `destroy`                           | internal                                       | ❌ Omitted                                  | Wrapper-only concern.                                                                                               |

**Custom introspection policy (Jazzy):**
`lifecycle_msgs` in Jazzy does not include `GetTransitionGraph`. `rosrustext`
provides `rosrustext_interfaces/srv/GetTransitionGraph` **only** behind the
`transition_graph` feature. Default builds keep the standard Jazzy surface
and do not require the custom interface package.

**Design constraint:**
`change_state` must not block the executor thread. Whether the transition
completes before the response is returned is **adapter-defined**, as permitted
by the ROS2 service contract (“able to initiate transition”).

---

## Topics

| Topic                      | ROS Type                             | Status                         | Notes                                                                                                                                                                                                               |
| -------------------------- | ------------------------------------ | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅ Implemented (Slice-3)        | Native publisher. Emits **one event per accepted transition attempt** (success/failure/error). Busy rejections do not emit events. Jazzy `timestamp` is `uint64` nanoseconds.                                     |
| `/bond`                    | `bond/msg/Status`                    | ✅ Implemented (feature `bond`) | Adapter-owned publisher + heartbeat timer for Nav2 lifecycle manager compatibility. **QoS is normative (see below).**                                                                                               |

---

### TransitionEvent behavior (documented)

Canonical ROS2 semantics require one TransitionEvent per transition attempt.

Current behavior (ros2_rust, Jazzy):

* TransitionEvent is emitted for **accepted** attempts (success/failure/error).
* Busy/invalid requests are rejected and do **not** emit events.

---

## Semantics (core truth projected through adapter)

| Aspect                            | Status          | Notes                                                                           |
| --------------------------------- | --------------- | ------------------------------------------------------------------------------- |
| Busy-state rejection              | ✅ Core-provided | Adapter rejects deterministically without mutating state.                       |
| Activation gating                 | ✅ Core-provided | `ActivationGate` owned by lifecycle node.                                       |
| Publish suppression when inactive | ✅ Core-provided | Silent drop (no per-message warnings).                                          |
| Timer suppression when inactive   | ✅ Core-provided | Implemented via guarded callbacks.                                              |
| Shutdown from any state           | ✅ Core-provided | Best-effort path to Finalized.                                                  |
| ErrorProcessing handling          | ✅ Core-provided | Adapter maps outcomes to ROS-visible transitions + events (future full wiring). |
| Fatal error policy                | ✅ Core-provided | Adapter enforces Finalized per core policy.                                     |

**Transition table single source of truth:**
The adapter derives validation, `get_available_transitions`, and `get_transition_graph`
from one canonical transition table in `crates/rosrustext_rosrs/src/lifecycle/utils.rs`.

---

## Callback execution model (transport-specific)

The ros2_rust adapter is a **native `rclrs` node**.

* Lifecycle callbacks (core): synchronous hooks.
* Service handlers: must **not block the executor thread**.
* Application owns the executor/spin loop (same model as rclcpp).
* No adapter-owned background spinner.

**Implication:**
The adapter supports:

* transition work occurring off-thread or incrementally
* transition completion observed in executor context
* service response sent **after** completion *or* after initiation (both allowed)

Tokio is **not required** and should not be assumed.

---

### Bond QoS (normative)

Nav2 lifecycle manager expects `/bond` with the following QoS (not optional in practice):

* Reliability: **Reliable**
* Durability: **TransientLocal**
* History: **KeepLast(1)**
* Depth: **1**

This QoS is part of lifecycle parity (adapter responsibility), not an application tuning knob.

---

## Lifecycle node ownership & lifetime model (normative)

The ros2_rust lifecycle adapter **must mirror rclcpp/rclpy lifecycle ownership semantics**.

### Required behavior

* A `LifecycleNode` is the **primary node abstraction** exposed to application code.
* Lifecycle-internal ROS entities are **owned by the lifecycle node**, not the application:

  * lifecycle services
  * transition_event publisher
  * bond publisher / heartbeat timer
* Application code **must not** be required to:

  * store lifecycle service handles
  * keep lifecycle timers alive
  * reason about lifecycle service lifetimes

### Rust-specific constraint

Because `rclrs` uses RAII lifetimes:

* The adapter **must internally retain** service / timer / publisher handles
* This retention is an **implementation detail**, invisible to application code

---

## Outstanding parity items

The following items are required to reach full parity with
`rclcpp_lifecycle::LifecycleNode` semantics:

* `LifecycleNode` becomes the primary node abstraction

  * `LifecycleNode::create(&mut executor, name)`
  * `LifecycleNode::from_node(Arc<Node>)`
* Explicit API forwarding (no `Deref`)
* Replace adapter-owned `Mutex<State>` with `rosrustext_core` state machine
* Emit TransitionEvent for rejected transitions (busy / invalid)

These are tracked in `TODO.md` and are **not blockers** for current Jazzy
lifecycle manager compatibility.

---

## Validation (Jazzy)

* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal configure`
* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal activate`
* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal deactivate`
* `ros2 topic echo /<node>/transition_event --once`
* Bond smoke: `scripts/test/ros2_rust/lifecycle/test_bond.sh`
* Nav2 manager smoke: `scripts/test/ros2_rust/lifecycle/test_nav2_lifecycle_manager.sh`
* Transition graph smoke: `scripts/test/ros2_rust/lifecycle/test_transition_graph.sh`
* ChangeState timing: `scripts/test/ros2_rust/lifecycle/test_change_state_timing.sh`
* Full suite: `scripts/test/ros2_rust/run_all_tests.sh`

---

## Definition of Done (ros2_rust lifecycle)

Lifecycle parity is complete when a Rust node using `rosrustext_rosrs` can be:

* Driven by `ros2 lifecycle get/set`
* Managed by Python lifecycle managers
* Managed by `nav2_lifecycle_manager` (bond enabled)

…with no semantic drift from `docs/spec/lifecycle.md`.
