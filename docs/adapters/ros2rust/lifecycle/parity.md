# Lifecycle Parity – rosrs (rclrs) Adapter

This document tracks lifecycle parity for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/lifecycle.md` (normative)

This file answers:

> "Given the ROS2 lifecycle spec, what does the rosrs adapter provide?"

---

## Parity definitions (rosrs)

Tool parity (external behavior) means ROS tools and managers observe lifecycle
behavior that matches ROS 2 semantics, regardless of adapter internals.

Acceptance criteria (tool parity):

* `/<node>/change_state`, `get_state`, `get_available_states`, and
  `get_available_transitions` exist and match the state machine semantics.
* `transition_event` publishes once per accepted transition attempt after
  completion, with ROS-consistent IDs and labels.
* Busy/in-flight requests are rejected deterministically with `success=false`
  and no TransitionEvent.
* `get_transition_graph` is available when `transition_graph` is enabled and
  reflects the canonical transition table.
* `/bond` heartbeats (feature `bond`) are emitted only while Active with Nav2
  QoS.

User parity (developer experience) means authoring lifecycle nodes in Rust feels
like rclcpp-style lifecycle code.

Acceptance criteria (user parity):

* Callbacks can be installed at construction (executor or existing node) and
  replaced.
* Callbacks receive lifecycle context (node + gate/state) sufficient to allocate
  managed resources.
* Managed publishers/timers can be constructed inside callbacks via public API.
* An in-repo minimal lifecycle example demonstrates configure/activate with
  gated resources.
* Publish suppression is observable in user code.

Status summary: Tool parity is largely complete; user parity has known gaps (see
below).

---

## Dependency source

* Dev workspace (`colcon`) provides `rclrs` and ROS message crates
  via Cargo `[patch.crates-io]`.
* `rosrustext_rosrs` is **not publishable** until ROS msg crates are available
  on crates.io.

---

## Tool Parity (External behavior)

Tool parity tracks ROS-facing behavior: lifecycle services, transition_event,
bond, busy/in-flight handling, and graph introspection.

### Services (ROS-facing)

| Service                             | ROS Type                                       | Status | Notes                                                                                                                            |
| ----------------------------------- | ---------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------- |
| `/<node>/change_state`              | `lifecycle_msgs/srv/ChangeState`               | ✅     | Rejects invalid/busy with `success=false`. Accepted requests return `success=true` regardless of callback outcome. Completion may be sync (delay=0) or async (delay>0). |
| `/<node>/get_state`                 | `lifecycle_msgs/srv/GetState`                  | ✅     | Reports `StateMachine::stable_state()`; while in-flight, remains the previous stable state.                                      |
| `/<node>/get_available_states`      | `lifecycle_msgs/srv/GetAvailableStates`        | ✅     | Returns primary states only (Unconfigured/Inactive/Active/Finalized).                                                            |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions`   | ✅     | Uses `StateMachine::current_state()`; returns empty while in-flight (transition/intermediate state).                             |
| `/<node>/get_transition_graph`      | `rosrustext_interfaces/srv/GetTransitionGraph` | ✅     | Feature `transition_graph`. Returns primary states and transitions derived from `utils::TRANSITION_SPECS`.                        |

**Custom introspection policy (Jazzy):**
`lifecycle_msgs` in Jazzy does not include `GetTransitionGraph`. `rosrustext`
provides `rosrustext_interfaces/srv/GetTransitionGraph` **only** behind the
`transition_graph` feature. Default builds keep the standard Jazzy surface and
avoid the custom interface package.

**Design constraint:**
`change_state` must not block the executor thread. Whether the transition
completes before the response is returned is adapter-defined, per the ROS2
service contract ("able to initiate transition").

### Topics

| Topic                      | ROS Type                             | Status | Notes                                                                                                                   |
| -------------------------- | ------------------------------------ | ------ | ----------------------------------------------------------------------------------------------------------------------- |
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅     | Published once per accepted attempt after completion is applied. Busy/invalid requests do not emit events. Timestamp is `u64` nanoseconds. |
| `/bond`                    | `bond/msg/Status`                    | ✅     | Feature `bond`. Publisher + timer created at node startup; heartbeats only while Active; one edge message on active<->inactive transitions; QoS is Reliable + TransientLocal + KeepLast(1). |

### Behavioral semantics

| Aspect                               | Status | Notes                                                                                                               |
| ------------------------------------ | ------ | ------------------------------------------------------------------------------------------------------------------- |
| Single in-flight transition          | ✅     | Any transition request while `in_flight` is set is rejected.                                                        |
| Busy rejection semantics             | ✅     | Busy requests return `success=false` and do not emit `transition_event`.                                             |
| Transition IDs + labels              | ✅     | Transition IDs from `lifecycle_msgs::msg::Transition::*`; labels from `rosrustext_core::lifecycle::Transition::label()` (lowercase). State IDs match `lifecycle_msgs::msg::State` constants; labels are Rust enum debug strings. |
| ErrorProcessing + on_error semantics | ✅     | If a transition callback yields `Error`, `on_error()` is invoked and the final state resolves per core rules.        |
| Async/in-flight execution model      | ✅     | Controlled by `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS`: 0 runs callbacks inline; >0 spawns a worker thread and completion is applied by `enable_completion_pump` timer. |
| Lifecycle-owned entity retention     | ✅     | Services, publishers, and timers are stored in internals so application code does not need to keep handles alive.    |
| ManagedPublisher gating (local)      | ✅     | Drops publish calls while inactive; does not enable/disable DDS entities.                                            |
| ManagedTimer gating (local)          | ✅     | Timer callback is guarded by `ActivationGate::is_active()`; timer still runs at the `rclrs` layer.                   |

**Transition table single source of truth:**
The adapter derives validation, `get_available_transitions`, and
`get_transition_graph` from one canonical transition table in
`crates/rosrustext_rosrs/src/lifecycle/utils.rs`.

### Execution model (transport-specific)

The rosrs adapter is a **native `rclrs` node**.

* Lifecycle callbacks (core): synchronous hooks.
* Service handlers: must **not** block the executor thread.
* Application owns the executor/spin loop (same model as rclcpp).
* No adapter-owned background spinner.

**Implication:**
The adapter supports:

* transition work occurring off-thread or incrementally
* transition completion observed in executor context (completion pump)
* service response sent **after** completion *or* after initiation (both allowed)

Tokio is **not required** and should not be assumed.

### Test-only hooks (ROS workspace)

These environment variables exist to make semantics testable in a ROS workspace:

* `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS` — injects a delay; nonzero enables in-flight completion.
* `ROSRUSTEXT_RCLRS_TRANSITION_RESULT` / `ROSRUSTEXT_RCLRS_TRANSITION_RESULT_<TRANSITION>` —
  forces Success/Failure/Error for the primary transition.
* `ROSRUSTEXT_RCLRS_ON_ERROR_RESULT` / `ROSRUSTEXT_RCLRS_ON_ERROR_RESULT_<TRANSITION>` —
  forces Success/Failure/Error for ErrorProcessing.

### Bond QoS (normative)

Nav2 lifecycle manager expects `/bond` with the following QoS (not optional in practice):

* Reliability: **Reliable**
* Durability: **TransientLocal**
* History: **KeepLast(1)**
* Depth: **1**

This QoS is part of lifecycle parity (adapter responsibility), not an
application tuning knob.

### Lifecycle node ownership & lifetime model (normative)

The rosrs lifecycle adapter **must mirror rclcpp/rclpy lifecycle ownership
semantics**.

#### Required behavior

* A `LifecycleNode` is the **primary node abstraction** exposed to application code.
* Lifecycle-internal ROS entities are **owned by the lifecycle node**, not the application:

  * lifecycle services
  * transition_event publisher
  * bond publisher / heartbeat timer
* Application code **must not** be required to:

  * store lifecycle service handles
  * keep lifecycle timers alive
  * reason about lifecycle service lifetimes

#### Rust-specific constraint

Because `rclrs` uses RAII lifetimes:

* The adapter **must internally retain** service / timer / publisher handles
* This retention is an **implementation detail**, invisible to application code

---

## User Parity (Developer experience)

User parity tracks developer ergonomics and rclcpp-style lifecycle authoring.

| Aspect                              | Status | Notes                                                                                                                      |
| ----------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------- |
| Callback registration                | ⚠️     | Callbacks can be installed via `new_with_callbacks` or executor-based `create_with_callbacks`. `create`/`try_new` still use DefaultCallbacks; no set/replace API. |
| LifecycleNode handle in callbacks    | ✅     | `LifecycleCallbacksWithNode` receives `&LifecycleNode` and `&State`.                                                       |
| Allocate managed resources in callbacks | ✅  | `LifecycleCallbacksWithNode` can call `LifecycleNode::{create_publisher, create_timer_repeating_gated}`.                   |
| Callback return mapping              | ✅     | `CallbackResult::{Success, Failure, Error}` fully drives transition outcomes (including ErrorProcessing via `on_error`).   |
| Publish suppression signal           | ⚠️     | `ManagedPublisher::publish` returns `Result<()>` and does not indicate "suppressed vs published".                          |
| External minimal rclcpp-style example | ✅     | Example lives outside this repo and allocates gated publishers/timers in `on_configure`. |

---

## Parity Gaps / Required Changes (rosrs)

| Gap | Evidence | Fix | Risk | Test |
| --- | -------- | --- | ---- | ---- |
| No set/replace callbacks API | `LifecycleNode` only installs callbacks at construction. | Add `set_callbacks` / `replace_callbacks`. | Low | Unit/compile test for replacement. |
| Publish suppression signal | `ManagedPublisher::publish` returns `Result<()>` without indicating suppression. | Add a return enum or side-channel to signal suppressed publish. | Low | Unit test for suppressed publish in inactive state. |

---

## Known intentional differences

* Busy/invalid `change_state` requests do not emit `transition_event`.
* `change_state` responds `success=true` once a transition is accepted; callback
  outcome is reflected in `get_state` and `transition_event`.
* Transition graph uses `rosrustext_interfaces/srv/GetTransitionGraph` behind
  the `transition_graph` feature (Jazzy compatibility).
* Managed publisher/timer parity is achieved by local gating, not DDS/RCL
  enable/disable.

---

## Validation (Jazzy)

* `ros2 lifecycle get /ros2_rust_lifecycle_gate_minimal`
* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal configure`
* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal activate`
* `ros2 lifecycle set /ros2_rust_lifecycle_gate_minimal deactivate`
* `ros2 topic echo /<node>/transition_event --once`
* Bond smoke: `scripts/test/ros2_rust/lifecycle/test_bond.sh`
* Nav2 manager smoke: `scripts/test/ros2_rust/lifecycle/test_nav2_lifecycle_manager.sh`
* Transition graph smoke: `scripts/test/ros2_rust/lifecycle/test_transition_graph.sh`
* ChangeState timing (rclpy, not `ros2` CLI): `scripts/test/ros2_rust/lifecycle/test_change_state_timing.sh`
* Busy rejection: `scripts/test/ros2_rust/lifecycle/test_busy_rejection.sh`
* Failure path: `scripts/test/ros2_rust/lifecycle/test_change_state_failure.sh`
* ErrorProcessing -> Unconfigured: `scripts/test/ros2_rust/lifecycle/test_change_state_error_processing_unconfigured.sh`
* ErrorProcessing -> Finalized: `scripts/test/ros2_rust/lifecycle/test_change_state_error_processing_finalized.sh`
* CLI smoke: `scripts/test/ros2_rust/lifecycle/test_lifecycle_cli.sh`
* User parity example: `scripts/test/ros2_rust/lifecycle/test_rosrustext_rosrs_user_parity.sh`
* Full suite: `scripts/test/ros2_rust/run_all_tests.sh`

---

## Definition of Done (rosrs lifecycle)

Tool parity is complete when a Rust node using `rosrustext_rosrs` can be:

* Driven by `ros2 lifecycle get/set`
* Managed by Python lifecycle managers
* Managed by `nav2_lifecycle_manager` (bond enabled)

User parity is complete when lifecycle callbacks can access lifecycle context,
allocate gated resources, callbacks can be installed/replaced at runtime, and
publish suppression is observable, with an in-repo example demonstrating the
intended authoring workflow.
