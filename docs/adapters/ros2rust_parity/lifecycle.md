# Lifecycle Parity – rosrs (rclrs) Adapter

This document tracks lifecycle parity for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/lifecycle.md` (normative)

This file answers:

> **“Given the ROS 2 lifecycle specification, what does the rosrs adapter provide?”**

---

### Feature requirements

Spec feature flag: `lifecycle`

Normative meaning:
- Defines lifecycle states, transitions, busy rejection, transition_event semantics, and gating rules.
- Does not depend on a specific ROS transport.

Required (to claim lifecycle parity in an adapter):
- `core`
- Adapter-specific feature enabling lifecycle ROS surface (adapter-defined)

Optional (adapter-defined, must be explicit):
- `bond` (Nav2 compatibility surface)

Notes:
- The spec requires canonical ROS interfaces (`lifecycle_msgs/*`) but does not mandate how message types are provided (generated vs vendored vs proxy).

## Parity definitions (rosrs)

**Tool parity (external behavior)** means ROS tools and lifecycle managers observe
behavior that matches ROS 2 semantics, regardless of adapter internals.

Acceptance criteria (tool parity):

* `/<node>/change_state`, `get_state`, `get_available_states`, and
  `get_available_transitions` exist and follow the canonical state machine.
* `transition_event` publishes exactly once per accepted transition attempt,
  after completion, with ROS-consistent IDs and labels.
* Busy/in-flight requests are rejected deterministically with `success=false`
  and **no** `TransitionEvent`.
* `/bond` heartbeats (feature `bond`) are emitted only while **Active**, with
  Nav2-compatible QoS.

**User parity (developer experience)** means authoring lifecycle nodes in Rust
feels structurally comparable to rclcpp lifecycle code.

Acceptance criteria (user parity):

* Lifecycle callbacks can be installed at construction time.
* Callbacks receive sufficient lifecycle context (node + state/gate).
* Managed publishers and timers can be allocated inside lifecycle callbacks
  via public APIs.
* Publish suppression is observable by user code.
* An external minimal example demonstrates configure → activate with gated
  resources.

**Status summary:**
Tool parity is largely complete.
User parity is largely complete, with a small number of ergonomic gaps
documented below.

---

## Dependency source

* Canonical ROS lifecycle interfaces (`lifecycle_msgs`, and optionally `bond`)
  are consumed via **pre-generated Rust bindings** consistent with the tutorial
  workflow.
* End users are **not required** to run code generators to use lifecycle tooling
  parity.
* In a ROS workspace, Cargo `[patch.crates-io]` may be used to pin ROS message
  crate versions, but lifecycle functionality does not depend on user-side
  codegen.

---

## Tool Parity (External behavior)

Tool parity tracks ROS-facing behavior: lifecycle services, `transition_event`,
bond semantics, and busy rejection.

### Services (ROS-facing)

| Service                             | ROS Type                                       | Status | Notes                                                                                                                                                                                                                                                                                                                                                       |
| ----------------------------------- | ---------------------------------------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/<node>/change_state`              | `lifecycle_msgs/srv/ChangeState`               | ✅      | Rejects invalid/busy with `success=false`. `success=true` indicates the transition was accepted/initiated (ROS contract). When delay=0, the adapter attempts synchronous completion and applies the resulting stable state before responding. When delay>0, completion is asynchronous and the outcome is reflected via `transition_event` and `get_state`. |
| `/<node>/get_state`                 | `lifecycle_msgs/srv/GetState`                  | ✅      | Returns the **stable** lifecycle state only. While in-flight, remains at the previous stable state.                                                                                                                                                                                                                                                         |
| `/<node>/get_available_states`      | `lifecycle_msgs/srv/GetAvailableStates`        | ✅      | Returns primary states only (Unconfigured, Inactive, Active, Finalized).                                                                                                                                                                                                                                                                                    |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions`   | ✅      | Uses the current internal state; returns empty while in-flight.                                                                                                                                                                                                                                                                                             |

---

### Topics

| Topic                      | ROS Type                             | Status | Notes                                                                                                                                                                                             |
| -------------------------- | ------------------------------------ | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅      | Published once per accepted transition attempt after completion. Busy/invalid requests do not emit events. Timestamp is `u64` nanoseconds.                                                        |
| `/bond`                    | `bond/msg/Status`                    | ✅      | Feature `bond`. Publisher and heartbeat timer are lifecycle-owned. Heartbeats only while Active; one edge message on Active↔Inactive transitions. QoS is Reliable + TransientLocal + KeepLast(1). |

---

### Behavioral semantics

| Aspect                           | Status | Notes                                                                                   |
| -------------------------------- | ------ | --------------------------------------------------------------------------------------- |
| Single in-flight transition      | ✅      | Any request while a transition is in-flight is rejected.                                |
| Busy rejection semantics         | ✅      | Busy requests return `success=false` and do not emit `transition_event`.                |
| Transition IDs + labels          | ✅      | IDs from `lifecycle_msgs::msg::Transition::*`. Labels from adapter mapping.             |
| ErrorProcessing semantics        | ✅      | `CallbackResult::Error` triggers ErrorProcessing; outcome resolves per canonical rules. |
| Async/in-flight execution model  | ✅      | Controlled by `ROSRUSTEXT_RCLRS_CHANGE_STATE_DELAY_MS`.                                 |
| Lifecycle-owned entity retention | ✅      | Services, publishers, timers are retained internally by the lifecycle node.             |
| ManagedPublisher gating          | ✅      | Publish calls suppressed while inactive (local gating).                                 |
| ManagedTimer gating              | ✅      | Timer callbacks guarded by `ActivationGate::is_active()`.                               |

**Single source of truth:**
Validation and `get_available_transitions` are derived from one canonical
transition table.

---

## Execution model (transport-specific)

The rosrs adapter is a **native `rclrs` node**.

* Lifecycle callbacks are synchronous hooks.
* Service handlers must not block indefinitely.
* Application code owns the executor and spin loop.
* No adapter-owned background spinner exists.

**Implication:**
Lifecycle parity is achieved without Tokio or hidden runtime assumptions.

---

## Parity levels (Lifecycle)

Lifecycle parity is evaluated at two distinct levels:

### Tool parity (required)

* ROS lifecycle services behave canonically.
* `ros2 lifecycle get/set` works end-to-end.
* Busy rejection and transition events are deterministic.
* Nav2 lifecycle manager interoperability works when bond is enabled.

### Execution parity (preferred)

* Execution is gated by lifecycle state.
* Publishers are gated at publish-time.
* Timers are gated at callback entry.
* DDS/RCL entities are **not** enabled/disabled; gating is local and explicit.
  This is an intentional design choice.

---

## Lifecycle node ownership & lifetime model (normative)

* `LifecycleNode` is the **primary node abstraction**.
* Lifecycle-internal ROS entities are owned by the adapter:

  * lifecycle services
  * `transition_event` publisher
  * bond publisher and timer
* Application code must **not** manage lifecycle service or timer lifetimes.

Rust RAII constraints require the adapter to retain handles internally; this is
an implementation detail invisible to users.

---

## User Parity (Developer experience)

| Aspect                                  | Status | Notes                                                              |
| --------------------------------------- | ------ | ------------------------------------------------------------------ |
| Callback registration                   | ⚠️     | Installed at construction time; no public replace API yet.         |
| LifecycleNode in callbacks              | ✅      | `LifecycleCallbacksWithNode` provides node + state.                |
| Allocate managed resources in callbacks | ✅      | Publishers and timers can be created during `on_configure`.        |
| Callback return mapping                 | ✅      | `CallbackResult::{Success, Failure, Error}` fully drives outcomes. |
| Publish suppression visibility          | ✅      | `publish_with_outcome` reports suppression explicitly.             |
| Minimal rclcpp-style example            | ✅      | External example demonstrates intended authoring pattern.          |

---

## Parity Gaps / Required Changes (rosrs)

| Gap                          | Evidence                                 | Fix                                                                          | Risk | Test                 |
| ---------------------------- | ---------------------------------------- | ---------------------------------------------------------------------------- | ---- | -------------------- |
| No set/replace callbacks API | Callbacks installed only at construction | Add `set_callbacks` / `replace_callbacks`; reject while transition in-flight | Low  | Unit + compile tests |

---

## Known intentional differences

* Busy/invalid `change_state` requests do not emit `transition_event`.
* `success=true` on `change_state` indicates acceptance/initiated, not guaranteed success.
* Managed gating is local (publish/callback suppression), not DDS enable/disable.

---

## Validation (Jazzy)

* `ros2 lifecycle get /<node>`
* `ros2 lifecycle set /<node> configure|activate|deactivate|cleanup|shutdown`
* `ros2 topic echo /<node>/transition_event --once`
* Nav2 lifecycle manager smoke tests
* Busy rejection tests
* ErrorProcessing resolution tests
* Full suite: `scripts/test/ros2_rust/run_all_tests.sh`

---

## Definition of Done (rosrs lifecycle)

Tool parity is complete when a Rust node using `rosrustext_rosrs` can be:

* Driven by `ros2 lifecycle get/set`
* Managed by Python lifecycle managers
* Managed by `nav2_lifecycle_manager` (bond enabled)

User parity is complete when lifecycle callbacks can access full context,
allocate gated resources, callbacks can be replaced safely, and publish
suppression is observable, with a clear reference example.
