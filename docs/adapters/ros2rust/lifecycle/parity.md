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
| `/<node>/change_state` | `lifecycle_msgs/srv/ChangeState` | 🚧 Planned | Native service server. **May complete transition before responding** (allowed by spec). Must not block executor thread. |
| `/<node>/get_state` | `lifecycle_msgs/srv/GetState` | 🚧 Planned | Native service server. |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | 🚧 Planned | Native service server. |
| `/<node>/get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | 🚧 Planned | Native service server. |
| `/<node>/get_transition_graph` | `lifecycle_msgs/srv/GetTransitionGraph` | 🚧 Planned | **Standard lifecycle introspection service**. Must match rclcpp observables. |
| `create` | internal | ❌ Omitted | Wrapper-only concern. |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern. |

**Design constraint:**  
`change_state` must not block the executor thread. Whether the transition
completes before the response is returned is **adapter-defined**, as permitted
by the ROS2 service contract (“able to initiate transition”).

---

## Topics

| Topic | ROS Type | Status | Notes |
|------|---------|--------|------|
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 🚧 Planned | Native publisher. Must emit **one event per transition attempt**, success or failure. |
| `/bond` | `bond/msg/Status` | 🚧 Planned | Native publisher for Nav2 lifecycle manager compatibility. |

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

## Transport-specific constraints (rclrs)

- Executor: application-provided (single-threaded baseline).
- Service response timing: must prove deferred response is possible without blocking.
- Discovery & remapping: standard ROS 2 behavior.
- CLI compatibility: `ros2 lifecycle`, `ros2 node list`, etc. must work without proxy nodes.

---

## Known gaps / risks (must be proven)

- Deferred service response correctness in `rclrs`
- Executor-safe transition completion signaling
- Timer cancellation vs guarded execution trade-offs
- Bond QoS + heartbeat timing under Nav2
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
