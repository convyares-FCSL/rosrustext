# Lifecycle Parity – roslibrust Adapter

This document tracks lifecycle parity for the **roslibrust transport adapter**
(`rosrustext_roslibrust` + rosbridge).

Canonical reference:
- `lifecycle_parity_spec.md` (normative)

This file answers:
> “Given the ROS2 lifecycle spec, what does the roslibrust adapter provide today?”

---

## Services

| Service | ROS Type | Status | Notes |
|------|---------|--------|------|
| `change_state` | `lifecycle_msgs/srv/ChangeState` | ✅ Implemented | Provided via Rust proxy tool |
| `get_state` | `lifecycle_msgs/srv/GetState` | ✅ Implemented | Provided via Rust proxy tool |
| `get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | ✅ Implemented | Provided via Rust proxy tool |
| `get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | ✅ Implemented | Provided via Rust proxy tool |
| `get_transition_graph` | `lifecycle_msgs/srv/GetTransitionGraph` | ⏳ Pending | Data already exists in core |
| `create` | internal | ❌ Omitted | Wrapper-only concern |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern |

---

## Topics

| Topic | ROS Type | Status | Notes |
|------|---------|--------|------|
| `transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅ Implemented | Bridged by proxy from backend events |

---

## Semantics

| Aspect | Status | Notes |
|------|--------|------|
| Busy-state rejection | ✅ | Core enforces deterministic rejection |
| Activation gating | ✅ | `ActivationGate` owned by node |
| Publish suppression when inactive | ✅ | Silent drop (policy TBD) |
| Timer suppression when inactive | ✅ | Matches publisher behavior |
| Shutdown from any state | ✅ | Best-effort path implemented |
| ErrorProcessing handling | ✅ | Delegated to core |
| Fatal error policy | ⏳ | Needs explicit definition |

---

## Proxy requirement

roslibrust-backed nodes expose lifecycle services under a private namespace:

- `/<target>/_rosrustext/change_state`
- `/<target>/_rosrustext/get_state`
- `/<target>/_rosrustext/get_available_states`
- `/<target>/_rosrustext/get_available_transitions`
- `/<target>/_rosrustext/transition_event`

The Rust proxy tool (`rosrustext_lifecycle_proxy`) bridges these to standard
ROS 2 lifecycle endpoints on the ROS graph so `ros2 lifecycle` works.

The rosbridge node is launched with the target node name to satisfy ROS 2
CLI node discovery (the roslibrust backend itself is not a ROS graph node).

---

## Transport-specific constraints (rosbridge)

- Service calls are serialized per websocket client
- No executor-level preemption
- Transition event publication must be non-blocking
- Python-side message availability depends on sourced workspace

These constraints **must not** leak into core lifecycle semantics.

---

## Known Gaps vs rclcpp_lifecycle

- No executor-managed callback groups
- No parameter lifecycle hooks (future)
- No `get_transition_graph` yet

All gaps are intentional and tracked.

---

## Definition of Done (roslibrust)

The roslibrust adapter is considered lifecycle-complete when:

- Python lifecycle manager can:
  - Discover the node
  - Drive all valid transitions
  - Observe transition events
- `ros2 lifecycle get/set` works without warnings
- No semantic drift from `lifecycle_parity_spec.md`

---

## Next Adapter

A parallel document will exist for:
- `lifecycle_parity_ros2_rust.md`

The two adapter docum
