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
| `change_state` | `lifecycle_msgs/srv/ChangeState` | ✅ Implemented | Proxy returns immediately; truth confirmed via `transition_event` |
| `get_state` | `lifecycle_msgs/srv/GetState` | ✅ Implemented | Provided via Rust proxy tool |
| `get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | ✅ Implemented | Provided via Rust proxy tool |
| `get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | ✅ Implemented | Provided via Rust proxy tool |
| `create` | internal | ❌ Omitted | Wrapper-only concern |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern |

---

## Topics

| Topic | ROS Type | Status | Notes |
|------|---------|--------|------|
| `transition_event` | `lifecycle_msgs/msg/TransitionEvent` | ✅ Implemented | Bridged by proxy from backend events |
| `/bond` | `bond/msg/Status` | ✅ Implemented | Proxy heartbeat for Nav2 lifecycle manager |

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

### ChangeState truthfulness (rosbridge caveat)

`rosbridge` service handlers are synchronous. Blocking inside the proxy
`change_state` callback can starve the websocket executor and cause ROS CLI
timeouts, even when the backend succeeds.

Current behavior (by design):

- Proxy returns `success: true` immediately for valid transition IDs.
- Proxy updates its local state to the transitional state right away
  (Configuring/Activating/Deactivating/CleaningUp/ShuttingDown).
- `get_state` reports the expected goal primary state while a transition is
  pending (rclcpp-compatible observable behavior).
- Backend call is fire-and-forget.
- `transition_event` is the source of truth for final state.
- If no matching event arrives within a short window, the proxy logs a warning
  and reverts local state to the previous primary state. The warning window can
  be tuned with `ROSRUSTEXT_CHANGE_STATE_TIMEOUT_MS`.

This preserves ROS tooling usability while keeping a clear signal when the
backend did not actually transition.

### Bond compatibility (Nav2 lifecycle manager)

Nav2 uses `bond` to verify that managed nodes are alive after transitions.
The proxy provides a minimal bond heartbeat so Nav2 can manage roslibrust
nodes without disabling bond.

Current behavior:

- Publishes `bond/msg/Status` on `/bond`
- `id` is the target node name, `instance_id` is generated per proxy start
- Heartbeats are sent while the node is in any non-finalized state

This is intentionally minimal (publish-only) and designed for compatibility
with `nav2_lifecycle_manager` when rosbridge QoS settings are fixed.

Verification:

- `scripts/test_nav2_bond.sh`
- `ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:="[hyfleet_ring_roslibrust]" -p autostart:=true`

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

The two adapter documents should match this spec while capturing adapter-specific
constraints and deviations.
