# Lifecycle Parity – roslibrust Adapter

This document tracks lifecycle parity for the **roslibrust transport adapter**
(`rosrustext_roslibrust` + rosbridge).

Canonical reference:
- `docs/spec/lifecycle.md` (normative)

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
| `get_transition_graph` | `rosrustext_interfaces/srv/GetTransitionGraph` | ✅ Implemented | Custom introspection service (Jazzy-compatible) |
| `create` | internal | ❌ Omitted | Wrapper-only concern |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern |

---

## CLI note (Jazzy)

`ros2 interface show` for `lifecycle_msgs/msg/State` can fail if the active
overlay workspace has a broken symlink for `State.msg`, causing ros2cli to fall
back to the `.idl` file (which the rosidl_adapter parser cannot render). The
`GetTransitionGraph` service itself is correct; this is an environment issue.

Recommended fix:
- Ensure the overlay `lifecycle_msgs` install has a real `State.msg`, or
- Re-source `/opt/ros/jazzy/setup.bash` after the overlay so ros2cli prefers the
  system `lifecycle_msgs` for interface rendering.

The transition graph test script applies this ordering automatically when it
detects a missing/unreadable overlay message.

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
| Publish suppression when inactive | ✅ | Silent drop (no warnings; publish returns `Ok(false)`) |
| Timer suppression when inactive | ✅ | Matches publisher behavior |
| Shutdown from any state | ✅ | Best-effort path implemented |
| ErrorProcessing handling | ✅ | Delegated to core |
| Fatal error policy | ✅ | ErrorProcessing recovery in core (Success->Unconfigured, Failure/Error->Finalized) |

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

### Shutdown semantics

- Shutdown always resolves to `Finalized` (ShuttingDown → Finalized), regardless of callback result.
- If shutdown is requested while a transition is in flight, the proxy treats the
  pending goal state as the effective start state and issues a best-effort shutdown.

### Bond compatibility (Nav2 lifecycle manager)

Nav2 uses `bond` to verify that managed nodes are alive after transitions.
The proxy provides a minimal bond heartbeat so Nav2 can manage roslibrust
nodes without disabling bond.

Current behavior:

- Publishes `bond/msg/Status` on `/bond`
- `id` is the target node name, `instance_id` is generated per proxy start
- Heartbeats are sent while the node is Active
- Bond stops on deactivation, shutdown, or finalization
- Bond is dropped if the backend is unreachable (change_state transport failure or timeout)
- Bond can be disabled with `--no-bond` or `ROSRUSTEXT_BOND=0`

This is intentionally minimal (publish-only) and designed for compatibility
with `nav2_lifecycle_manager` when rosbridge QoS settings are fixed.

Verification:

- `scripts/test/roslibrust/lifecycle/test_nav2_bond.sh`
- `scripts/test/roslibrust/lifecycle/test_python_lifecycle_manager.sh`
- `scripts/test/roslibrust/lifecycle/test_lifecycle_stress.sh`
- `ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:="[hyfleet_ring_roslibrust]" -p autostart:=true`

---

## Proxy requirement

roslibrust-backed nodes expose lifecycle services under a private namespace:

- `/<target>/_rosrustext/change_state`
- `/<target>/_rosrustext/get_state`
- `/<target>/_rosrustext/get_available_states`
- `/<target>/_rosrustext/get_available_transitions`
- `/<target>/_rosrustext/get_transition_graph`
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

All gaps are intentional and tracked.

---

## Definition of Done (roslibrust)

The roslibrust adapter is considered lifecycle-complete when:

- Python lifecycle manager can:
  - Discover the node
  - Drive all valid transitions
  - Observe transition events
- `ros2 lifecycle get/set` works without warnings
- No semantic drift from `docs/spec/lifecycle.md`

---

## Next Adapter

A parallel document will exist for:
- `docs/adapters/ros2rust/lifecycle/parity.md`

The two adapter documents should match this spec while capturing adapter-specific
constraints and deviations.
