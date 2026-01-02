# Lifecycle Parity Spec (ROS2 Canonical)

This document defines the *canonical* ROS2 lifecycle surface area and expected semantics.
It is transport-agnostic.

Related implementation matrices:
- `docs/adapters/roslibrust/lifecycle/parity.md`
- `docs/adapters/ros2rust/lifecycle/parity.md` (future)

## Names & Interfaces

Node namespace:
- Services: `/<node_name>/<service_name>`
- Topic: `/<node_name>/transition_event`

Required services (ROS2 lifecycle):
- `change_state` — `lifecycle_msgs/srv/ChangeState`
- `get_state` — `lifecycle_msgs/srv/GetState`
- `get_available_transitions` — `lifecycle_msgs/srv/GetAvailableTransitions`
- `get_available_states` — `lifecycle_msgs/srv/GetAvailableStates`

Required topic:
- `transition_event` — `lifecycle_msgs/msg/TransitionEvent`

Additional (Nav2 compatibility):
- `/bond` — `bond/msg/Status` (heartbeat for lifecycle manager liveness checks)

rosrustext introspection (custom, Jazzy-compatible):
- `get_transition_graph` — `rosrustext_interfaces/srv/GetTransitionGraph`

## Required Semantics (Normative)

- State model:
  - Primary: Unconfigured, Inactive, Active, Finalized
  - Transition: Configuring, CleaningUp, Activating, Deactivating, ShuttingDown, ErrorProcessing

- Busy rejection:
  - While in a transition state, reject new transitions deterministically.

- ChangeState:
  - Accept ROS transition IDs per ROS conventions.
  - On success: state updates and event published.
  - On failure: state remains unchanged (unless ROS semantics require ErrorProcessing).

- Shutdown:
  - Valid from any primary state; maps to correct ROS shutdown IDs.
  - Always resolves to Finalized (ShuttingDown → Finalized), regardless of callback result.
  - If requested while transitioning, adapters should treat the pending goal state
    as the effective start state and attempt best-effort shutdown.

- Transition event:
  - Published after successful transitions.
  - Must not block state machine progression.
  - Ordering preserved per node instance.

- Managed entities:
  - Publisher/timer work is suppressed while inactive.
  - Suppression is silent (no per-message warnings) to avoid log spam.

- Error policy:
  - Invalid requests must not mutate state.
  - CallbackResult::Error triggers ErrorProcessing; on_error Success -> Unconfigured,
    Failure/Error -> Finalized (fatal policy in core).
  - Error kinds: InvalidArgument, InvalidState, InvalidTransition, NotSupported, Timeout, Transport.

## Edge-case compatibility traps

- Shutdown requested while transitioning
- Error returned from callbacks (Failure vs Error)
- Unsupported transition IDs
- Re-entrant ChangeState storms

## Transport caveat (informative)

Some transports (notably rosbridge) execute service callbacks synchronously.
Blocking inside a ChangeState handler can starve the websocket executor and make
`ros2 lifecycle set` time out even when the backend succeeds. Adapters should
avoid waiting inside the callback; update local state optimistically to the
transitional state, treat `transition_event` as the source of truth for final
state, and log/revert if the expected event is not observed. To preserve
rclcpp-style observables, `get_state` may report either the transitional state
or the expected goal primary state while a transition is pending.

## CLI note (informative)

`ros2 interface show` renders nested types by reading `.msg` sources. If an
overlay workspace shadows `lifecycle_msgs` with a broken `.msg` symlink, ros2cli
falls back to the `.idl` file and the rosidl_adapter parser fails. This is an
environment issue, not a lifecycle spec mismatch. Fix by restoring the
`lifecycle_msgs` `.msg` in the overlay or by preferring `/opt/ros/jazzy` for
ros2cli rendering.

## Nav2 bond compatibility (informative)

Nav2 lifecycle manager uses `bond` to ensure managed nodes remain alive after
transitions. Adapters that want Nav2 parity should provide a minimal bond
heartbeat on `/bond` using `bond/msg/Status`, with `id` matching the managed
node name and a unique `instance_id`. Heartbeats should run while the node is
active and stop on deactivation, shutdown, or finalization.

## Definition of Done

Lifecycle parity is complete when:
- `ros2 lifecycle get/set` works end-to-end
- Python lifecycle manager can drive and observe the node via standard endpoints
- Transition event behavior matches expectations (no deadlocks, ordered, non-spam)
