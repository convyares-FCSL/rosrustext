# Lifecycle Parity Spec (ROS2 Canonical)

This document defines the *canonical* ROS2 lifecycle surface area and expected semantics.
It is transport-agnostic.

Related implementation matrices:
- `docs/lifecycle_parity_roslibrust.md`
- `docs/lifecycle_parity_ros2_rust.md` (future)

## Names & Interfaces

Node namespace:
- Services: `/<node_name>/<service_name>`
- Topic: `/<node_name>/transition_event`

Required services (ROS2 lifecycle):
- `change_state` — `lifecycle_msgs/srv/ChangeState`
- `get_state` — `lifecycle_msgs/srv/GetState`
- `get_available_transitions` — `lifecycle_msgs/srv/GetAvailableTransitions`
- `get_available_states` — `lifecycle_msgs/srv/GetAvailableStates`
- `get_transition_graph` — `lifecycle_msgs/srv/GetTransitionGraph`

Required topic:
- `transition_event` — `lifecycle_msgs/msg/TransitionEvent`

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

- Transition event:
  - Published after successful transitions.
  - Must not block state machine progression.
  - Ordering preserved per node instance.

## Edge-case compatibility traps

- Shutdown requested while transitioning
- Error returned from callbacks (Failure vs Error)
- Unsupported transition IDs
- Re-entrant ChangeState storms

## Definition of Done

Lifecycle parity is complete when:
- `ros2 lifecycle get/set` works end-to-end
- Python lifecycle manager can drive and observe the node via standard endpoints
- Transition event behavior matches expectations (no deadlocks, ordered, non-spam)
