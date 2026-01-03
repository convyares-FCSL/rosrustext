# Lifecycle Parity Spec (ROS2 Canonical)

This document defines the *canonical* ROS2 lifecycle surface area and semantics.
It is transport-agnostic.

Related implementation matrices:
- `docs/adapters/roslibrust/lifecycle/parity.md`
- `docs/adapters/ros2rust/lifecycle/parity.md`

---

## Names & Interfaces

Node namespace:
- Services: `/<node_name>/<service_name>`
- Topic: `/<node_name>/transition_event`

Required services:
- `change_state` — `lifecycle_msgs/srv/ChangeState`
- `get_state` — `lifecycle_msgs/srv/GetState`
- `get_available_transitions` — `lifecycle_msgs/srv/GetAvailableTransitions`
- `get_available_states` — `lifecycle_msgs/srv/GetAvailableStates`
- `get_transition_graph` — `lifecycle_msgs/srv/GetTransitionGraph`

Required topic:
- `transition_event` — `lifecycle_msgs/msg/TransitionEvent`

Additional (Nav2 compatibility):
- `/bond` — `bond/msg/Status`

---

## Required Semantics (Normative)

### State model

Primary:
- Unconfigured, Inactive, Active, Finalized

Transition:
- Configuring, CleaningUp, Activating, Deactivating, ShuttingDown, ErrorProcessing

---

### Busy rejection

- While in a transition state, reject new transitions deterministically.
- Rejection must not mutate state.

---

### ChangeState

- Accepts ROS transition IDs per conventions.
- Success response indicates **transition was accepted or initiated**.
- Completion timing is implementation-defined.
- Transition events must report actual outcomes.

---

### Shutdown

- Valid from any primary state.
- Maps to ShuttingDown → Finalized.
- If requested while transitioning:
  - Treat pending goal as effective start state.
  - Attempt best-effort shutdown.

---

### Transition events

- **One event per transition attempt**, regardless of outcome.
- Must be published after attempt resolution.
- Must not block state machine progression.
- Ordering preserved per node instance.

---

### Managed entities

- Publishers and timers suppressed while inactive.
- Suppression is silent (no warnings).

---

### Error policy

- Invalid requests do not mutate state.
- CallbackResult::Error triggers ErrorProcessing.
- ErrorProcessing outcomes:
  - Success → Unconfigured
  - Failure/Error → Finalized
- Error kinds: InvalidArgument, InvalidState, InvalidTransition, NotSupported, Timeout, Transport.

---

## Transport caveat (informative)

Some transports (e.g. rosbridge) execute service callbacks synchronously.
Adapters may respond to ChangeState immediately and rely on transition_event as
the source of truth, provided observables remain consistent with ROS tooling.

---

## Definition of Done

Lifecycle parity is complete when:
- `ros2 lifecycle get/set` works end-to-end
- Python and C++ lifecycle managers function correctly
- Transition events are reliable, ordered, and non-spammy
