# rclrust – TODO (ROS2 Parity Oriented)

This list tracks what is required to reach functional parity with
ROS2 lifecycle nodes (`rcl_lifecycle` + `rclcpp_lifecycle`), while
keeping `rclrust_core` ROS-agnostic and pushing transport/policy
into `rclrust_wrapper`.

---

## Lifecycle – Core (ROS-agnostic, pure logic)

### State machine
- [x] Primary states (Unconfigured / Inactive / Active / Finalized)
- [x] Transition states (Configuring / CleaningUp / Activating / Deactivating / ShuttingDown / ErrorProcessing)
- [x] Explicit begin → callback → finish model
- [x] Callback result model (Success / Failure / Error)
- [x] ErrorProcessing recovery semantics (Success → Unconfigured, Failure → Finalized)
- [x] Deterministic unit tests for all core paths

### Transition model
- [ ] Explicit list of available transitions per state
      (needed for lifecycle managers + introspection)
- [ ] Explicit modelling of “busy” transition states
      (no transitions allowed while in Configuring, Activating, etc.)

### Semantics alignment
- [ ] Verify full alignment with ROS2 lifecycle diagram
      (including shutdown behavior from all primary states)
- [ ] Decide whether CREATE / DESTROY are modelled explicitly
      or treated as wrapper-only concerns

---

## Lifecycle – Wrapper (ROS-facing, policy + transport)

### ROS lifecycle API surface
- [ ] Implement ROS2 lifecycle services:
      - get_state
      - change_state
      - get_available_states
      - get_available_transitions
      - get_transition_graph
- [ ] Publish lifecycle state change notifications
- [ ] Map rclrust_core transitions to ROS2 transition IDs
      (including per-state shutdown IDs)

### Managed entities
- [ ] Activation-gated publishers (no publish unless Active)
- [ ] Activation-gated timers / workers
- [ ] Decide policy: drop vs warn on publish while inactive
      (ROS2 typically warns)

### Error and shutdown policy
- [ ] Define wrapper-level behavior on fatal error
      (immediate shutdown vs allow recovery)
- [ ] Ensure shutdown behavior matches ROS2 expectations
      from Unconfigured / Inactive / Active

### Compatibility
- [ ] Verify control from standard ROS2 lifecycle managers
      (Python + C++)
- [ ] Ensure behavior matches rclcpp_lifecycle observably

---

## Actions – Core (protocol-level, ROS-agnostic)

- [ ] Define action protocol core:
      - goal
      - feedback
      - result
      - cancel
- [ ] Explicit action state machine
- [ ] Timeout and cancellation semantics
- [ ] Unit tests for action protocol correctness

---

## Actions – Wrapper (ROS-facing)

- [ ] Map core action protocol to ROS2 actions
- [ ] Ensure compatibility with C++ action clients
- [ ] Decide threading / execution policy for callbacks

---

## Infrastructure

- [ ] Logging conventions (tracing, severity mapping)
- [ ] Config loading patterns (deterministic, testable)
- [ ] Example lifecycle node using rclrust_wrapper
- [ ] Example showing publisher gating in practice

---

## Documentation

- [ ] Lifecycle parity notes vs rclcpp_lifecycle
- [ ] Error-handling philosophy (core vs wrapper)
- [ ] Design rationale: why lifecycle is explicit, not “magic”
