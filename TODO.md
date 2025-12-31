# rclrust – TODO (ROS2 Parity Oriented)

This list tracks what is required to reach **observable parity**
with ROS2 lifecycle nodes (`rcl_lifecycle` + `rclcpp_lifecycle`).

The rule:
- `rclrust_core` defines truth
- `rclrust_wrapper` adapts that truth to ROS

---

## Lifecycle – Core (ROS-agnostic)

### State machine
- [x] Primary states (Unconfigured / Inactive / Active / Finalized)
- [x] Transition states (Configuring / CleaningUp / Activating / Deactivating / ShuttingDown / ErrorProcessing)
- [x] Explicit begin → callback → finish pipeline
- [x] Callback result model (Success / Failure / Error)
- [x] ErrorProcessing recovery semantics
- [x] Deterministic unit tests for all transitions

### Transition semantics
- [x] Explicit list of available transitions per state
- [x] Busy-state rejection (no transitions while transitioning)
- [x] Shutdown modeled from all primary states
- [x] ROS transition ID mapping (Configure / Activate / Shutdown variants)

### Alignment verification
- [ ] Final cross-check against ROS2 lifecycle diagram
- [ ] Decide whether CREATE / DESTROY remain wrapper-only (likely yes)

---

## Lifecycle – Wrapper (ROS-facing)

### Lifecycle node API
- [x] LifecycleNode abstraction
- [x] Internal state tracking
- [x] ActivationGate ownership
- [x] ChangeState handler
- [x] GetState handler
- [x] GetAvailableTransitions handler
- [ ] GetAvailableStates handler
- [ ] GetTransitionGraph handler
- [ ] Lifecycle state change publisher

### Managed entities
- [x] Activation-gated publisher
- [x] Activation-gated timer (tokio-based)
- [x] Publish suppression when inactive
- [ ] Decide warning vs silent drop policy (match ROS2)

### Transport
- [x] roslibrust integration behind feature flag
- [x] Transport adapters isolated under `transport::*`
- [ ] Wire lifecycle service servers (rosbridge)
- [ ] Ensure lifecycle manager compatibility (Python + C++)

### Error & shutdown policy
- [x] ErrorProcessing recovery delegated to wrapper
- [ ] Define fatal error shutdown policy
- [ ] Confirm shutdown semantics per ROS2 expectations

---

## Actions – Core (future)

- [ ] Explicit action protocol state machine
- [ ] Goal / feedback / result semantics
- [ ] Cancellation and timeout handling
- [ ] Unit tests

---

## Actions – Wrapper (future)

- [ ] Map action protocol to ROS2 actions
- [ ] Ensure compatibility with C++ action clients
- [ ] Execution and threading policy

---

## Infrastructure

- [ ] Logging conventions (severity mapping to ROS)
- [ ] Config patterns (deterministic, testable)
- [ ] Example lifecycle-managed node
- [ ] Example showing publisher + timer gating

---

## Documentation

- [ ] Lifecycle parity notes vs rclcpp_lifecycle
- [ ] Error handling philosophy (core vs wrapper)
- [ ] Design rationale: explicit lifecycle over framework magic
