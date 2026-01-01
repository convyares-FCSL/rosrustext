# rosrustext – TODO (ROS2 Parity Oriented)

This list tracks what is required to reach **observable parity**
with ROS2 lifecycle nodes (`rcl_lifecycle` + `rclcpp_lifecycle`).

Guiding rule:
- `rosrustext_core` defines lifecycle truth
- `rosrustext_roslibrust` adapts that truth to ROS transports

This is **not** a reimplementation of rclcpp.
It is an explicit, testable lifecycle model that can be *controlled by*
standard ROS2 lifecycle managers.

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
- [x] ROS transition ID mapping (Configure / Activate / Deactivate / Cleanup / Shutdown)

### Alignment verification
- [ ] Final cross-check against official ROS2 lifecycle diagram
- [ ] Confirm edge cases (shutdown during transition, error escalation)
- [ ] Decide whether CREATE / DESTROY remain wrapper-only (likely yes)

---

## Lifecycle – Wrapper (ROS-facing)

### Lifecycle node API
- [x] `LifecycleNode` abstraction
- [x] Internal state tracking
- [x] ActivationGate ownership
- [x] `ChangeState` handler
- [x] `GetState` handler
- [x] `GetAvailableTransitions` handler
- [x] `GetAvailableStates` handler
- [ ] `GetTransitionGraph` handler
- [x] Lifecycle state change publisher (`/transition_event` equivalent)

### Managed entities
- [x] Activation-gated publisher
- [x] Activation-gated timer (tokio-based)
- [x] Publish suppression when inactive
- [ ] Decide warning vs silent drop policy (match ROS2 behaviour)

### Transport
- [x] roslibrust integration behind feature flag
- [x] Transport adapters isolated under `transport::*`
- [x] ChangeState service via rosbridge
- [x] Wire remaining lifecycle services via rosbridge (via Rust proxy tool)
- [x] `ros2 lifecycle set/get` works over rosbridge (proxy)
- [x] Document rosbridge node name requirement for CLI discovery
- [ ] Verify compatibility with Python lifecycle manager
- [ ] Verify compatibility with C++ lifecycle manager

### Error & shutdown policy
- [x] ErrorProcessing recovery delegated to wrapper
- [x] Best-effort shutdown path implemented
- [ ] Define fatal error shutdown policy (when to force Finalized)
- [ ] Document shutdown semantics vs ROS2 expectations

---

## Lifecycle – Parity Matrix (Documentation)

> This section blocks Milestone 6 completion.

- [ ] Create lifecycle parity table:
  - Service/topic name
  - Implemented / Stubbed / Omitted
  - Notes vs `rclcpp_lifecycle`
- [ ] Explicitly document intentional deviations
- [ ] Confirm “boring compatibility” with lifecycle managers

---

## Actions – Core (future)

- [ ] Explicit action protocol state machine
- [ ] Goal / feedback / result semantics
- [ ] Cancellation and timeout handling
- [ ] Deterministic unit tests

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
- [x] Rust lifecycle proxy tool (rosbridge)
- [x] Local run scripts (rosbridge/backend/proxy/lifecycle test)

---

## Naming / Packaging

- [ ] Keep `rosrustext_roslibrust` name stable for now
- [ ] Consider adding a top-level façade crate later
  - e.g. `fcsl_ros_rust` or similar
  - Pure re-export + documentation layer only

---

## Definition of Done (Lifecycle)

Lifecycle is considered **complete** when:

- A Rust node can be controlled by:
  - `ros2 lifecycle set`
  - Python lifecycle manager
  - C++ lifecycle manager
- All lifecycle services respond correctly
- `transition_event` is published with valid IDs/labels

---

## Move to dual support (Lifecycle)

Link to example roproject shows 4 (2 rust + CPP + python)
