# rosrustext – TODO (ROS 2 Parity Oriented)

This file tracks work required to reach **observable parity**
with ROS 2 semantics as exercised by real tooling
(`ros2 lifecycle`, lifecycle managers, Nav2, launch).

Guiding rules:
- `rosrustext_core` defines semantic truth
- Adapters project that truth into ROS transports
- Parity is judged by **behavior under ROS tools**, not API shape

Lifecycle is the **first completed feature**, not the final goal.

---

## 1. Lifecycle – Core (`rosrustext_core`)

### State machine
- [x] Primary states (Unconfigured / Inactive / Active / Finalized)
- [x] Transition states (Configuring / CleaningUp / Activating / Deactivating / ShuttingDown / ErrorProcessing)
- [x] Begin → callback → finish pipeline
- [x] Callback result model (Success / Failure / Error)
- [x] ErrorProcessing recovery semantics
- [x] Deterministic unit tests for all transitions

### Transition semantics
- [x] Explicit transition table per state
- [x] Busy-state rejection
- [x] Shutdown from all primary states
- [x] ROS transition ID mapping
- [x] Alignment with official ROS 2 lifecycle diagram

---

## 2. Lifecycle – Adapter: rosrustext_roslibrust (rosbridge)

> Reference adapter — establishes parity baseline

### Lifecycle ROS surface
- [x] `change_state`
- [x] `get_state`
- [x] `get_available_transitions`
- [x] `get_available_states`
- [x] `get_transition_graph` (custom interface)
- [x] `transition_event` publisher
- [x] `/bond` heartbeat (Nav2 compatible)

### Semantics
- [x] Activation gating
- [x] Silent publish suppression when inactive
- [x] Busy-state rejection
- [x] ErrorProcessing mapping
- [x] Deterministic shutdown behavior

### Compatibility validation
- [x] `ros2 lifecycle` CLI
- [x] Python lifecycle manager
- [x] `nav2_lifecycle_manager`

### Testing
- [x] CLI smoke tests
- [x] Python manager tests
- [x] Nav2 bond tests
- [x] Stress / concurrency tests
- [x] Proxy-based test harness under `scripts/test/roslibrust`

---

## 3. Lifecycle – Adapter: rosrustext_rosrs (rclrs, dev_ws)

> Native RCL adapter — parity with roslibrust achieved in dev_ws

### Lifecycle ROS surface
- [x] `change_state` (non-blocking semantics validated)
- [x] `get_state`
- [x] `get_available_transitions`
- [x] `get_available_states`
- [x] `get_transition_graph` (feature-gated)
- [x] `transition_event` publisher
- [x] `/bond` heartbeat (feature `bond`, Nav2 QoS)

### Semantics
- [x] Activation-gated publishers
- [x] Activation-gated timers
- [x] Silent publish suppression
- [x] Busy-state rejection
- [x] Deterministic ChangeState timing contract
- [x] Best-effort shutdown handling

### Node model / API parity
- [x] `LifecycleNode` is the primary node abstraction
- [x] `LifecycleNode::create(&mut executor, name)`
- [x] `LifecycleNode::from_node(Arc<Node>)`
- [x] Explicit escape hatch (`node()` / `node_arc()`)
- [x] No `Deref<Target = Node>` (policy enforcement)

### Core integration
- [ ] Replace adapter-owned `Mutex<State>` with `rosrustext_core` state machine
- [ ] Wire lifecycle callbacks to core (configure / activate / deactivate / cleanup / shutdown)

### Compatibility validation
- [x] `ros2 lifecycle` CLI
- [x] Python lifecycle manager
- [x] `nav2_lifecycle_manager`

### Testing
- [x] CLI smoke tests (dev_ws-backed)
- [x] Bond smoke tests
- [x] Nav2 lifecycle manager tests
- [x] Transition graph tests
- [x] ChangeState timing test
- [x] System test suite under `scripts/test/ros2_rust`

---

## 4. Cross-cutting lifecycle parity

- [x] Lifecycle parity matrix (services / topics / semantics)
- [x] Document intentional deviations
- [x] Document Nav2 bond QoS requirements
- [x] Shared `rosrustext_interfaces` package
- [x] Consistent behavior across adapters

---

## 5. Parameters & Execution (future, non-blocking)

> Explicitly **not required** for lifecycle parity

### Parameters
- [ ] Parameter semantic model (core)
- [ ] Adapter mapping (roslibrust / rclrs)
- [ ] Tooling compatibility expectations

### Execution / threading
- [ ] Executor ownership model
- [ ] Transition execution policy
- [ ] Cancellation semantics

---

## 6. Actions (future feature)

### Core
- [ ] Action protocol state machine
- [ ] Goal / feedback / result semantics
- [ ] Cancellation and timeout rules
- [ ] Deterministic unit tests

### Adapters
- [ ] Map core actions to ROS 2 actions
- [ ] Verify C++ / Python client compatibility

---

## Definition of Done – Lifecycle

Lifecycle is considered **complete** when:

- A Rust node can be controlled by:
  - `ros2 lifecycle`
  - Python lifecycle managers
  - C++ / Nav2 lifecycle managers
- All lifecycle services respond correctly
- `transition_event` publishes correct IDs and labels
- `/bond` satisfies Nav2 expectations

After this point, lifecycle work moves to **maintenance only**.
