
# rosrustext – TODO (Production Parity Roadmap)

This file tracks work required to reach **production-grade parity**
with ROS 2 as exercised by real systems and tooling
(`ros2` CLI, launch, lifecycle managers, Nav2, orchestration).

Guiding rules:
- `rosrustext_core` defines semantic truth
- Adapters project that truth into ROS transports
- Parity is judged by **observable behavior**, not API shape
- rclrs (`rosrustext_rosrs`) is the **production target**
- roslibrust is tooling / reference parity only

Lifecycle is the **first completed capability**, not the final goal.

---

## 0. Hygiene / Baseline Stability

- [x] Clippy clean (core + adapters)
- [x] Docs build on docs.rs (no ROS dependency)
- [x] Patch alignment for `rclrs::vendor::*` message crates
- [x] Version alignment across core / adapters

### Outstanding
- [ ] Tutorial workspace compilation parity (examples vs docs)
- [ ] Import consistency (`State`, lifecycle enums)
- [ ] QoS helper naming/signature alignment
- [ ] Core ↔ adapter error taxonomy review

---

## 1. Lifecycle (COMPLETE – Maintenance Only)

### Core (`rosrustext_core`)
- [x] Canonical lifecycle state machine
- [x] Transition table (single source of truth)
- [x] ErrorProcessing semantics
- [x] Deterministic unit tests

### Adapter: roslibrust
- [x] Full lifecycle service surface
- [x] `transition_event`
- [x] Bond heartbeat (Nav2 QoS)
- [x] Busy rejection + gating
- [x] CLI + Python + Nav2 validation
- [x] Demo/test harness covers configure → activate → shutdown

### Adapter: rosrs (rclrs)
- [x] Full lifecycle service surface
- [x] Deterministic ChangeState timing contract
- [x] Gated publishers/timers
- [x] Bond heartbeat (feature gated)
- [x] CLI + Nav2 validation

Lifecycle work is **frozen** except for bug fixes.

---

## 2. Parameters (IN PROGRESS — Baseline Implemented)

Parameters are implemented to the extent required for Lesson 05
(declare + store + event-driven updates). Tool parity is mostly present.
Remaining work is focused on closing known rclcpp parity gaps and making
limitations explicit.

### Core
- [x] Parameter store model (name → typed value + descriptor)
- [x] Declared-only vs allow-undeclared mode support (documented)
- [x] Event emission rules (accepted updates only)
- [x] Atomic vs non-atomic semantics at store/service level
- [ ] Consolidate canonical parameter spec tests (ROS-free)

### Adapter: rosrs (production target)
- [x] Standard parameter services implemented
- [x] `/<node>/parameter_events` implemented
- [x] ParameterWatcher (event-driven; no polling)
- [x] Lesson 05 support (declare + watch + apply)
- [ ] Typed convenience accessors (ergonomics parity)
- [ ] Cross-parameter validation surface (blocked upstream; document)
- [ ] Deletion semantics decision (reject vs support + events)
- [ ] Lifecycle-aware parameter policy (what survives cleanup; when watchers run)

### Known hard limit (must be explicit)
- rclrs does not expose an `on_set_parameters_callback` equivalent, so true
  set-time user validation + atomic cross-parameter rejection cannot fully match rclcpp
  without upstream changes or an adapter-level workaround.


---

## 3. Actions (MAJOR FEATURE GAP)

### Core
- [ ] Canonical action state machine
- [ ] Goal lifecycle (accept → execute → terminal)
- [ ] Cancellation semantics
- [ ] Concurrency policy model
- [ ] Lifecycle interaction rules
- [ ] Deterministic unit tests

### Adapter: rosrs (production)
- [ ] Map core actions to ROS 2 action services/topics
- [ ] Lifecycle-gated goal acceptance
- [ ] Correct status stream behavior
- [ ] CLI compatibility (`ros2 action`)
- [ ] Cancellation race handling
- [ ] Shutdown safety

Actions are the **largest remaining parity blocker**.

---

## 4. Composition (Structural Parity)

### Core
- [ ] Define composition semantics (node ownership, lifetimes)
- [ ] Container vs application ownership model
- [ ] Deterministic startup/shutdown ordering

### Adapter: rosrs
- [ ] Implement composition service surface
  - LoadNode
  - UnloadNode
  - ListNodes
- [ ] Static composition (compile-time)
- [ ] Optional dynamic loading (C-ABI seam, documented)
- [ ] Launch system compatibility

Dynamic plugin loading is optional; service parity is mandatory.

---

## 5. Executors & Execution Model

### Core
- [ ] Define executor contract:
  - ownership
  - shutdown guarantees
  - callback ordering expectations
- [ ] Cancellation semantics
- [ ] Transition execution policy

### Adapter: rosrs
- [ ] Document supported executor patterns
- [ ] Ensure lifecycle transitions do not deadlock executors
- [ ] Clean shutdown guarantees (Ctrl+C, lifecycle shutdown)

No adapter-owned background threads.

---

## 6. Ecosystem Parity (Integration Layer)

### Required
- [x] ROS CLI compatibility
- [x] Launch integration
- [x] Nav2 lifecycle manager compatibility
- [ ] Action manager compatibility
- [ ] Composition manager compatibility

### CI / Packaging
- [x] ROS-free unit tests
- [x] ROS-native integration tests
- [ ] Explicit CI split documentation
- [ ] Contributor guidelines (what belongs where)

---

## 7. Feature-Gate Correctness (Cross-Cutting, Required)

All parity features MUST have explicit and correct feature-flag behavior.

This section tracks verification that feature flags:
- gate code correctly
- match documented behavior
- do not leak ROS dependencies unintentionally

### Requirements (normative)

For each parity feature (lifecycle, parameters, actions, composition, executors):

- [ ] Feature flags are explicitly defined (`ros2`, `lifecycle`, `parameters`, `actions`, etc.)
- [ ] Public APIs are unavailable when the feature is disabled (compile-time)
- [ ] Docs state which features are required for which capabilities
- [ ] Default feature set is intentional and documented
- [ ] Enabling one feature does not implicitly enable unrelated surfaces
- [ ] System tests are run with:
  - minimal feature set
  - full parity feature set

### Validation

- Cargo feature matrix build:
  - `--no-default-features`
  - `--features ros2`
  - `--features ros2,lifecycle`
  - `--features ros2,lifecycle,parameters`
  - full parity set
- Docs reviewed against actual feature flags

---
## 8. Message Packaging Strategy Review (rosrustext_msgs)

- [ ] Review long-term requirements for bundled ROS message crates (`rosrustext_msgs`).
  - Confirm which message packages are required for tooling parity (e.g. `lifecycle_msgs`, `bond`, `rcl_interfaces`).
  - Validate crates.io publishing constraints vs ROS workspace generation workflows.
  - Ensure bundled messages do not diverge semantically from upstream ROS IDL.
  - Document the criteria for:
    - keeping `rosrustext_msgs` internal-only,
    - promoting it to a stable public dependency, or
    - removing it once generator-based workflows are sufficient.
  - Decision must prioritize:
    - production parity,
    - user trust,
    - minimal ecosystem fragmentation.

This review is intentionally non-blocking and will be addressed once core parity
(lifecycle, parameters, actions, execution) stabilizes.

---

##  9. Lifecycle Proxy Review (rosrustext_lifecycle_proxy)

- [ ] Review `rosrustext_lifecycle_proxy` scope and role.
  - Determine whether the proxy should remain lifecycle-only or be generalized
    as a ROS management proxy (lifecycle, parameters, actions).
  - Validate that responsibilities do not overlap with adapter crates.
  - Confirm which ROS-facing services/topics are proxy-owned vs adapter-owned.
  - Update `rosrustext_lifecycle_proxy/README.md` to reflect the final scope,
    guarantees, and intended usage.

---

## 10. Documentation & Trust Surface

- [x] Canonical specs (lifecycle, parameters, actions, composition, executors, ecosystem)
- [x] Adapter parity matrices
- [ ] “Known limitations” consolidated doc
- [ ] Contributor decision guide (core vs adapter vs tutorial)
- [ ] One authoritative parity dashboard
- [ ] Update min examples for all features

---

## Definition of Done – Production Parity

Rust reaches **production-grade ROS 2 parity** when:

- Lifecycle, parameters, and actions behave identically to rclcpp from the outside
- `ros2` CLI and launch tools work without caveats
- Lifecycle managers (Nav2) orchestrate Rust nodes reliably
- Actions are lifecycle-safe and cancellable
- Executors shut down deterministically
- CI and packaging are reproducible
- Limitations are explicit, not accidental

At that point, Rust is no longer “supported” — it is **first-class**.
