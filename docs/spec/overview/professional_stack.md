# Professional Rust ROS 2 Stack – Engineering Parity Spec

This document defines what a **professional-grade Rust ROS 2 stack** looks like when the goal is **parity with rclcpp / rclpy**, not experimentation or ecosystem extensions.

The focus is on **engineering capability** and **observable behavior**, not robot-specific domains (navigation, mapping, sensors, simulators).

The intent is:

> *rclrs + rosrustext_rclrs should be a drop-in professional alternative to rclcpp.*

---

## Scope and Non-Goals

### In scope

* Lifecycle-managed nodes
* Parameters (static + dynamic)
* Actions (long-running control flows)
* Composition and execution
* Fault handling and recovery
* Observability and operability
* Packaging, testing, and reproducibility

### Explicitly out of scope

* Robot-specific stacks (Nav2 internals, SLAM, perception, Gazebo)
* Custom ROS message definitions for core semantics
* Non-standard ROS interfaces
* Reinventing ROS tooling

Parity is achieved by **matching observable ROS 2 behavior**, not by copying APIs verbatim.

---

## ROS Interface Binding Strategy (Normative)

For ROS-native management APIs (including but not limited to
`lifecycle_msgs`, `rcl_interfaces`, and `composition_interfaces`), the Rust stack **must provide pre-generated, versioned Rust bindings** as part of the stack.

* Users **must not** be required to run IDL generators to author nodes.
* Bindings must match the canonical ROS 2 IDL definitions exactly.
* Build-time code generation is **optional and opt-in**, and may be provided for advanced or experimental use cases.
* The production contract lives in the library, not in user build scripts.

This preserves a stable engineering surface while keeping crates.io usable and maintainable.

---

## Parity Vocabulary

### Tool parity

ROS tools and other nodes observe **identical behavior** compared to rclcpp:

* CLI commands work (`ros2 lifecycle`, `ros2 param`, `ros2 action`)
* Services/topics exist with correct names and types
* State transitions and rejections behave predictably
* No hidden or undocumented side effects

### User parity

Authoring nodes in Rust feels **structurally comparable** to rclcpp:

* Clear lifecycle hooks
* Declarative parameters
* Action servers/clients with well-defined state
* Errors are explicit and testable

User APIs may differ stylistically, but **semantics must match**.

---

## 1. Lifecycle (Foundational Capability)

A professional stack **must** treat lifecycle as a first-class contract.

### Required behavior (normative)

* Nodes start in **Unconfigured**
* Resources are allocated **only** during Configure
* Data flow is enabled **only** during Activate
* Deactivate pauses execution without destroying resources
* Cleanup releases all resources deterministically
* Shutdown is always safe and idempotent

### Engineering requirements

* Lifecycle state is not cosmetic — it **gates execution**
* Timers, publishers, subscriptions, and actions are inert unless Active
* Transition callbacks are single-flight and reject re-entry
* Errors during transitions are handled explicitly (`on_error`)

### Tool parity

* `ros2 lifecycle get/set` works exactly as with rclcpp
* Lifecycle managers can orchestrate nodes reliably

---

## 2. Parameters (Configuration Surface)

Parameters define how a system is configured and tuned at runtime.

### Required surface

* All standard ROS 2 parameter services
* `parameter_events` topic with correct semantics
* Declared vs undeclared behavior must be explicit

### Required semantics

* Non-atomic sets may partially succeed
* Atomic sets are all-or-nothing
* Rejected updates do not modify state
* Accepted updates emit exactly one event
* Ordering is preserved

### Engineering expectations

* Parameter declaration is explicit and typed
* Validation logic is testable
* Dynamic updates do not require polling
* Parameter handling is lifecycle-aware

If the underlying transport cannot reject parameter updates at set-time, the limitation **must be documented** and observable behavior clearly specified.

---

## 3. Actions (Long-Running Control)

Actions are **not optional** for professional systems.

They represent goals that:

* Span time
* Produce feedback
* Can be canceled or aborted

### Canonical ROS 2 action model

* Goal
* Feedback
* Result
* Cancel
* Status

### Required lifecycle interaction

A lifecycle-managed action server must:

* Reject goals unless the node is **Active**
* Abort or cancel in-flight goals on Deactivate or Cleanup
* Never leak tasks/futures across lifecycle transitions
* Shut down deterministically

This behavior exists implicitly in rclcpp because lifecycle governs execution.

In Rust, it must be **explicit and auditable**.

### Tool parity

* `ros2 action list/info/send/cancel` works
* Status and feedback behave predictably

---

## 4. Composition and Execution Model

Professional systems are composed, not monolithic.

### Required capabilities

* Multiple nodes per process
* Clear ownership of executors and spinning
* Deterministic startup and shutdown order
* No hidden background threads

### Engineering expectations

* Executor ownership is explicit (app-owned or container-owned)
* Lifecycle transitions do not deadlock the executor
* Cleanup does not depend on drop-order accidents

### Note on Composition Parity

Composition parity refers to **ROS-visible container behavior**:
the service surface, node lifecycle, and executor semantics exposed to tools such as `ros2 component`.

Runtime loading of nodes from shared libraries is **not required** for parity.
If implemented, it may involve a deliberate C-ABI boundary due to Rust ABI stability constraints.

---

## 5. Observability and Operability

A system that cannot be observed cannot be operated.

### Required capabilities

* Structured logging with consistent fields
* Clear logs for lifecycle transitions
* Action goal acceptance/rejection visibility
* Parameter change visibility

### Professional expectations

* Logs are machine-parsable
* Failures are attributable to a node and lifecycle phase
* Operators can correlate:

  * lifecycle transitions
  * parameter updates
  * action outcomes

---

## 6. Fault Model and Recovery

Failures must be **designed for**, not patched over.

### Required behavior

* Clear error taxonomy (core vs transport vs user logic)
* User callback failures are contained
* Lifecycle `on_error` is meaningful
* Recovery paths are documented

### Engineering expectations

* No silent failure
* No panic-driven control flow
* Cleanup always runs, even after errors

---

## 7. Packaging, Reproducibility, and CI

Professional systems must build reliably.

### Required structure

* Crates publish cleanly to crates.io
* docs.rs builds without ROS installed
* ROS-native integration tests run in a dev workspace

### CI split

* ROS-free unit tests (core semantics)
* ROS-native integration tests (tool parity)

This split prevents accidental coupling to the build environment.

---

## Client Library Capability Assumptions

This spec assumes a client library capable of enforcing:

* Lifecycle-aware execution gating
* Service-level parameter handling
* Action goal state ownership

Where a transport or client library cannot enforce these semantics
(e.g. rosbridge-based stacks), deviations **must be documented**, and the implementation is considered **tool-parity only**, not full production parity.

---

## 8. Definition of Done (Professional Stack)

A Rust ROS 2 stack reaches professional parity when:

* Lifecycle, parameters, and actions behave identically to rclcpp from the outside
* ROS CLI tools work without caveats
* Resources are allocated, gated, and cleaned deterministically
* Long-running goals are lifecycle-safe
* Failures are observable and recoverable
* The system can be built, tested, and shipped reproducibly

---

## Guiding Principle

> **Do not extend ROS semantics until Rust matches them.**

Parity first.
Extensions later.