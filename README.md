# rosrustext

**First-class Rust feature parity for ROS 2**

`rosrustext` is a Rust project whose goal is to enable **full, first-class Rust participation in ROS 2 systems**, on equal footing with C++ and Python.

The long-term objective is:

> **A ROS 2 system where Rust, C++, and Python nodes are interchangeable**,
> work with the same tools, follow the same semantics,
> and can be composed freely in production systems.

Lifecycle support is the **first completed feature**, not the end goal.

---

## Why this exists

ROS 2 already has mature client libraries for:

* C++ (`rclcpp`)
* Python (`rclpy`)

Rust support exists, but is fragmented across:

* rosbridge-based solutions
* native RCL bindings
* incomplete or uneven feature coverage

This creates a gap:

* Rust nodes often *compile* and *publish messages*,
  but **fail under real ROS 2 tooling**:

  * lifecycle managers
  * Nav2
  * orchestration tools
  * production launch patterns

`rosrustext` exists to close that gap by providing:

* **Shared semantic truth in Rust**
* **Multiple transport adapters**
* **Observable parity with ROS 2 expectations**

---

## Project vision

### What success looks like

A user should be able to:

* Write a Rust node
* Choose a Rust ROS adapter (`roslibrust`, `ros2_rust`, etc.)
* Drop that node into an existing ROS 2 system
* Use:

  * `ros2 lifecycle`
  * Python lifecycle managers
  * Nav2
  * Standard launch files
* And have it behave **indistinguishably** from a C++ or Python node

No custom glue.
No disabling safety checks.
No “Rust special cases.”

---

## Design principles

* **Parity before convenience**
  Behavior matters more than API ergonomics.
* **One semantic core**
  ROS meaning is defined once, not per transport.
* **Multiple adapters, same truth**
  Different Rust ROS stacks share the same semantics.
* **Transport isolation**
  rosbridge vs native RCL differences must not leak upward.
* **Deterministic and explicit**
  State machines, not callbacks and side effects.
* **Industrial-grade observability**
  Failures must be visible, diagnosable, and predictable.

---

## Architecture overview

`rosrustext` is deliberately layered.

### `rosrustext_core`

**Semantic truth (ROS-agnostic)**

Defines **what ROS concepts mean**, independently of transport.

Currently implemented:

* ROS 2 lifecycle state machine
* Transition graph
* Error and recovery semantics
* Activation gating
* Deterministic transition handling

Future scope:

* Actions
* Parameters
* Node orchestration semantics
* Execution and cancellation models

This crate contains **no ROS messages, no executors, no async assumptions**.

---

### Adapter crates (ROS-facing)

Adapters map the core semantics onto specific Rust ROS stacks.

#### `rosrustext_roslibrust`

* Uses `roslibrust` + rosbridge
* Works today with:

  * `ros2 lifecycle`
  * Python lifecycle managers
  * `nav2_lifecycle_manager`
* Includes:

  * Lifecycle proxy
  * Bond heartbeat
  * Transition graph service

#### `rosrustext_ros2_rust` *(planned)*

* Uses native RCL bindings
* Same lifecycle surface
* Same tests
* Same semantics
* Different transport constraints

Adapters are **replaceable**, not competing.

---

## Lifecycle: first completed feature

Lifecycle is the first feature implemented end-to-end because it is:

* Semantically rich
* Tooling-heavy
* Widely relied upon (Nav2, orchestration)
* A good stress test for parity claims

Current lifecycle support includes:

* All standard ROS 2 lifecycle services
* Transition events
* Transition graph introspection
* Busy-state rejection
* ErrorProcessing semantics
* Bond compatibility for Nav2
* Verified with:

  * ROS CLI
  * Python lifecycle manager
  * Nav2 lifecycle manager

Lifecycle is **done**, but the project is **not lifecycle-only**.

---

## Testing philosophy

Testing is layered by intent:

1. **Core unit tests (Rust)**
   Verify semantic correctness without ROS.
2. **Adapter integration tests (Rust)**
   Verify adapter contracts and state reconciliation.
3. **System tests (CLI-level)**
   Shell scripts that behave like real users and CI:

   * `ros2 lifecycle`
   * Python managers
   * Nav2

Shell scripts are intentional:
they test **real ROS behavior**, not mocked APIs.

---

## What this project is *not*

* ❌ A replacement for `rclcpp` or `rclpy`
* ❌ A single Rust ROS client library
* ❌ A macro-driven abstraction layer
* ❌ A green-field ROS reimplementation

This is about **compatibility, correctness, and confidence**.

---

## Status

* Lifecycle parity: **complete**
* roslibrust adapter: **complete**
* Nav2 compatibility: **verified**
* ros2_rust adapter: **next**
* Actions, parameters, execution: **planned**

See `TODO.md` and parity documents for tracked work.

---

## Context

This project is developed alongside real industrial systems
(OPC UA bridges, safety-critical control),
where Rust is chosen for correctness and failure containment —
but must still operate inside the ROS 2 ecosystem.

`rosrustext` exists so choosing Rust does **not** mean opting out of ROS 2.
