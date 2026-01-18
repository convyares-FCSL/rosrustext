# rosrustext

**First-class Rust feature parity for ROS 2**

`rosrustext` is a Rust project whose goal is to enable **full, first-class Rust participation in ROS 2 systems**, on equal footing with C++ and Python.

The long-term objective is:

> **A ROS 2 system where Rust, C++, and Python nodes are interchangeable**,
> work with the same tools, follow the same semantics,
> and can be composed freely in production systems.

This project builds on existing Rust ROS libraries:
* RosLibRust — https://github.com/RosLibRust/roslibrust  
* ros2_rust / rclrs — https://github.com/ros2-rust/ros2_rust  

Lifecycle support is the **first completed feature**, not the end goal.

---

## Why this exists

ROS 2 already has mature client libraries for:

* C++ (`rclcpp`)
* Python (`rclpy`)

Rust support exists, but is distributed across:

* rosbridge-based solutions
* native RCL bindings
* partial or uneven feature coverage

In practice, this can lead to a gap where:

* Rust nodes compile and publish messages,
* but fail under **real ROS 2 tooling**, such as:

  * lifecycle managers
  * Nav2
  * orchestration tools
  * production launch patterns

`rosrustext` exists to close that gap by providing:

* **Shared semantic truth in Rust**
* **Multiple transport adapters**
* **Observable parity with ROS 2 expectations**

Parity is judged by **external behavior under ROS tools**, not by API similarity.

---

## Project vision

### What success looks like

A user should be able to:

* Write a Rust node
* Choose a Rust ROS adapter (`roslibrust`, `rosrs`, etc.)
* Drop that node into an existing ROS 2 system
* Use:

  * `ros2 lifecycle`
  * Python lifecycle managers
  * C++ lifecycle managers (Nav2)
  * Standard launch files

And have it behave **indistinguishably** from a C++ or Python node.

No custom glue.  
No disabling safety checks.  
No “Rust-only” behavior.

---

## Design principles

* **Parity before convenience**  
  Observable behavior matters more than API ergonomics.
* **One semantic core**  
  ROS meaning is defined once, not per transport.
* **Multiple adapters, same truth**  
  Different Rust ROS stacks share the same semantics.
* **Transport isolation**  
  rosbridge vs native RCL differences must not leak upward.
* **Deterministic and explicit**  
  State machines, not hidden side effects.
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

Planned scope:

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
* Verified with:

  * `ros2 lifecycle`
  * Python lifecycle managers
  * `nav2_lifecycle_manager`
* Includes:

  * Lifecycle proxy
  * Bond heartbeat

#### `rosrustext_rosrs` *(ROS workspace only)*

* Uses native RCL bindings (`rclrs`)
* Lifecycle services + `transition_event`
* Parameter support via `rclrs::vendor::rcl_interfaces`
* Bond heartbeat behind feature `bond` (Nav2 QoS)
* Not published on crates.io due to ROS-generated message crate dependencies

Adapters are **replaceable**, not competing.

---

## Lifecycle: first completed feature

Lifecycle was implemented first because it is:

* Semantically rich
* Tooling-heavy
* Widely relied upon (Nav2, orchestration)
* A strong test of parity claims

Lifecycle behavior is verified under:

* ROS CLI
* Python lifecycle managers
* C++ / Nav2 lifecycle managers

Parity is documented via specs and validated by system-level tests.

---

## Testing philosophy

Testing is layered by intent:

1. **Core unit tests (Rust)**  
   Semantic correctness without ROS.
2. **Adapter integration tests (Rust)**  
   Adapter contracts and state reconciliation.
3. **System tests (CLI-level)**  
   Real ROS tools (`ros2 lifecycle`, Nav2, Python managers).

Shell scripts are intentional:  
they test **what operators actually run**, not mocked APIs.

---

## Status

* Lifecycle parity (roslibrust): **complete**
* roslibrust adapter: **complete**
* Nav2 compatibility (roslibrust): **verified**
* rosrs adapter (lifecycle): **complete (ROS workspace only)**
* Parameters, actions, execution: **in progress**

---

## What this project is *not*

* ❌ A replacement for `rclcpp`, `rclpy`, `roslibrust`, or `ros2_rust`
* ❌ A single Rust ROS client library
* ❌ A macro-heavy abstraction framework
* ❌ A green-field ROS reimplementation

This project is about **compatibility, correctness, and confidence**.

---

## Context

This project is developed alongside real industrial systems
(OPC UA bridges, safety-critical control),
where Rust is chosen for correctness and failure containment —
but must still operate fully inside the ROS 2 ecosystem.

`rosrustext` exists so choosing Rust does **not** mean opting out of ROS 2.
