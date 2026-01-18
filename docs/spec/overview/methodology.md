# rosrustext Methodology

## Purpose

`rosrustext` exists to help Rust participate in **production ROS 2 systems** with the same observable behavior, tooling compatibility, and operational guarantees as C++ (`rclcpp`) and Python (`rclpy`).

It does **not** replace existing Rust ROS libraries.

Instead, it provides:

* a **shared semantic reference** for ROS behavior
* **parity extensions** layered on top of existing Rust ROS stacks
* **guardrails** that prevent accidental divergence from ROS ecosystem expectations

The goal is not innovation for its own sake, but **compatibility, confidence, and correctness**.

---

## Context: Why methodology matters

Rust ROS development today builds on strong foundations:

* **`rclrs` / ros2_rust** — native RCL bindings and code generation
* **`roslibrust`** — rosbridge-based transport with fast iteration
* **ROS tooling** — lifecycle managers, launch, Nav2, CLI utilities

However, ROS 2 behavior is defined as much by **ecosystem conventions and tools** as by APIs. In practice, several structural challenges naturally arise when integrating a new systems language into ROS:

* ROS build/toolchain concerns (colcon, rosidl, Python dependencies) can leak into library design
* Features can be implemented incrementally without a single canonical semantic reference
* “Parity” can be claimed at the API level while diverging at the tooling/behavior level
* Tutorial or example code can grow ad-hoc semantics that should live in a shared library layer

None of these are bugs or failures — they are normal outcomes of ecosystem growth.

This methodology exists to **make boundaries explicit**, so Rust can reach production parity without fragmenting semantics, workflows, or expectations.

---

## Core working principle

> **Parity is judged by observable ROS behavior, not API similarity.**

If ROS tools (`ros2 lifecycle`, `ros2 param`, Nav2 managers, launch files) behave the same way, then parity exists — even if Rust APIs look different.

---

## Non-Negotiables

### 1) Separation of concerns

There are **two distinct worlds**, and they must not bleed into each other.

**Library repository**

* Pure Cargo builds
* No ROS build artifacts
* No generated message crates
* Builds without sourcing ROS

**ROS workspace**

* colcon + rosidl + Python dependencies
* message generation
* integration/system testing
* tooling validation

This separation is a **hard firewall**. It prevents drift, keeps crates publishable, and makes failures diagnosable.

---

### 2) Parity extension, not a rewrite

`rosrustext` **extends** existing Rust ROS libraries.

It does **not**:

* replace `rclrs`
* replace `roslibrust`
* reimplement ROS concepts
* invent new semantics

Instead, it:

* defines missing semantic layers
* maps those semantics onto existing transports
* documents intentional limitations when parity cannot be achieved

Upstream libraries remain the foundation.

---

### 3) One semantic truth

All ROS semantics are defined **once**, in a transport-agnostic form.

* `rosrustext_core` defines:

  * lifecycle semantics
  * parameter semantics
  * action semantics
  * error and recovery models
* Adapters project that truth into:

  * `rclrs`
  * `roslibrust`
  * future transports

No adapter is allowed to invent its own state machine or interpretation.

---

### 4) Minimal surface area

Prefer:

* explicit types
* explicit traits
* feature-gated modules
* visible control flow

Avoid:

* deep abstraction layers
* macros that hide ROS semantics
* “framework-ifying” ROS

ROS is already a framework. Rust code should make that explicit, not obscure it.

---

### 5) User experience matters

Rust users should not need to understand:

* colcon internals
* rosidl generator quirks
* Cargo patch mechanics
* transport-specific workarounds

Complexity is handled through:

* documentation
* templates
* clear workflows

—not by hiding behavior behind magic.

---

## Design rules

### A) Core crate is transport-agnostic

`rosrustext_core` contains:

* semantic state machines
* transition rules
* error models
* observability hooks
* zero ROS dependencies

It compiles and tests with **pure Rust**.

---

### B) Transport crates are adapters

Each adapter:

* maps ROS transport APIs to core semantics
* owns exactly what it must (services, publishers, timers)
* avoids introducing new state machines unless required by parity specs

Adapters differ in mechanics, **not meaning**.

---

### C) Generated messages are inputs, not artifacts

Message crates (`std_msgs`, `lifecycle_msgs`, custom interfaces):

* are generated in a ROS workspace
* are never committed to the library repo
* are consumed via workspace patches or path dependencies

This keeps crates.io clean and avoids version drift.

---

## Testing strategy

### 1) Core unit tests (no ROS)

* `cargo test`
* semantic correctness only
* deterministic and fast

---

### 2) Adapter integration tests (Rust)

* run against generated message crates
* validate adapter wiring
* still no ROS graph assumptions

---

### 3) System / tooling tests (ROS)

* CLI-driven (`ros2 lifecycle`, `ros2 param`, `ros2 action`)
* Python lifecycle managers
* Nav2 lifecycle manager
* launch behavior

Shell scripts are intentional:
they test **what operators actually use**, not internal APIs.

---

## Toolchain reality

ROS message generation depends on Python tooling.

Common failure modes:

* missing NumPy headers
* missing `lark`
* wrong Python interpreter used by colcon

Rule:

> Always be explicit about which Python environment colcon uses.

This is documented, not abstracted away.

---

## Decision log

When a decision affects structure, workflow, or semantics:

* it is recorded
* alternatives are noted
* rationale is explicit

This avoids re-litigation and accidental regressions.

---

## Stop conditions (hard)

Stop immediately if any of the following occur:

* generated ROS artifacts enter the library repo
* a new abstraction hides ROS semantics without a parity requirement
* users are forced to adopt a specific workspace layout
* core API surface grows without a corresponding spec requirement

---

## Guiding principle

> **Do not extend ROS semantics until Rust matches them.**

Parity first.
Extensions later.

This is how Rust becomes a **production-grade ROS language**, not a side path.
