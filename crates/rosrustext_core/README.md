# rosrustext_core

**Canonical, ROS-agnostic lifecycle and parameter semantics for ROS 2, implemented in Rust.**

`rosrustext_core` defines the **semantic meaning** of core ROS 2 concepts
independently of any transport, executor, or client library.

It contains **no ROS messages, no executors, and no transport-specific code**.

This crate is intended to be:

- deterministic
- testable without ROS
- reusable across multiple Rust ROS transports
- stable as a semantic foundation

---

## Scope

Implemented:

- ROS 2 lifecycle state machine semantics
- Canonical lifecycle transition graph and validation
- Busy-state rejection rules
- Error and ErrorProcessing semantics
- Activation gating rules
- Parameter value, descriptor, and change semantics
- Deterministic unit tests for lifecycle behavior

Planned:

- Action state machine semantics
- Executor and execution-model semantics
- Cancellation and shutdown coordination rules

---

## What this crate provides

`rosrustext_core` answers questions like:

> *What does a ROS 2 lifecycle state mean, independent of how it is transported?*  
> *When is a transition valid or invalid?*  
> *What does “busy” mean, semantically?*  
> *What does it mean for a parameter update to be accepted, rejected, or atomic?*

Concretely, it provides:

- Lifecycle primary and transition state definitions
- A single canonical transition table
- Transition validation and outcome rules
- Error handling and recovery semantics
- Parameter value and descriptor types
- Parameter change records and ordering guarantees

All logic is expressed without assuming:
- DDS
- rcl
- rosbridge
- async runtimes
- executors or spinners

---

## What this crate does *not* do

This crate intentionally does **not** include:

- ❌ ROS messages or services
- ❌ Parameter or lifecycle service servers
- ❌ Executors or threading models
- ❌ Node abstractions or CLI helpers
- ❌ Transport assumptions (rclrs, roslibrust, rosbridge, etc.)

Those concerns belong in **adapter crates**.

---

## How it is used

`rosrustext_core` is consumed by adapter crates that map these semantics
onto real ROS 2 transports.

Current adapters include:

- **rosrustext_roslibrust**  
  roslibrust + rosbridge adapter  
  https://crates.io/crates/rosrustext_roslibrust

- **rosrustext_rosrs**  
  native rclrs adapter  
  https://crates.io/crates/rosrustext_rosrs

All adapters project the **same semantic truth**
into ROS-facing services, topics, and behaviors.

---

## Who should depend on this crate

This crate is intended for:

- Adapter authors building Rust ROS 2 integrations
- Infrastructure code that needs lifecycle or parameter semantics
- Testing and validation tools
- Projects that need deterministic, ROS-agnostic behavior definitions

It is **not** intended for:
- Application-level node authors
- Users looking for a full ROS client library

Most users should depend on an adapter crate instead.

---

## Stability and evolution

- Lifecycle and parameter semantics are considered **stable**
- Changes are expected to be **additive**, not breaking
- New features (actions, execution) will follow the same
  spec-first, parity-driven approach

Semantic changes are treated as **breaking changes** and versioned accordingly.

---

## Testing

This crate is fully testable without ROS installed:

```bash
cargo test -p rosrustext_core
````

Tests validate:

* transition validity
* error handling paths
* busy-state rejection
* deterministic state resolution