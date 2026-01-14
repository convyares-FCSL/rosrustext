# rosrustext_core

**ROS-agnostic lifecycle and parameter semantics for ROS 2, implemented in Rust.**

`rosrustext_core` defines the *canonical lifecycle state machine* and parameter
semantics used by the `rosrustext` adapter crates.  
It contains **no ROS messages, no executors, and no transport-specific code**.

This crate is intended to be:

- deterministic
- testable without ROS
- reusable across multiple Rust ROS transports

Scope:

- Lifecycle state machine semantics (implemented)
- Parameter store semantics (implemented)
- Action state machine semantics (planned)
- Executor semantics (planned)

---

## What this crate provides

- ROS 2 lifecycle primary and transition states
- Canonical transition graph and validation
- Busy-state rejection semantics
- Error and ErrorProcessing handling
- Activation gating semantics
- Parameter value/descriptor types and change records
- Deterministic unit tests for lifecycle semantics

This crate answers the questions:

> "What does a ROS 2 lifecycle mean, independent of how it is transported?"
>
> "What do ROS 2 parameters mean, independent of how they are transported?"

---

## What this crate does *not* do

- ❌ No ROS messages
- ❌ No executors or async runtimes
- ❌ No transport assumptions (rosbridge, rclrs, etc.)
- ❌ No CLI or node abstractions

Those concerns live in adapter crates.

---

## How it is used

`rosrustext_core` is consumed by adapter crates such as:

- **rosrustext_roslibrust**  
  (roslibrust + rosbridge adapter)  
  https://crates.io/crates/rosrustext_roslibrust

- **rosrustext_rosrs**  
  (native rclrs adapter)  
  https://crates.io/crates/rosrustext_rosrs

All adapters project the same semantic truth into ROS-facing services and topics.

---

## Testing

This crate is fully testable without ROS:

```bash
cargo test -p rosrustext_core
