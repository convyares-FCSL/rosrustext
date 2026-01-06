# rosrustext_core

**ROS-agnostic lifecycle semantics for ROS 2, implemented in Rust.**

`rosrustext_core` defines the *canonical lifecycle state machine* and transition
semantics used by the `rosrustext` adapter crates.  
It contains **no ROS messages, no executors, and no transport-specific code**.

This crate is intended to be:

- deterministic
- testable without ROS
- reusable across multiple Rust ROS transports

---

## What this crate provides

- ROS 2 lifecycle primary and transition states
- Canonical transition graph and validation
- Busy-state rejection semantics
- Error and ErrorProcessing handling
- Activation gating semantics
- Deterministic unit tests for all lifecycle paths

This crate answers the question:

> “What does a ROS 2 lifecycle *mean*, independent of how it is transported?”

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
  https://github.com/RosLibRust/roslibrust

- **rosrustext_rosrs**  
  (native rclrs adapter, dev workspace only)  
  https://github.com/ros2-rust/ros2_rust

All adapters project the same semantic truth into ROS-facing services and topics.

---

## Testing

This crate is fully testable without ROS:

```bash
cargo test -p rosrustext_core
