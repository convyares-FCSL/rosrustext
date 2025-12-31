# rclrust

`rclrust` is a small Rust library that provides **ROS 2–compatible lifecycle
and action semantics** without depending on `rclrs`.

The goal is **behavioral parity with ROS 2 lifecycle nodes**
(`rcl_lifecycle` + `rclcpp_lifecycle`), while remaining explicit,
inspectable, and suitable for safety-critical systems.

---

## Design goals

- Match ROS 2 lifecycle behavior closely enough to be controlled by:
  - Python lifecycle managers
  - C++ lifecycle managers
  - Future Rust tooling
- Separate **pure lifecycle logic** from **ROS transport and policy**.
- Prefer explicit state machines and enums over framework magic.
- Avoid macros, implicit globals, or hidden executors.
- Be easy to vendor into another repository and build as-is.

---

## Structure

### `rclrust_core`
**ROS-agnostic logic only**

- Lifecycle state machine
- Explicit transition semantics
- Error classification and payloads
- Activation gating primitive
- Action protocol (future)

No ROS messages, no async runtime assumptions, no transport.

### `rclrust_wrapper`
**ROS-facing adapter layer**

- LifecycleNode abstraction
- Managed publishers and timers
- Mapping between core semantics and ROS2 services
- Built on top of `roslibrust` (rosbridge)

Transport concerns live here.
Policy decisions live here.

---

## Non-goals

- Replacing `rclcpp` or `rclpy`
- Recreating rcl internals in Rust
- Async “frameworks” or opinionated executors
- Macro-heavy DSLs

---

## Context

This library is developed alongside a real industrial system
(OPC UA bridge, safety-critical hydrogen refuelling),
and is designed to stay boring, explicit, and debuggable.
