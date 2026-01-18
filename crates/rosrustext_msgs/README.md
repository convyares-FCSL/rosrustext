# rosrustext_msgs

Bundled ROS 2 message definitions used by the `rosrustext` project.

This crate provides **pre-generated Rust message types** for a *small, curated
subset* of ROS 2 interface packages that are required for **tooling and lifecycle
parity**, without requiring end users to generate messages locally.

---

## What this crate contains

Generated Rust bindings for selected ROS 2 interface packages, currently:

- `lifecycle_msgs`
- `bond`
- `rcl_interfaces`
- `builtin_interfaces`
- `std_msgs` (minimal set)

These packages are required to expose **canonical ROS-facing services and topics**
used by:

- lifecycle management (`ros2 lifecycle`, Nav2)
- parameter services and events
- orchestration and supervision tools

---

## Why this crate exists

Publishing Rust crates that depend on ROS message types presents a tension:

- ROS 2 expects message types to be generated via `rosidl` and `colcon`.
- crates.io users expect `cargo build` to work without a ROS installation.

`rosrustext_msgs` exists as a **temporary compatibility layer** that allows:

- `rosrustext_core` and adapter crates to be published on crates.io
- docs.rs builds to succeed without ROS
- users to explore lifecycle/parameter semantics without setting up a ROS workspace

It is **not** intended to replace the standard ROS message generation workflow.

---

## Intended usage

This crate is:

- ✅ Used internally by `rosrustext_*` crates when publishing to crates.io
- ✅ Suitable for documentation builds and ROS-free compilation
- ⚠️ Optional when building in a full ROS workspace

When building in a ROS workspace with generated message crates available
(e.g. via `ros2_rust` / `rosidl_generator_rs`), downstream crates may instead rely
on those generated crates directly.

---

## Stability and guarantees

- Message definitions mirror upstream ROS 2 IDL at the time of generation.
- No semantic extensions or modifications are introduced.
- Updates are intentional and reviewed for parity impact.

However:

> This crate does **not** guarantee perpetual synchronization with upstream ROS
> message definitions.

For production systems that require strict alignment with a specific ROS
distribution, **using ROS-generated message crates from a workspace overlay is
recommended**.

---

## Future direction

`rosrustext_msgs` is intentionally conservative in scope.

As the Rust ROS ecosystem matures, this crate may:

- remain as an internal publishing aid,
- be reduced to a minimal set of lifecycle-critical interfaces, or
- be retired in favor of generator-based workflows once those are robust and
  user-friendly.

Any such change will be documented and reflected in the project’s parity specs
and TODO tracking.

---

## License

Apache-2.0
