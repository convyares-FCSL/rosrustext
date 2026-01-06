
---

## `rosrustext_rosrs/README.md` (dev-only crate)

```markdown
# rosrustext_rosrs

**Native `rclrs` lifecycle adapter for `rosrustext_core`.**

This crate provides a native ROS 2 (`rclrs`) adapter that maps
`rosrustext_core` lifecycle semantics onto the ROS 2 RCL API.

Upstream library:
- ros2-rust / rclrs: https://github.com/ros2-rust/ros2_rust

---

## Status

✔ Lifecycle parity **complete**  
✔ Transition events (success / failure / error)  
✔ Busy-state rejection  
✔ Non-blocking ChangeState semantics  
✔ Bond heartbeat for Nav2 (normative QoS)  
✔ Transition graph introspection  

---

## Dev-workspace only

**This crate is not published on crates.io.**

Reason:
- It depends on ROS 2 message crates (`lifecycle_msgs`, `bond`, etc.)
- Those crates are generated in a **colcon workspace**, not available on crates.io

As a result:
- `publish = false`
- excluded from the root Cargo workspace
- intended to be built from a ROS dev workspace (`dev_ws`)

---

## Build (dev_ws)

```bash
cargo build \
  --manifest-path crates/rosrustext_rosrs/Cargo.toml \
  --features ros2,bond,transition_graph
