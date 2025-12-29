# rclrust

`rclrust` is a small Rust library intended to provide **ROS 2–compatible lifecycle
and action semantics** without depending directly on `rclrs`.

Design goals:

- Match ROS 2 lifecycle behaviour closely enough to be controlled by standard
  lifecycle managers (Python, C++, or Rust).
- Separate **pure logic** from **ROS transport / bindings**.
- Prefer explicit state machines and enums over hidden framework magic.
- Be easy to copy into another repository and build as-is.

Structure:

- `rclrust_core`
  - ROS-agnostic logic
  - Lifecycle state machines
  - Action protocol core types
  - Config and logging helpers
- `rclrust_wrapper`
  - ROS-facing adapter layer
  - Built on top of `roslibrust`
  - Exposes ROS 2 lifecycle-compatible services and topics

Non-goals:

- Replacing `rclcpp` or `rclpy`
- Clever async abstractions
- Macro-heavy frameworks

This library is developed alongside a real industrial system
(OPC UA bridge, safety-critical context), and aims to stay realistic.
