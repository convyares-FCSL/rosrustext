# Ecosystem Parity Spec (ROS 2 Canonical)

This document defines the **ecosystem-level requirements** for a professional
ROS 2 client stack. It covers interoperability, tooling, testing, packaging,
and operational integration — not node-local semantics.

This spec applies **only** to native ROS 2 client libraries built on rcl/rmw
(e.g. rclcpp, rclpy, rclrs).

Related implementation matrices:
- `docs/adapters/ros2rust/ecosystem/parity.md`

---

## Scope

Ecosystem parity covers:

- ROS CLI and tooling compatibility
- Launch system integration
- Lifecycle manager compatibility (Nav2, system bringup)
- Build, packaging, and CI expectations
- Cross-language interoperability
- Operational conventions (logging, shutdown, failure modes)

Out of scope:
- Transport-bridged systems (rosbridge, web APIs)
- Experimental or non-ROS-native workflows

---

## 1) ROS CLI compatibility (normative)

A professional client stack MUST interoperate with standard ROS 2 tools.

Required commands (non-exhaustive):

- `ros2 node list/info`
- `ros2 topic list/info/echo`
- `ros2 service list/call`
- `ros2 param list/get/set/describe`
- `ros2 lifecycle get/set`
- `ros2 action list/info/send_goal/cancel`
- `ros2 launch` (indirectly, via launch files)

Observable behavior:
- Tools MUST not require language-specific flags or workarounds.
- Behavior MUST match rclcpp semantics unless explicitly documented.

---

## 2) Launch system integration

Nodes MUST be usable from standard ROS 2 launch files:

- Python launch descriptions
- lifecycle manager orchestration
- parameter YAML injection
- remapping rules
- composable vs standalone nodes (where applicable)

A Rust node must be launchable exactly like a C++ node.

---

## 3) Lifecycle manager compatibility (normative)

A professional stack MUST work with existing lifecycle managers:

- `nav2_lifecycle_manager`
- custom Python lifecycle managers
- CLI-driven orchestration

Requirements:
- Bond semantics must be correct (QoS, timing, state transitions).
- Transition rejection/acceptance must be deterministic.
- Managers must not deadlock or time out due to adapter behavior.

---

## 4) Cross-language interoperability

Rust nodes MUST interoperate seamlessly with:

- rclcpp nodes
- rclpy nodes
- standard ROS 2 tools

This includes:
- correct QoS defaults
- correct message/service/action types
- correct namespacing and discovery behavior

A Rust node must be indistinguishable on the ROS graph.

---

## 5) Build and packaging (normative)

A professional stack MUST support:

### Crates.io
- Clean `cargo publish`
- No ROS environment required for docs.rs builds
- Clear feature flags for ROS-dependent functionality

### ROS workspace
- `colcon build` with Rust nodes
- Integration with existing C++/Python packages
- Reproducible builds in CI

The split between **crate-level code** and **ROS-workspace code** MUST be clear
and documented.

---

## 6) Testing model (normative)

Testing MUST be split cleanly:

### Unit tests (ROS-free)
- Core semantics
- State machines
- Validation logic
- Error handling

Executed via:
- `cargo test`
- CI without ROS installed

### Integration tests (ROS-native)
- CLI-driven tests
- Launch tests
- Lifecycle/action/parameter orchestration

Executed via:
- ROS workspace
- `launch_testing`
- shell/python scripts invoking `ros2` CLI

No ROS-dependent tests should run on crates.io CI.

---

## 7) Failure and shutdown behavior (normative)

Operational expectations:

- Ctrl+C shuts down cleanly
- Executors unblock deterministically
- Lifecycle cleanup always runs
- No orphan threads/tasks
- No background activity after shutdown

Failure behavior must be:
- observable
- attributable to a node
- non-panicking

---

## 8) Documentation and user trust

A professional ecosystem requires:

- Clear parity documentation (this repo)
- Explicitly documented limitations
- Examples that reflect production patterns, not demos
- No “magic” behavior that surprises operators

---

## Definition of Done (Ecosystem)

Ecosystem parity is complete when:

- Rust nodes work with ROS tooling and launch without special handling
- Lifecycle, parameters, and actions integrate with existing managers
- CI cleanly separates ROS-free and ROS-native tests
- Rust nodes are operationally indistinguishable from rclcpp nodes
