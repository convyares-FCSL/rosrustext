# rosrustext_roslibrust

**ROS 2 lifecycle parity for Rust via roslibrust + rosbridge.**

`rosrustext_roslibrust` is a ROS-facing adapter that projects
`rosrustext_core` lifecycle semantics into standard ROS 2 services and topics
using **roslibrust** and **rosbridge**.

Upstream library:
- RosLibRust: https://github.com/RosLibRust/roslibrust

---

## What this crate provides

- Full ROS 2 lifecycle service surface:
  - `change_state`
  - `get_state`
  - `get_available_states`
  - `get_available_transitions`
- `transition_event` publisher
- Busy-state rejection
- ErrorProcessing semantics
- Bond heartbeat for Nav2 lifecycle manager compatibility
- Compatibility with:
  - `ros2 lifecycle`
  - Python lifecycle managers
  - `nav2_lifecycle_manager`

This adapter is **feature-complete for lifecycle parity**.

---

## What this crate is for

This crate is suitable when you want:

- maximum ROS 2 tooling compatibility today
- lifecycle correctness over raw performance
- a transport that works well with existing ROS graphs

It is commonly used for:
- orchestration nodes
- lifecycle-managed components
- Nav2-integrated systems

---

## Testing

System-level tests validate behavior against real ROS tools:

```bash
# From the repo root (ROS environment sourced)
./scripts/test/run_all_tests.sh
```
