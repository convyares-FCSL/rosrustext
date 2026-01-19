# rosrustext_roslibrust

**ROS 2 lifecycle parity for Rust via roslibrust + rosbridge.**

`rosrustext_roslibrust` is a ROS-facing adapter that projects the canonical
lifecycle semantics defined in `rosrustext_core` into **standard ROS 2 services
and topics**, using **roslibrust** over **rosbridge**.

Upstream transport:
- RosLibRust: https://github.com/RosLibRust/roslibrust

This crate exists to provide **production-grade lifecycle behavior today**,
even in environments where native RCL bindings are unavailable or undesirable.

Lifecycle services are **not** exposed directly by roslibrust alone.
They are projected onto the ROS graph by `rosrustext_lifecycle_proxy` over
rosbridge.

Required runtime stack:
- `rosbridge_websocket`
- `rosrustext_lifecycle_proxy`
- your `rosrustext_roslibrust` node (or the demo)

---

## Scope and status

| Area        | Tool parity | User parity |
|------------|------------|-------------|
| Lifecycle  | **Implemented** (services / events / bond / gating) | Partial |
| Parameters | Not implemented | Not implemented |
| Actions    | Not implemented | Not implemented |
| Executor   | Not implemented | Not implemented |

Lifecycle is the **only** supported feature in this adapter.

---

## Lifecycle parity (implemented)

The following ROS-facing lifecycle surface is fully implemented and verified
against real ROS tooling. These endpoints are provided by
`rosrustext_lifecycle_proxy` and are **not** native to roslibrust alone.

- Lifecycle services:
  - `change_state`
  - `get_state`
  - `get_available_states`
  - `get_available_transitions`
- `transition_event` publisher
  - exactly one event per accepted transition attempt
  - published after completion
- Busy-state rejection (deterministic)
- ErrorProcessing semantics
- Optional `/bond` heartbeats for Nav2 compatibility (via proxy)
- Activation-gated publishing and timers:
  - publishers and timers are suppressed while inactive
  - suppression is silent and deterministic

All lifecycle semantics are sourced from `rosrustext_core`.

---

## Rosbridge backend registration

In roslibrust/rosbridge transports, you must call
`register_lifecycle_backend_rosbridge` once at startup to expose the backend
for the proxy.

```rust
use std::sync::{Arc, Mutex};

use roslibrust::rosbridge::ClientHandle;
use rosrustext_roslibrust::lifecycle::LifecycleNode;
use rosrustext_roslibrust::transport::roslibrust::register_lifecycle_backend_rosbridge;

let lifecycle = Arc::new(Mutex::new(LifecycleNode::new(node_name, Box::new(MyCallbacks))?));
let client = ClientHandle::new(bridge_url).await?;
register_lifecycle_backend_rosbridge(&client, node_name, Arc::clone(&lifecycle)).await?;
```

---

## Compatibility and transport notes

- Transport uses **roslibrust 0.18.x** over **rosbridge**.
- DDS-level QoS knobs are not exposed by rosbridge and are therefore not tunable.
- rosbridge serialization limits apply (only supported message/service types).
- No `rosapi` or graph-introspection dependencies are introduced.
- `change_state` returns `success=true` once a transition is accepted;
  callback outcome is reflected in:
  - the final stable state
  - the `transition_event`
- Subscriptions are **not lifecycle-gated** in this adapter.
  Only publish and timer execution paths are gated.
- Shutdown may emit rclpy "context not valid" noise from rosbridge at teardown.

These behaviors match documented and tested ROS 2 expectations.

---

## What this crate is for

This adapter is intended for scenarios where:

- maximum ROS 2 **tooling compatibility** is required today
- lifecycle correctness matters more than raw performance
- rosbridge is already part of the system architecture
- native RCL bindings are unavailable, undesirable, or impractical

Typical use cases include:

- lifecycle-managed orchestration nodes
- supervisory or coordination components
- Nav2-integrated systems requiring bond compatibility
- mixed-language ROS systems where Rust nodes must behave identically
  to C++ and Python nodes

This crate is **not** intended to replace native RCL-based adapters.

---

## Relationship to other rosrustext crates

- `rosrustext_core`  
  Defines lifecycle semantics (transport-agnostic)

- `rosrustext_roslibrust`  
  Projects those semantics via rosbridge (this crate)

- `rosrustext_rosrs`  
  Native RCL (`rclrs`) adapter for ROS-workspace environments

All adapters share the same semantic core and parity goals.

---

## Build (ROS environment required)

```bash
cargo build \
  --manifest-path crates/rosrustext_roslibrust/Cargo.toml
````

A running rosbridge backend is required at runtime.

---

## Testing

System-level tests validate behavior against real ROS 2 tools:

```bash
# From the repository root (ROS environment sourced)
./scripts/test/roslibrust/run_all_tests.sh
```

These tests exercise:

* `ros2 lifecycle` CLI
* `ros2 service call` for `get_available_transitions`
* Python lifecycle managers
* Nav2 lifecycle manager (bond behavior)

See `scripts/test/roslibrust/README.md` for backend and environment requirements.

---

## Summary

`rosrustext_roslibrust` provides **reliable, production-grade lifecycle parity**
for Rust nodes in ROS 2 systems via rosbridge.

It is a pragmatic, well-tested adapter for environments where tooling
compatibility and correctness matter more than transport purity.
