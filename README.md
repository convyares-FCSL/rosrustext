# rosrustext

`rosrustext` is a small Rust library that provides **ROS 2–compatible lifecycle
semantics** without depending on `rclrs`.

The goal is **observable behavioral parity with ROS 2 lifecycle nodes**
(`rcl_lifecycle` / `rclcpp_lifecycle`), while remaining:

- explicit
- testable without ROS
- transport-agnostic at the core
- suitable for safety-critical systems

This is **not** a ROS client library.  
It is a lifecycle model that can be *controlled by* ROS.

---

## Design goals

- Match ROS 2 lifecycle behavior closely enough to be driven by:
  - Python lifecycle managers
  - C++ lifecycle managers
  - Future Rust tooling
- Keep lifecycle logic **pure and deterministic**
- Separate **semantics** from **transport and policy**
- Prefer explicit state machines and enums over hidden callbacks
- Avoid macros, implicit globals, or background executors
- Be easy to vendor into another repository and build as-is

---

## Architecture

The crate is split deliberately along semantic boundaries.

### `rosrustext_core`
**ROS-agnostic lifecycle truth**

Contains:

- Lifecycle state machine (primary + transition states)
- Explicit transition semantics
- Callback result model (Success / Failure / Error)
- Error processing and recovery rules
- Activation gating primitive

Properties:

- No ROS messages
- No async runtime assumptions
- No transport or threading
- Fully unit-testable in pure Rust

This crate defines *what lifecycle means*.

---

### `rosrustext_roslibrust`
**ROS-facing adaptation layer**

Contains:

- `LifecycleNode` abstraction
- Mapping of ROS lifecycle services to core transitions
- Activation-gated publishers
- Activation-gated timers
- Transport adapters (currently `roslibrust` / rosbridge)

Properties:

- All ROS concepts are isolated here
- Transport is behind feature flags
- Policy decisions (logging, shutdown behaviour) live here

This crate defines *how lifecycle is exposed to ROS*.

---

## Lifecycle compatibility

The lifecycle model mirrors ROS 2 concepts:

- Primary states:
  - Unconfigured
  - Inactive
  - Active
  - Finalized
- Transitional states:
  - Configuring
  - Activating
  - Deactivating
  - CleaningUp
  - ShuttingDown
  - ErrorProcessing
- Transition IDs aligned with ROS lifecycle services
- Busy-state rejection
- Deterministic shutdown from all primary states

Compatibility is verified by:
- Unit tests against all transition paths
- Live control via `ros2 lifecycle set`
- Interaction with Python and C++ lifecycle managers (in progress)

---

## What this is **not**

- ❌ A replacement for `rclcpp` or `rclpy`
- ❌ A Rust reimplementation of `rcl`
- ❌ An async framework or executor
- ❌ A macro-driven DSL

The intent is **boring correctness**, not convenience magic.

---

## Current transport support

- `roslibrust` via rosbridge (WebSocket)
- Lifecycle services exposed over rosbridge via a Rust proxy tool
- Publisher and timer gating enforced in Rust

Additional transports can be added without touching `rosrustext_core`.

---

## Lifecycle proxy tool (rosbridge)

Lifecycle-aware roslibrust nodes expose **backend** endpoints under a private namespace:

- `/<target>/_rosrustext/change_state`
- `/<target>/_rosrustext/get_state`
- `/<target>/_rosrustext/get_available_states`
- `/<target>/_rosrustext/get_available_transitions`
- `/<target>/_rosrustext/transition_event`

The Rust proxy tool bridges those to standard ROS 2 lifecycle endpoints so
`ros2 lifecycle` works without glue code:

- `/<target>/change_state`
- `/<target>/get_state`
- `/<target>/get_available_states`
- `/<target>/get_available_transitions`
- `/<target>/transition_event`

Tool crate:
`crates/rosrustext_roslibrust/tools/rosrustext_lifecycle_proxy`

---

## Scripts (local validation)

From the repo root:

- `./scripts/run_rosbridge.sh`
- `./scripts/run_backend.sh`
- `./scripts/run_proxy.sh`
- `./scripts/run_lifecycle_test.sh`
- `./scripts/run_all.sh` (single-terminal end-to-end run)

`run_all.sh` starts rosbridge, the backend app, the proxy, runs lifecycle
commands, then shuts everything down cleanly (SIGINT).

---

## Status

- Core lifecycle: **implemented and fully tested**
- Wrapper lifecycle services: **implemented (via proxy)**
- Graceful shutdown semantics: **implemented**
- Actions: **planned**
- Parameters: **planned**

See `TODO.md` for a parity-oriented task breakdown.

---

## Context

This library is developed alongside a real industrial control system
(OPC UA bridge, hydrogen refuelling),
where correctness, observability, and controlled failure modes matter
more than API sugar.

If something feels explicit or verbose, it is probably intentional.
