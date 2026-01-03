# Lifecycle Parity – ros2_rust Adapter

This document tracks lifecycle parity for the **ros2_rust transport adapter**
(`rosrustext_ros2_rust` using `rclrs`).

Canonical reference:
- `docs/spec/lifecycle.md` (normative)

This file answers:
> “Given the ROS2 lifecycle spec, what will the ros2_rust adapter provide?”

---

## Dependency source

- Primary: `rclrs` (crate) from crates.io (repo `ros2-rust/ros2_rust` is a multi-crate workspace)
- Pin to git only if Jazzy-required APIs/bugs force it (document exact commit if so)

---

## Services (ROS-facing)

| Service | ROS Type | Status | Notes |
|------|---------|--------|------|
| `/<node>/change_state` | `lifecycle_msgs/srv/ChangeState` | 🚧 Planned | Native service server. **Must complete transition before responding** (no semantic drift). |
| `/<node>/get_state` | `lifecycle_msgs/srv/GetState` | 🚧 Planned | Native service server. |
| `/<node>/get_available_transitions` | `lifecycle_msgs/srv/GetAvailableTransitions` | 🚧 Planned | Native service server. |
| `/<node>/get_available_states` | `lifecycle_msgs/srv/GetAvailableStates` | 🚧 Planned | Native service server. |
| `/<node>/get_transition_graph` | `rosrustext_interfaces/srv/GetTransitionGraph` | 🚧 Planned | Custom introspection service (same as roslibrust adapter). |
| `create` | internal | ❌ Omitted | Wrapper-only concern. |
| `destroy` | internal | ❌ Omitted | Wrapper-only concern. |

**Design constraint:** `change_state` must not block the executor thread, yet must not respond early. This is the key transport/executor constraint to prove in `rclrs`.

---

## Topics

| Topic | ROS Type | Status | Notes |
|------|---------|--------|------|
| `/<node>/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | 🚧 Planned | Native publisher. Must match rclcpp observables (IDs/labels/timestamps). |
| `/bond` | `bond/msg/Status` | 🚧 Planned | Native publisher for Nav2 bond compatibility (same heartbeat strategy as roslibrust adapter). |

---

## Semantics (core truth projected through adapter)

| Aspect | Status | Notes |
|------|--------|------|
| Busy-state rejection | ✅ Core-provided | Adapter must surface correct failure response (no transport-specific “retry later” semantics). |
| Activation gating | ✅ Core-provided | `ActivationGate` owned by lifecycle node. |
| Publish suppression when inactive | ✅ Core-provided | Silent drop (as documented in roslibrust parity). |
| Timer suppression when inactive | ✅ Core-provided | Must be implementable without background spinner; either cancel timers on deactivate or guard in callback. |
| Shutdown from any state | ✅ Core-provided | Best-effort path to Finalized (adapter must wire to app shutdown model). |
| ErrorProcessing handling | ✅ Core-provided | No semantic drift; adapter must map to ROS-visible outcomes + transition events. |
| Fatal error policy | ✅ Core-provided | Adapter just reports/forces Finalized per core policy. |

---

## Callback execution model (transport-specific)

The `ros2_rust` adapter is a native `rclrs` node.

- **Lifecycle callbacks (core):** synchronous hooks (per `rosrustext_core::LifecycleCallbacks`), executed by wrapper logic.
- **Non-blocking rule:** lifecycle *service handlers* must not block the executor thread.
- **No adapter-owned spinner:** application owns the spin loop (like “normal C++”).
- **Implication:** the adapter must support a “transition work off-thread, completion observed in executor context, then respond” pattern **without** requiring Tokio.

> NOTE: The earlier claim “callbacks are strictly async” is incorrect for this project’s core contract and is removed.

---

## Transport-specific constraints (rclrs)

- **Executor:** application-provided. Adapter must work under typical `rclrs` executor patterns (single-threaded at minimum).
- **Service response timing:** must prove `rclrs` can respond after transition completion without blocking executor thread.
- **Naming/remapping:** standard ROS 2 remapping applies automatically.
- **Discovery:** should work out-of-the-box with `ros2 node list` / `ros2 lifecycle` (no proxy node).

---

## Known gaps / risks (must be proven)

- **ChangeState response mechanics:** can we defer response until transition completes without blocking executor thread?
- **Timer gating semantics:** best native strategy in `rclrs` without hidden background threads.
- **Bond:** confirm QoS + timing expectations for `nav2_lifecycle_manager` with native publisher.
- **Parameters:** lifecycle managers sometimes assume parameter presence; confirm whether adapter needs minimal parameter compatibility surface or can omit.

---

## Test layers (same philosophy)

- Core unit tests (Rust): `cargo test -p rosrustext_core`
- Adapter integration tests (Rust): `cargo test -p rosrustext_ros2_rust`
- System tests (ROS CLI + managers): extend `scripts/test/*` with native ros2_rust adapter runs (no proxy)

---

## Definition of Done (ros2_rust lifecycle)

Lifecycle is complete when a Rust node using `rosrustext_ros2_rust` is controllable by:

- `ros2 lifecycle get/set`
- Python lifecycle manager
- `nav2_lifecycle_manager` (bond enabled)

…with no semantic drift from `docs/spec/lifecycle.md`, and correct `transition_event` publication.
