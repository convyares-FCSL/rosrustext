# Composition Parity Spec (ROS 2 Canonical)

This document defines the *canonical* ROS 2 **composition / components** surface area and semantics.
It is transport-agnostic and focuses on **observable behavior** (tools + other nodes), not any specific client library API.

Related implementation matrices:
- `docs/adapters/ros2rust/composition/parity.md`
- `docs/adapters/roslibrust/composition/parity.md`

---

## Terminology

**Composition / Composable nodes** in ROS 2 typically means:

- A **container process** hosts multiple nodes in-process.
- Nodes are **loaded/unloaded at runtime** via a standard ROS-facing service API.
- The ROS CLI and launch system interact with the container as a **service client**.

In C++, the reference container is `rclcpp_components::ComponentManager`.

---

## Names & Interfaces

A composition container exposes services in its node namespace (typically the container node name):

- `/<container>/load_node`
- `/<container>/unload_node`
- `/<container>/list_nodes`

> Some distros also expose batched variants (`load_nodes`) and/or `list_node_types`.
> This spec treats those as **optional** unless your target tooling requires them.

### Required services (baseline tool parity)

- `load_node` — `composition_interfaces/srv/LoadNode`
- `unload_node` — `composition_interfaces/srv/UnloadNode`
- `list_nodes` — `composition_interfaces/srv/ListNodes`

The C++ component manager exposes these services (baseline reference). :contentReference[oaicite:0]{index=0}

### Service payload expectations (semantic)

**LoadNode request** (conceptual semantics; exact fields are defined by the ROS interface):
- Identify which component to load (e.g. plugin / class identifier).
- Provide:
  - node name / namespace
  - remap rules
  - parameters (and/or parameter overrides)
  - extra arguments
- Response includes:
  - acceptance / error string
  - a unique identifier for the loaded node instance

**ListNodes response**:
- returns the set of currently loaded nodes and their unique IDs. :contentReference[oaicite:1]{index=1}

**UnloadNode request**:
- requests unloading by unique ID; response indicates success/failure with reason.

---

## Required Semantics (Normative)

### 1) Tool compatibility contract

A container is “tool compatible” if:

- `ros2 component list -c <container>` succeeds (maps to `list_nodes`)
- `ros2 component load -c <container> ...` succeeds (maps to `load_node`)
- `ros2 component unload -c <container> <id>` succeeds (maps to `unload_node`)

(Exact CLI flags vary by distro, but the behavior is the same: CLI is a service client.)

### 2) Load behavior

When `load_node` returns success:

- The node becomes observable on the ROS graph (topics/services/etc.) within a bounded time.
- `list_nodes` includes it.
- Unloading it removes it from `list_nodes` and releases resources deterministically.

If `load_node` is rejected:

- The node must not partially appear (no “ghost node” in list_nodes).
- The response must include a human-meaningful error string.

### 3) Unload behavior

When `unload_node` returns success:

- The node is no longer listed by `list_nodes`.
- All of its ROS entities are destroyed (publishers/subscriptions/services/timers), and it stops participating in the graph.

If rejected:

- No state change occurs.

### 4) ID stability

- Each loaded node instance receives a unique ID.
- IDs must remain stable until unloaded.
- Reuse of IDs is not allowed within a single container lifetime (recommended).

### 5) Concurrency + determinism

- Concurrent `load_node`/`unload_node` requests must be handled deterministically:
  - Either serialize them, or reject when busy.
- If “busy rejection” exists, it must not mutate container state.

### 6) Parameter and remap semantics

- Remaps and parameter overrides supplied at load time must apply to the loaded node instance.
- Any limitations (e.g., “parameter overrides supported but descriptors not”) must be documented in adapter parity docs.

---

## Dynamic loading caveat (Informative, but important)

The ROS-facing composition contract is service-based; **dynamic loading** is an implementation detail.

A container can be tool-compatible even if it doesn’t load `.so`/`.dll` plugins,
as long as it implements the same service semantics.

However, “full rclcpp parity” usually implies runtime loading of components.
That requires a stable ABI boundary:

- C++ has a stable-enough ABI story for this use-case (with conventions).
- Rust **does not** have a stable Rust-to-Rust ABI across `cdylib`.

So “load arbitrary components at runtime” in Rust typically implies one of:
- a minimal `extern "C"` factory seam, or
- a process-level composition alternative (spawn nodes as subprocesses), which is *not* in-process composition.

---

## Definition of Done

Composition parity is complete when:

- `list_nodes`, `load_node`, and `unload_node` exist with correct types and names
- `ros2 component` CLI works end-to-end against the container
- Load/unload is deterministic and resource-clean
- Parameters/remaps supplied at load behave as users expect
- Deviations (if any) are explicit and tested
