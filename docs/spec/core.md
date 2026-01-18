# Core Parity Spec (ROS 2 Canonical)

This document defines the *canonical* ROS 2 "core" behavior required for a
professional-grade client stack. It is transport-agnostic and focuses on
**observable behavior** (tools + operators + other nodes), not API shape.

Related implementation matrices:
- `docs/adapters/ros2rust/core/parity.md`
- `docs/adapters/roslibrust/core/parity.md`

---

## Scope

Core covers:
- init/shutdown and context lifetime
- node identity (name/namespace), remapping, ROS arguments
- clocks/time and time sources
- logging and loggers (incl. rosout expectations)
- discovery participation and graph visibility (minimal expectations)
- thread/task policy (no hidden execution)
- error model expectations (no panics as control flow)

Out of scope:
- lifecycle, parameters, actions (separate specs)
- executor scheduling details (separate spec)
- composition/container services (separate spec)

---

## Required Semantics (Normative)

## 1) Context and shutdown

- A process MUST explicitly initialize ROS context before creating nodes.
- Shutdown MUST be:
  - bounded (spin loops unblock)
  - idempotent (safe to call multiple times)
  - safe under partial initialization

Observable behavior:
- Ctrl+C / SIGINT results in a clean exit (no "second Ctrl+C" requirement).
- After shutdown, node interfaces stop responding and resources are released.

---

## 2) Node identity and namespacing

- Nodes MUST have a name and namespace consistent with ROS rules.
- Fully qualified names MUST be reflected consistently in:
  - services/topics created by the node
  - parameter/lifecycle/action endpoints (where applicable)
- Node name validation MUST reject invalid names deterministically.

---

## 3) ROS arguments and remapping

The stack MUST support standard ROS argument behaviors:
- name/namespace configuration via args or runtime config
- remapping rules for topic/service names (where supported by transport)
- consistent behavior with `ros2 run ... --ros-args ...`

If a transport cannot implement true remapping (e.g., rosbridge),
the limitation MUST be documented.

---

## 4) Time and clocks

A professional stack MUST expose:
- steady/system time for timers and durations
- ROS time (sim time) support where possible:
  - respect `use_sim_time` parameter if supported
  - provide a node clock that reflects ROS time when enabled

Observable behavior:
- tools and logs show consistent timestamps
- timers behave predictably under sim time (where supported)

If ROS time is not supported by a transport, it MUST be explicit.

---

## 5) Logging and observability

- Logging MUST be available per-node via a logger.
- Logs MUST include:
  - node name (and ideally namespace)
  - severity
  - timestamp
- The stack SHOULD integrate with ROS log conventions where feasible:
  - `rosout` topic publication (optional but commonly expected)
  - severity mapping consistent with ROS tools

No "log spam" surprises:
- suppression/gating should be silent unless explicitly requested.

---

## 6) Discovery and graph participation

Core parity requires that a node:
- participates in ROS discovery such that:
  - its services/topics can be discovered by other nodes/tools
- does not rely on non-standard graph APIs (unless clearly optional)

Note:
Graph introspection convenience APIs are not required for core parity,
but observable participation in discovery is.

---

## 7) Thread/task policy (normative)

- No hidden background spinning threads/tasks are started implicitly.
- If background threads/tasks are required for transport I/O, they MUST be:
  - explicitly constructed, OR
  - explicitly documented with lifecycle and shutdown behavior.

---

## 8) Error model (normative)

- Library code MUST NOT panic as control flow.
- Errors should be returned as typed results.
- If a panic can occur due to upstream/library behavior,
the adapter must document where and how to mitigate it.

---

## Definition of Done

Core parity is complete when:
- node init/shutdown is deterministic and bounded
- name/namespace/args behave consistently with ROS tooling
- time/logging are usable operationally
- discovery works and endpoints are visible to tools
- background execution is explicit and documented
