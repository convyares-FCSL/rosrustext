# Ecosystem Parity – rosrs (rclrs) Adapter

This document tracks ecosystem-level parity for the rosrs adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:
- `docs/spec/ecosystem.md`

---

### Feature requirements

Spec feature flag: `ecosystem`

Normative meaning:
- Defines “production ecosystem parity” expectations: packaging, reproducibility, CI split, docs.rs viability, and ROS-tool compatibility test posture.
- Not a runtime feature: it is a delivery/operability contract.

Required (to claim ecosystem parity for an adapter):
- `core`
- Documented build modes:
  - ROS-free (docs/unit tests)
  - ROS-native (integration/system tests)

Optional (adapter-defined, must be explicit):
- crates.io publishability constraints and workaround strategy (vendored msg crates vs generator vs proxy tool)
- supported ROS distros matrix

Notes:
- Ecosystem parity MUST state exactly what is publishable to crates.io and what is dev_ws-only.

---

## Status summary

**Ecosystem parity is achievable and mostly within scope**, provided the stack:

- stays native to rcl/rmw
- avoids transport bridges
- prioritizes tooling compatibility over API novelty

Most remaining work is **integration polish**, not missing primitives.

---

## Parity matrix (rosrs)

| Area | Status | Notes |
|---|---:|---|
| ROS CLI compatibility | ✅ | lifecycle/param tools validated; actions pending |
| Launch integration | ✅ | Rust binaries usable from Python launch |
| Lifecycle managers (Nav2) | ✅ | Bond + transitions validated |
| Cross-language interop | ✅ | Native DDS/rmw participation |
| QoS defaults | ✅ | rclrs defaults align with rclcpp |
| Composition support | ⚠️ | Container possible; plugin loading requires C-ABI seam |
| Build on crates.io | ⚠️ | Blocked by ROS msg crates availability |
| Build in ROS workspace | ✅ | colcon + Cargo integration validated |
| Unit test separation | ✅ | Pure Rust tests ROS-free |
| Integration testing | ✅ | CLI + launch tests in dev workspace |
| Shutdown semantics | ⚠️ | Executor-dependent; must document guarantees |

---

## Known ecosystem gaps

| Gap | Impact | Plan |
|---|---|---|
| Action parity incomplete | High | Implement canonical action surface + lifecycle gating |
| Composition container | Medium | Optional; implement service surface first |
| ROS msg crates on crates.io | Medium | Continue using generator + patch strategy |
| Executor shutdown guarantees | Low | Document best practices; avoid adapter-owned threads |

---

## Explicit non-goals

- rosbridge / roslibrust parity
- web-native tooling
- custom orchestration protocols
- replacing ROS launch system

---

## Definition of Done (rosrs ecosystem)

The rosrs ecosystem reaches parity when:

- Rust nodes are first-class citizens in ROS launch files
- Lifecycle managers treat Rust nodes identically to C++ nodes
- Actions, parameters, and lifecycle all work via standard CLI tools
- CI cleanly separates crate-level and ROS-level validation
- Operators do not need to know or care that a node is written in Rust
