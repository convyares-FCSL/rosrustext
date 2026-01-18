# Composition Parity – rosrs (rclrs) Adapter

This document tracks ROS 2 **composition (components)** parity for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:
- `docs/spec/composition.md` (normative)

This file answers:
> "Given the ROS2 composition spec, what does the rosrs adapter provide?"

---
### Feature requirements

Spec feature flag: `composition`

Normative meaning:
- Defines ROS2 composition/container behavior as observed through canonical composition management services.
- Focuses on runtime node loading/unloading/listing semantics and compatibility with ROS tooling.

Required (to claim composition parity in an adapter):
- `core`
- Adapter feature that provides the composition service surface (adapter-defined)

Optional (adapter-defined, must be explicit):
- Dynamic loading support (runtime load from shared library / plugin system)
- Discovery mechanism (ament index / registry integration)

Notes:
- The spec must separate:
  1) ROS-facing container service contract (tooling compatibility)
  2) implementation mechanism (static linking vs plugin loading)

---

## Status summary

**Not implemented yet** in `rosrustext_rosrs`.

However: composition is feasible in Rust at the ROS interface level because the
container contract is service-based and can be implemented on top of `rclrs`.

The C++ reference service set is exposed by `rclcpp_components::ComponentManager`
(`load_node`, `unload_node`, `list_nodes`). :contentReference[oaicite:2]{index=2}

---

## Tool Parity (External behavior)

### Services

| Service | ROS Type | Status | Notes |
|---|---|---:|---|
| `/<container>/load_node` | `composition_interfaces/srv/LoadNode` | ❌ | No container implementation yet |
| `/<container>/unload_node` | `composition_interfaces/srv/UnloadNode` | ❌ | — |
| `/<container>/list_nodes` | `composition_interfaces/srv/ListNodes` | ❌ | — |

### Behavioral semantics

| Aspect | Status | Notes |
|---|---:|---|
| CLI compatibility (`ros2 component`) | ❌ | Requires the 3 services above |
| Deterministic load/unload | ❌ | — |
| Load-time remaps + parameters | ❌ | Depends on LoadNode request support |
| Unique ID allocation | ❌ | Must match interface semantics |

---

## User Parity (Developer experience)

| Aspect | Status | Notes |
|---|---:|---|
| Authoring components as Rust types | ⚠️ | Rust can author node “factories”, but runtime plugin ABI is non-trivial |
| In-process multi-node container | ❌ | Not yet implemented |
| “Load from shared library” parity | ⚠️ | Likely needs an `extern "C"` factory seam or alternative strategy |

---

## Constraints / Design notes

- **Service contract is the primary requirement** for tool parity; the container’s internal mechanism (plugins vs registry vs static linking) can vary.
- “Load from `.so` at runtime” is optional for baseline tool parity, but required for “rclcpp-like UX parity”.
- For crates.io: ROS interface crates (composition_interfaces) must be provided via the existing generator / pre-generated crates approach, consistent with your current pattern.

---

## Definition of Done (rosrs composition)

Tool parity is complete when a Rust container node built on `rclrs` can be:

- driven by `ros2 component list/load/unload`
- deterministic and leak-free under repeated load/unload
- parameter/remap compatible with common launch workflows
