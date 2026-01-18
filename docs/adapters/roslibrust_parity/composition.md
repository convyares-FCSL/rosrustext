# Composition Parity – roslibrust Adapter

This document tracks ROS 2 **composition (components)** parity for the roslibrust transport adapter
(rosbridge/websocket-based).

Canonical reference:
- `docs/spec/composition.md` (normative)

---

## Status summary

**Not a realistic target for roslibrust** beyond “tooling-shaped imitation”.

Reason: the canonical composition contract assumes a container can create and destroy
ROS nodes in-process and expose their ROS entities directly. Over rosbridge, you are
not a native participant in the RCL graph in the same way, and “in-process node loading”
doesn’t map cleanly.

A roslibrust “container” can *simulate* the services, but it will not be a true ROS-native
composition container unless backed by a native bridge/proxy that actually loads nodes.

---

## Tool Parity (External behavior)

| Surface | Status | Notes |
|---|---:|---|
| `load_node` / `unload_node` / `list_nodes` services | ⚠️ | Only achievable via a native proxy that implements composition on behalf of the websocket client |
| `ros2 component` CLI compatibility | ⚠️ | Same as above; CLI needs real services with correct behavior |

---

## Recommended stance

- Treat **composition parity as a rosrs/rclrs goal** (native RCL participant).
- For roslibrust, keep the focus on:
  - lifecycle tool parity via proxy
  - parameter tool parity via proxy
  - “app-level composition” (multiple tasks/actors) rather than ROS-native component composition

---

## Definition of Done (roslibrust composition)

If you pursue it at all: parity is “complete” only when a native proxy provides
the real ROS composition services and roslibrust is merely a client.
