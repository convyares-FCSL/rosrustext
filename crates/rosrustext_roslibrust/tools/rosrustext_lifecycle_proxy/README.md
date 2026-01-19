# rosrustext_lifecycle_proxy

**ROS 2 lifecycle service and event proxy for non-native Rust transports.**

`rosrustext_lifecycle_proxy` is a small tool crate used by the `rosrustext`
project to expose **canonical ROS 2 lifecycle behavior** in environments where
the underlying Rust transport does not natively provide the full lifecycle
service surface.

It exists to support **tooling and ecosystem parity**, not as a general-purpose
node framework.

---

## What this proxy does

The proxy provides a ROS-facing lifecycle surface compatible with standard tools
and managers, including:

- `/<node>/change_state`
- `/<node>/get_state`
- `/<node>/get_available_states`
- `/<node>/get_available_transitions`
- `/<node>/transition_event`
- Optional `/bond` heartbeats for Nav2 compatibility (feature: `bond`)

It forwards lifecycle intent and state between ROS tools and a Rust-managed
lifecycle state machine implemented elsewhere (typically in
`rosrustext_core` + an adapter).

---

## Why this exists

Some Rust ROS transports (notably rosbridge-based stacks) do not expose:

- lifecycle services as first-class primitives
- lifecycle event topics
- Nav2-compatible `/bond` behavior

Rather than re-implement lifecycle semantics in every adapter or lesson, the
proxy allows:

- **one canonical lifecycle surface**
- **one place to satisfy ROS tooling expectations**
- reuse across transports that lack native lifecycle support

This keeps lifecycle semantics **centralized, testable, and observable**.

---

## Intended usage

This crate is:

- ✅ Used internally by `rosrustext_roslibrust`
- ✅ Used in system-level tests that exercise ROS tooling
- ❌ Not intended as a general user-facing library
- ❌ Not a replacement for native lifecycle support when available

End users typically do **not** depend on this crate directly.

---

## Features

- `bond` (disabled by default): include `bond/msg/Status` support and publish `/bond`
  heartbeats for Nav2. Disable at runtime with `--no-bond` or `ROSRUSTEXT_BOND=0`.

---

## Stability and scope

This proxy:

- Implements **ROS-facing lifecycle behavior only**
- Does not define lifecycle semantics (those live in `rosrustext_core`)
- Avoids graph introspection or rosapi dependencies
- Is deliberately minimal and explicit

The exact scope of this proxy (lifecycle-only vs more general management proxy)
is under active review.

---

## Relationship to other crates

- `rosrustext_core`  
  Defines lifecycle semantics and state machine.

- `rosrustext_roslibrust`  
  Uses this proxy to project lifecycle behavior into rosbridge-based systems.

- `rosrustext_rosrs`  
  Does **not** use this proxy; lifecycle is implemented natively via `rclrs`.

---

## Future direction

As Rust ROS transports mature, this crate may:

- remain lifecycle-only,
- evolve into a more generic ROS management proxy, or
- be retired where native lifecycle parity is available.

Any such change will be driven by **observable parity requirements**, not API
convenience.

---

## License

Apache-2.0
