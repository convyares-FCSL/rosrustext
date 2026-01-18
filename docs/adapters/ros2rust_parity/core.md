# Core Parity – rosrs (rclrs) Adapter

This document tracks ROS 2 core parity for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:
- `docs/spec/core.md` (normative)

---

### Feature requirements

Spec feature flag: `core`

Normative meaning:
- The semantic model exists in `rosrustext_core` and is transport-agnostic.
- No ROS message crates, no ROS runtime, no executors, no async runtime assumptions.

Required (library build):
- None (must build with `--no-default-features` everywhere it is referenced)

Forbidden dependencies:
- `rclrs`, `roslibrust`, ROS message crates, `tokio` (unless explicitly part of *core* semantics, which it should not be)
docs/spec/lifecycle.md

## Status summary

**Core parity is largely delegated to `rclrs` and the underlying rcl/rmw layer.**
rosrustext_rosrs should:
- avoid introducing hidden threads
- document any deviations (time/logging/rosout, remapping edge cases)
- provide small ergonomics helpers only where they do not alter semantics

---

## Parity matrix (rosrs)

| Core area | Status | Notes |
|---|---:|---|
| Context init/shutdown | ✅ | `rclrs::Context` + executor model; bounded shutdown depends on executor behavior |
| Node identity (name/namespace) | ✅ | Enforced by rcl/rclrs; invalid names should error deterministically |
| ROS args + remapping | ✅/⚠️ | Depends on `rclrs` arg plumbing; document supported flags/remaps |
| Logging baseline | ✅ | rclrs logger macros usable; structured logging is adapter responsibility (ecosystem spec) |
| rosout publication | ⚠️ | If not provided by rclrs, document; optional for core but common expectation |
| Time/clocks (steady/system) | ✅ | Timers/durations supported |
| ROS time / sim time | ⚠️ | Depends on rclrs clock + /clock handling; document current behavior |
| Discovery participation | ✅ | Provided by rmw via rclrs node creation |
| No hidden background execution | ✅ | Executor is app-owned; rosrustext must keep it that way |
| Error model (no panics) | ✅/⚠️ | Adapter code should not panic; document any upstream panics if present |

---

## Known gaps / action items (rosrs)

- Document remapping support explicitly (which `--ros-args` forms are supported).
- Document ROS time / use_sim_time behavior (supported vs not).
- Decide whether rosout is treated as "core optional" or "ecosystem required".

---

## Definition of Done (rosrs core)

- A rosrustext_rosrs node behaves like a normal ROS 2 node under:
  - name/namespace/args
  - discovery
  - logging
  - time
  - bounded shutdown
with deviations explicitly documented and tested where feasible.
