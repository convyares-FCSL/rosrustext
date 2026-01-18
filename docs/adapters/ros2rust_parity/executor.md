# Executor Parity – rosrs (rclrs) Adapter

This document tracks executor/spinning parity for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:
- `docs/spec/executors.md` (normative)

---

### Feature requirements

Spec feature flag: `executor`

Normative meaning:
- Defines execution/spinning expectations, ownership model, shutdown/cancellation, and interaction with lifecycle/actions/parameters.
- Transport-agnostic: it describes *observable scheduling behavior* and correctness constraints.

Required (to claim executor parity in an adapter):
- `core`
- Adapter feature enabling an executor model (adapter-defined)

Optional (adapter-defined, must be explicit):
- Multi-threaded execution mode
- Deterministic callback ordering guarantees (if provided)
- Async runtime dependency (e.g., tokio) if the adapter requires it

Notes:
- If an adapter inherently depends on an async runtime, that is an explicit limitation and must be documented in parity docs.

---

## Status summary

**Baseline parity is achievable and mostly present**, because `rclrs` provides an explicit
executor model and does not require hidden background spinning.

Key risk areas are:
- multi-threaded execution parity vs rclcpp
- shutdown/unblock behavior under service callbacks + lifecycle transitions
- deadlock hazards if user code blocks inside callbacks

---

## Tool Parity (External behavior)

Executors are not directly visible to ROS CLI tools, but they are visible via:
- whether services respond
- whether timers fire
- whether shutdown is clean
- whether lifecycle transitions complete deterministically

### Required semantics mapping

| Spec requirement | Status | Notes |
|---|---:|---|
| Explicit ownership (no hidden spinners) | ✅ | Application constructs executor and calls `spin` |
| Spin forever | ✅ | `Executor::spin(SpinOptions)` |
| Spin once / step | ⚠️ | Depends on what `rclrs` exposes (some patterns emulate via timeout) |
| Spin with timeout | ✅/⚠️ | If `SpinOptions` supports timeouts; otherwise emulate with non-blocking loops |
| Bounded shutdown | ✅/⚠️ | Context shutdown should unblock; needs integration test coverage |
| Concurrency model documented | ⚠️ | rclrs executor model must be stated (single-threaded vs multi) |
| Callback failure policy | ⚠️ | Rust panic behavior must be documented; prefer error returns |
| Lifecycle interaction no deadlocks | ✅/⚠️ | rosrustext lifecycle uses gating + bounded service handling; needs stress tests |

---

## Known gaps / risks (rosrs)

- If `rclrs` lacks a true `spin_once`, adapters must provide a tested pattern for bounded stepping.
- If multi-threaded executors are missing or limited, document as “single-thread only” parity.
- Ensure lifecycle transition handling does not depend on re-entrant spinning.

---

## Definition of Done (rosrs executors)

- Document supported executor models (single vs multi).
- Add integration tests for:
  - shutdown unblocks spin
  - timer tick + service response under load
  - lifecycle transitions completing without deadlocks
