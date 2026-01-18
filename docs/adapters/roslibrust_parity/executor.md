# Executor Parity – roslibrust Adapter

This document tracks executor/spinning parity for the roslibrust transport adapter
(rosbridge/websocket-based).

Canonical reference:
- `docs/spec/executors.md` (normative)

---

## Status summary

roslibrust does not use the ROS wait-set/executor model.
Instead, execution is typically driven by an async runtime (tokio) and websocket I/O tasks.

So parity must be defined as:
- explicit ownership of the runtime
- no hidden background tasks unless documented
- bounded shutdown
- deterministic callback dispatch policy

---

## Spec mapping

| Spec requirement | Status | Notes |
|---|---:|---|
| Explicit ownership (no hidden spinners) | ⚠️ | Depends on library design: many websocket clients spawn internal tasks |
| Spin forever | ⚠️ | Equivalent is “run tokio runtime / await tasks” |
| Spin once / timeout | ⚠️ | Equivalent via select!/timeouts; must be provided as patterns/helpers |
| Bounded shutdown | ⚠️ | Must close websocket + cancel tasks deterministically |
| Concurrency model documented | ⚠️ | Tokio tasks can run concurrently; ordering depends on channels/queues |
| Lifecycle interaction no deadlocks | ⚠️ | Must avoid blocking service handlers; prefer async state machine |

---

## Recommended stance

- Treat executor parity for roslibrust as “runtime/task model parity” rather than ROS executor parity.
- Ensure the adapter:
  - does not spawn hidden tasks without explicit API
  - exposes a clean shutdown method that cancels all background tasks
  - documents callback ordering guarantees (if any)

---

## Definition of Done (roslibrust executors)

- Provide explicit runtime ownership patterns.
- Provide deterministic shutdown (close websocket + stop tasks).
- Document callback dispatch and ordering.
