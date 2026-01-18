# Executor Parity Spec (ROS 2 Canonical)

This document defines the *canonical* ROS 2 **executor / callback scheduling** surface area
and semantics. It is transport-agnostic and focuses on **observable behavior**:
determinism, shutdown semantics, concurrency, and callback isolation.

Related implementation matrices:
- `docs/adapters/ros2rust/executors/parity.md`
- `docs/adapters/roslibrust/executors/parity.md`

---

## Terminology

- **Executor**: owns the event loop that waits for work and dispatches callbacks.
- **Callback**: any user function invoked by the framework (subscription, timer, service, action).
- **Spinning**: running the executor event loop.
- **Wait set**: the underlying mechanism used to wait on ready entities (conceptual; transport-specific).

---

## Scope

In scope:
- single-threaded and multi-threaded execution models
- bounded shutdown and cancellation behavior
- callback ordering guarantees (where applicable)
- preventing hidden threads / surprise background spinners
- lifecycle interaction expectations (gating must not deadlock)

Out of scope:
- QoS tuning (handled elsewhere)
- component container composition (separate spec)
- real-time guarantees (not required for parity)

---

## Required Semantics (Normative)

### 1) Explicit executor ownership (no hidden spinners)

A professional ROS stack MUST make executor ownership explicit:

- The application (or container) owns the executor.
- Libraries must not start background spinning threads implicitly.
- If any background threads/tasks are required, they MUST be:
  - explicitly constructed by the user, or
  - explicitly documented and controllable.

Observable requirement:
- A node must not “do work” unless the executor is being spun.

---

### 2) Spin modes

At minimum, the stack MUST support:

- **Spin forever**: block and dispatch callbacks until shutdown.
- **Spin once / step**: process a bounded unit of work and return.
- **Spin with timeout**: wait for work up to a duration and return.

These modes enable deterministic integration tests and integration in larger apps.

---

### 3) Shutdown semantics (bounded, deterministic)

Shutdown must be deterministic and bounded:

- A shutdown request must unblock any spin loop within a bounded time.
- Shutdown must be idempotent (safe to call multiple times).
- Dropping handles must not deadlock waiting threads.

Observable requirement:
- A node process must exit cleanly without needing “Ctrl+C twice”.

---

### 4) Callback concurrency model (declared and consistent)

The stack MUST define (and adapters MUST document):

- Whether callbacks can run concurrently.
- If multi-threading exists, whether per-entity ordering is preserved.
- Whether a single callback can starve others.

Canonical expectations (rclcpp-like):
- Single-thread executor: callbacks never run concurrently.
- Multi-thread executor: callbacks may run concurrently; ordering is best-effort except where specified.

---

### 5) Callback isolation and failure policy

Callback panics/crashes must not silently corrupt the executor:

- Panics should be contained (prefer return errors over unwinding).
- If a callback failure terminates the executor, it must be explicit and logged.
- Long-running callbacks must not permanently block shutdown (document policy).

---

### 6) Lifecycle interaction (must not deadlock)

Lifecycle transitions often create/destroy entities and may run callbacks.
Executor + lifecycle must obey:

- A lifecycle transition must not require re-entrantly spinning the same executor.
- If transitions are service-driven, transition handling must not block forever.
- Busy/in-flight transition rejection must remain deterministic under executor load.

---

### 7) Testing support (normative)

Executor semantics must be testable:

- Provide a “step/spin_once” capability or an equivalent bounded operation.
- Enable deterministic test harnesses for:
  - timer tick behavior
  - service request/response handling
  - lifecycle transitions and busy rejection

---

## Definition of Done

Executor parity is complete when:

- No work happens without explicit spinning.
- Shutdown is bounded and repeatable.
- The concurrency model is documented and consistent.
- Lifecycle transitions and gated entities do not deadlock the executor.
- There are ROS-native integration tests that validate:
  - spin/shutdown behavior
  - transition + executor interaction under load
