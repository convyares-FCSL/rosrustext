# rosrustext_core

ROS-agnostic lifecycle semantics and state-machine logic used by the adapter crates.
This crate contains no ROS messages, executors, or transport-specific code.

## Scope

- Lifecycle state machine, transitions, and error handling
- Activation gating semantics
- Deterministic unit tests for lifecycle behavior

## Test

```bash
cargo test -p rosrustext_core
```
