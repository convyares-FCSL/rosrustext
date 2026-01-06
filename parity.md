# ROS 2 Lifecycle Parity Milestone

**Status**: Achieved
**Date**: 2026-01-06

This document certifies that `rosrustext` has achieved full lifecycle parity with standard ROS 2 tooling, validated against:
- `ros2 lifecycle` CLI
- `nav2_lifecycle_manager` (Jazzy)
- Python `lifecycle_py`

## Nav2 Compatibility Requirements (Normative)

To satisfy the strict requirements of `nav2_lifecycle_manager`, the implementation adheres to the following rules:

### 1. Transition State IDs
Intermediate transition states **must** map to standard ROS 2 IDs, not `0`.
- `Configuring` = 10
- `CleaningUp` = 11
- `ShuttingDown` = 12
- `Activating` = 13
- `Deactivating` = 14
- `ErrorProcessing` = 15

### 2. Bond Timing
The `/bond` status **must** be published as `active=true` **immediately** upon entering the `Active` state.
- **Rule**: On `inactive -> active` edge, publish `BondStatus { active: true }` immediately.
- **Rationale**: Nav2 checks connectivity synchronously after the `Activate` transition returns. Waiting for a periodic heartbeat causes a race condition that fails the smoke test.

### 3. Synchronous Execution
The `ChangeState` service **must** execute synchronously when `delay=0` (default).
- **Behavior**: The transition callback (e.g., `on_configure`) runs, and the state is updated **before** the service response is sent.
- **Rationale**: The lifecycle manager polls `get_state` immediately after `change_state` returns. Async execution (classic "fire and forget") leads to race conditions where the manager sees `Unconfigured` (old state) despite a `success=true` response.
